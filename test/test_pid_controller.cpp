#include <unity.h>
#include <PidController.h>
#include <DualMotorDriver.h>
#include <thread>

Encoder encoder;
PidController* pid_controller{nullptr};

static constexpr float kP = 10.;
static constexpr float kI = 0.;
static constexpr float kD = 0.;

// A NOTE ON TIMING
// We have a few competing timing concerns here. One the one hand, we have some elements of the
// PID controller that rely on the nominal PID timing (e.g. speed ramping), and some that rely
// on actual time elapsed (speed calculation). For these unit tests, we are basically evaluating
// the PID controller by setting up more or less steady state conditions; the integral gain test
// being one exception where we look at accumulated error over a few iterations. Because of this
// mismatch between the nominal real-time PID loop time and the actual time that we run the PID
// loop at, the test is not exactly representative of the operating conditions. That said, I 
// believe the tests are still effective in creating representative conditions and verifying the
// logical implementation of the controller.

void pid_test_setup()
{
    pid_controller = new PidController(kP, kI, kD, &encoder);
}

void pid_test_teardown()
{
    if (pid_controller != nullptr)
        delete pid_controller;
}

// set encoder counts and update PID controller to get to a specific
// value for the current speed, and return the control signal
float set_speed_and_run_controller(float measured_speed_cps, size_t num_iterations = SPEED_SAMPLE_BUFFER_LENGTH + PWM_SAMPLE_BUFFER_LENGTH)
{
    int32_t counts = encoder.get_count();
    
    // we'll use a long time constant to give better granularity in the encoder delta 
    // per iteration
    constexpr float time_delta_ms = 100.;

    // the desired speed is in counts per second
    // speed = counts / time_delta_ms
    // counts = speed (counts/sec) * time_delta_ms (ms) * 1 sec / 1000 ms
    int32_t count_delta = static_cast<int32_t>(measured_speed_cps * time_delta_ms / 1000.);
    int32_t final_encoder_count = counts + static_cast<int32_t>(measured_speed_cps * (num_iterations+1) * time_delta_ms / 1000.);

    auto last_sleep_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < num_iterations; i++)
    {
        counts += count_delta;
        encoder.set_count(counts);
        std::this_thread::sleep_until(last_sleep_time + std::chrono::milliseconds(static_cast<int>(time_delta_ms)));
        last_sleep_time = std::chrono::high_resolution_clock::now();
        pid_controller->run_controller();
    }

    // we'll run one more iteration to make sure we get the total number of encoder counts
    encoder.set_count(final_encoder_count);
    std::this_thread::sleep_until(last_sleep_time + std::chrono::milliseconds(static_cast<int>(time_delta_ms)));

    return pid_controller->run_controller();
}

void test_pid_type(void)
{
    TEST_ASSERT_EQUAL(kP, pid_controller->get_kP());
    TEST_ASSERT_EQUAL(kI, pid_controller->get_kI());
    TEST_ASSERT_EQUAL(kD, pid_controller->get_kD());
}

void test_pid_speed_calculation()
{
    float target_speed = -100.f;
    set_speed_and_run_controller(target_speed);

    // our accuracy is limited by the accuracy of the sleep time implemented in the fill buffer
    // function; by trial and error 0.3 works well for 100 counts per second
    TEST_ASSERT_FLOAT_WITHIN(0.3f, target_speed, pid_controller->get_current_speed_cps());

    target_speed = 100.f;
    set_speed_and_run_controller(target_speed);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, target_speed, pid_controller->get_current_speed_cps());
}

void test_open_loop_control_calculation()
{
    float target_speed_pos = 100.f;
    float control_signal{0.};

    float ramp_time_ms = target_speed_pos / PidController::SPEED_RAMP_RATE_CPS_PER_MS;

    pid_controller->set_mode(OPEN_LOOP);
    pid_controller->set_target_speed(target_speed_pos);
    control_signal = pid_controller->run_controller();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(ramp_time_ms+1)));
    control_signal = pid_controller->run_controller();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target_speed_pos*PidController::CONTROL_SPEED_RATIO, control_signal);

    float target_speed_neg = -100.f;
    ramp_time_ms = (target_speed_pos - target_speed_neg) / PidController::SPEED_RAMP_RATE_CPS_PER_MS;
    pid_controller->set_target_speed(target_speed_pos);
    control_signal = pid_controller->run_controller();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(ramp_time_ms+1)));
    control_signal = pid_controller->run_controller();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1*target_speed_neg*PidController::CONTROL_SPEED_RATIO, control_signal);
}

void test_proportional_closed_loop_control()
{
    float target_speed = 100.f;
    float actual_speed = 90.f;
    float control_signal(0.);
    float kP = 10.;
    pid_controller->set_kP(kP);
    pid_controller->set_kI(0.);
    pid_controller->set_kD(0.);
    pid_controller->set_kFF(0.);

    pid_controller->set_mode(CLOSED_LOOP);
    pid_controller->set_target_speed(target_speed);

    float ramp_time_ms = target_speed / PidController::SPEED_RAMP_RATE_CPS_PER_MS;
    uint32_t ramp_iterations = static_cast<uint32_t>(ramp_time_ms / PID_LOOP_TIME_MS);

    // the time constant on the update is the speed sample buffer time plus the PWM sample buffer time
    control_signal = set_speed_and_run_controller(actual_speed, ramp_iterations + SPEED_SAMPLE_BUFFER_LENGTH + PWM_SAMPLE_BUFFER_LENGTH);

    TEST_ASSERT_FLOAT_WITHIN(0.02*target_speed, kP*(target_speed - actual_speed), control_signal);
}

void test_integral_closed_loop_control()
{
    float target_speed = 102.f;
    float actual_speed = 100.f;
    float control_signal(0.);
    float kI = 1.;

    // start with zero PID gain so there is no accumulated control signal in the PWM buffer
    pid_controller->set_kP(0.);
    pid_controller->set_kI(0.);
    pid_controller->set_kD(0.);
    pid_controller->set_kFF(0.);

    pid_controller->set_mode(CLOSED_LOOP);

    float ramp_time_ms = actual_speed / PidController::SPEED_RAMP_RATE_CPS_PER_MS;
    uint32_t ramp_iterations = static_cast<uint32_t>(ramp_time_ms / PID_LOOP_TIME_MS);

    // set the target speed to the actual speed
    pid_controller->set_target_speed(actual_speed);

    // first we'll run the update loop to load the speed buffer with the actual speed
    // for integral gain, the relevant count is the filtered speed buffer length
    // we'll give a couple extra iterations to allow the filter buffers to fully catch up
    set_speed_and_run_controller(actual_speed, ramp_iterations + FILTERED_SPEED_BUFFER_LENGTH + 5);

    // now we'll set the actual target speed, which clears the accumulated error
    pid_controller->set_target_speed(target_speed);
    
    // set the integral gain and run the controller for a few iterations to accumulate error
    pid_controller->set_kI(kI);
    control_signal = set_speed_and_run_controller(actual_speed, 2);

    // we now have 3 iterations where the error is about (target_speed - actual_speed), so the accumulated
    // error is about 6*(target_speed - actual_speed), which is then filtered into the otherwise empty 
    // control signal buffer, since all gains were zero before
    float expected_control = 6*kI*(target_speed - actual_speed)/PWM_SAMPLE_BUFFER_LENGTH;

    // 10% seems to be about the best we can do consistently here
    TEST_ASSERT_FLOAT_WITHIN(0.1*expected_control, expected_control, control_signal);

    // now verify we don't exceed max error - we'll crank up the target speed so we have a large error, and
    // we'll need a long run time to let the error saturate and then the filtered control signal to catch up
    float target_speed_max_error = PidController::MAX_ACCUMULATED_ERROR/10;
    ramp_iterations = static_cast<uint32_t>(target_speed_max_error / PidController::SPEED_RAMP_RATE_CPS_PER_MS);
    pid_controller->set_target_speed(target_speed_max_error);
    control_signal = set_speed_and_run_controller(actual_speed, ramp_iterations + FILTERED_SPEED_BUFFER_LENGTH + 5);

    TEST_ASSERT_FLOAT_WITHIN(0.1, kI*PidController::MAX_ACCUMULATED_ERROR, control_signal);
}

// This test is effectively evaluating open-loop gain in closed loop mode
void test_feedforward_control()
{
    float target_speed_pos = 100.f;
    float control_signal{0.};
    float K_ff = 1.0;

    float ramp_time_ms = target_speed_pos / PidController::SPEED_RAMP_RATE_CPS_PER_MS;
    uint32_t ramp_iterations = static_cast<uint32_t>(ramp_time_ms / PID_LOOP_TIME_MS);

    pid_controller->set_kP(0.);
    pid_controller->set_kI(0.);
    pid_controller->set_kD(0.);
    pid_controller->set_kFF(K_ff);

    pid_controller->set_mode(CLOSED_LOOP);
    pid_controller->set_target_speed(target_speed_pos);

    // the time constant on the update is the speed sample buffer time plus the PWM sample buffer time
    control_signal = set_speed_and_run_controller(target_speed_pos, ramp_iterations + SPEED_SAMPLE_BUFFER_LENGTH + PWM_SAMPLE_BUFFER_LENGTH);

    float target_speed_neg = -100.f;
    ramp_time_ms = (target_speed_pos - target_speed_neg) / PidController::SPEED_RAMP_RATE_CPS_PER_MS;
    ramp_iterations = static_cast<uint32_t>(ramp_time_ms / PID_LOOP_TIME_MS);
    pid_controller->set_target_speed(target_speed_neg);

    // the time constant on the update is the speed sample buffer time plus the PWM sample buffer time
    control_signal = set_speed_and_run_controller(target_speed_neg, ramp_iterations + SPEED_SAMPLE_BUFFER_LENGTH + PWM_SAMPLE_BUFFER_LENGTH);

    // control signal should come back negative (negative control is transformed to positive PWM with reverse direction at
    // the motor controller layer)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target_speed_neg*PidController::CONTROL_SPEED_RATIO, control_signal);
    
}
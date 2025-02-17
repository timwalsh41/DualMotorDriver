#include <unity.h>
#include <PidController.h>
#include <thread>

Encoder encoder;
PidController* pid_controller{nullptr};

static constexpr float kP = 10.;
static constexpr float kI = 0.;
static constexpr float kD = 0.;

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
float set_speed_and_run_controller(float measured_speed_cps, size_t num_iterations = SPEED_SAMPLE_BUFFER_LENGTH)
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
    float target_speed = 100.f;
    float control_signal{0.};

    pid_controller->set_mode(OPEN_LOOP);
    pid_controller->set_target_speed(target_speed);
    control_signal = pid_controller->run_controller();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target_speed*PidController::CONTROL_SPEED_RATIO, control_signal);

    target_speed = -100.f;
    pid_controller->set_target_speed(target_speed);
    control_signal = pid_controller->run_controller();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target_speed*PidController::CONTROL_SPEED_RATIO, control_signal);
}

void test_proportional_closed_loop_control()
{
    float target_speed = 100.f;
    float actual_speed = 90.f;
    float control_signal(0.);
    float kP = 10.;
    pid_controller->set_kP(kP);
    pid_controller->set_kI(0);
    pid_controller->set_kD(0);

    pid_controller->set_mode(CLOSED_LOOP);
    pid_controller->set_target_speed(target_speed);

    control_signal = set_speed_and_run_controller(actual_speed);

    TEST_ASSERT_FLOAT_WITHIN(0.02*target_speed, kP*(target_speed - actual_speed), control_signal);
}

void test_integral_closed_loop_control()
{
    float target_speed = 102.f;
    float actual_speed = 100.f;
    float control_signal(0.);
    float kI = 1.;
    pid_controller->set_kP(0);
    pid_controller->set_kI(kI);
    pid_controller->set_kD(0);

    pid_controller->set_mode(CLOSED_LOOP);
    pid_controller->set_target_speed(0);

    // first we'll run the update loop to load the speed buffer with the actual speed
    set_speed_and_run_controller(actual_speed);

    // now we'll set the target speed, which clears the accumulated error
    pid_controller->set_target_speed(target_speed);
    
    // run the controller for a few iterations to accumulate error
    control_signal = set_speed_and_run_controller(actual_speed, 2);

    // 15% seems to be about the best we can do consistently here
    TEST_ASSERT_FLOAT_WITHIN(0.15*3*kI*(target_speed - actual_speed), 3*kI*(target_speed - actual_speed), control_signal);

    // now verify we don't exceed max error
    pid_controller->set_target_speed(2*target_speed);
    control_signal = set_speed_and_run_controller(actual_speed, 10);

    TEST_ASSERT_FLOAT_WITHIN(0.1, kI*PidController::MAX_ACCUMULATED_ERROR, control_signal);
}
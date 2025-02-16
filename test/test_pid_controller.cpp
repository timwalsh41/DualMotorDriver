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
// value for the current speed
void fill_buffer_for_speed(float speed_cps)
{
    int32_t counts = encoder.get_count();
    
    // we'll use a long time constant to give better granularity in the encoder delta 
    // per iteration
    constexpr float time_delta_ms = 100.;

    // the desired speed is in counts per second
    // speed = counts / time_delta_ms
    // counts = speed (counts/sec) * time_delta_ms (ms) * 1 sec / 1000 ms
    int32_t count_delta = static_cast<int32_t>(speed_cps * time_delta_ms / 1000.);
    int32_t final_encoder_count = counts + static_cast<int32_t>(speed_cps * (SPEED_SAMPLE_BUFFER_LENGTH+1) * time_delta_ms / 1000.);

    auto last_sleep_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < SPEED_SAMPLE_BUFFER_LENGTH; i++)
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
    pid_controller->run_controller();
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
    fill_buffer_for_speed(target_speed);

    // our accuracy is limited by the accuracy of the sleep time implemented in the fill buffer
    // function; by trial and error 0.3 works well for 100 counts per second
    TEST_ASSERT_FLOAT_WITHIN(0.3f, target_speed, pid_controller->get_current_speed_cps());

    target_speed = 100.f;
    fill_buffer_for_speed(target_speed);
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

void test_closed_loop_control_calculation()
{

}
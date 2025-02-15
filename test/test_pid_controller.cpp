#include <unity.h>
#include <PidController.h>

Encoder encoder;
PidController* pid_controller{nullptr};

void pid_test_setup()
{
    pid_controller = new PidController(10., 0., 0., &encoder);
}

void pid_test_teardown()
{
    if (pid_controller != nullptr)
        delete pid_controller;
}

void test_pid_type(void)
{

    TEST_ASSERT_EQUAL(1 + 1, 2);
}
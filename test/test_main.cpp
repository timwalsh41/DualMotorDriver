#include <unity.h>
#include "Arduino.h"

MockSerial Serial;

void setUp(void)
{
    pid_test_setup();
    serial_test_setup();
}

void tearDown(void)
{
    pid_test_teardown();
    serial_test_teardown();
}

void test_hello_world(void) {
    TEST_ASSERT_EQUAL(1 + 1, 2);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_hello_world);
    
    RUN_TEST(test_serial_type);
    RUN_TEST(test_set_motor_speed_command);
    RUN_TEST(test_invalid_command);
    RUN_TEST(test_invalid_payload);
    RUN_TEST(test_invalid_crc);
    RUN_TEST(test_enable_disable_driver);
    RUN_TEST(test_read_fault);
    RUN_TEST(test_read_clear_encoders);
    RUN_TEST(test_get_motor_speeds);
    RUN_TEST(test_set_control_mode);

    RUN_TEST(test_pid_type);
    RUN_TEST(test_pid_speed_calculation);
    RUN_TEST(test_open_loop_control_calculation);
    RUN_TEST(test_proportional_closed_loop_control);
    RUN_TEST(test_integral_closed_loop_control);

    return UNITY_END();
}
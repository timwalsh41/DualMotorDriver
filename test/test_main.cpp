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
    RUN_TEST(test_pid_type);
    RUN_TEST(test_serial_type);
    return UNITY_END();
}
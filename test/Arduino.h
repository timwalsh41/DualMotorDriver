#ifndef ARDUINO_H
#define ARDUINO_H

// Mock Arduino definitions

#include <cstdint>
#include <deque> // Include deque header
#include "stddef.h"

class MockSerial;
extern MockSerial Serial;

// PID controller tests
void pid_test_setup();
void pid_test_teardown();
void test_pid_type();
void test_pid_speed_calculation();
void test_open_loop_control_calculation();
void test_proportional_closed_loop_control();
void test_integral_closed_loop_control();
void test_feedforward_control();

// serial interface tests
void serial_test_setup();
void serial_test_teardown();
void test_serial_type();
void test_set_motor_speed_command();
void test_invalid_command();
void test_invalid_payload();
void test_invalid_crc();
void test_enable_disable_driver();
void test_read_fault();
void test_read_clear_encoders();
void test_get_motor_speeds();
void test_set_control_mode();
void test_set_get_pid_gains();
void test_get_pwm();

// Mocked Arduino functionality
unsigned long millis();
unsigned long micros();
void yield();

class MockSerial
{
    public:
        unsigned char read();
        void write(unsigned char*, size_t);
        void write(unsigned char);
        size_t available();

        void load_read_buffer(unsigned char*, size_t);
        size_t retrieve_write_buffer(unsigned char*, size_t buffer_size);

        void flush();

    private:
        std::deque<unsigned char> m_read_buffer; // Use deque for read buffer
        std::deque<unsigned char> m_write_buffer; // Use deque for write buffer
};

#endif
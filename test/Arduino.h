#ifndef ARDUINO_H
#define ARDUINO_H

// Mock Arduino definitions

#include <cstdint>
#include <deque> // Include deque header
#include "stddef.h"

class MockSerial;
extern MockSerial Serial;

void pid_test_setup();
void pid_test_teardown();
void test_pid_type();

void serial_test_setup();
void serial_test_teardown();
void test_serial_type();
void test_valid_command_sequence();
void test_invalid_command();
void test_invalid_payload();
void test_invalid_crc();

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

    private:
        std::deque<unsigned char> m_read_buffer; // Use deque for read buffer
        std::deque<unsigned char> m_write_buffer; // Use deque for write buffer
};

#endif
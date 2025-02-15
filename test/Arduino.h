#ifndef ARDUINO_H
#define ARDUINO_H

// Mock Arduino definitions

#include <cstdint>
#include "stddef.h"

class MockSerial;
extern MockSerial Serial;

void pid_test_setup();
void pid_test_teardown();
void test_pid_type();

void serial_test_setup();
void serial_test_teardown();
void test_serial_type();

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
};

#endif
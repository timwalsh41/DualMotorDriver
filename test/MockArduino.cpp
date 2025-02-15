#include "Arduino.h"
#include <chrono>

static const auto startTime = std::chrono::steady_clock::now();

unsigned long millis() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - startTime;
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
    );
}

unsigned long micros() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - startTime;
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()
    );
}

// Used in Rob Tillaart CRC library
void yield()
{

}

unsigned char MockSerial::read()
{
    return 0;
}

void MockSerial::write(unsigned char*, size_t)
{}

void MockSerial::write(unsigned char)
{}

size_t MockSerial::available()
{
    return 0;
}
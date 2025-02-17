#include "Arduino.h"
#include <chrono>
#include <deque>

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
    if (!m_read_buffer.empty())
    {
        unsigned char c = m_read_buffer.front();
        m_read_buffer.pop_front(); // Remove the first element
        return c;
    }
    return 0;
}

void MockSerial::write(unsigned char* buffer, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        m_write_buffer.push_back(buffer[i]); // Add elements to the end
    }
}

void MockSerial::write(unsigned char c)
{
    m_write_buffer.push_back(c); // Add element to the end
}

size_t MockSerial::available()
{
    return m_read_buffer.size();
}

void MockSerial::load_read_buffer(unsigned char* buffer, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        m_read_buffer.push_back(buffer[i]); // Add elements to the end
    }
}

size_t MockSerial::retrieve_write_buffer(unsigned char* buffer, size_t max_buffer_size)
{
    size_t size = m_write_buffer.size();

    if (size > max_buffer_size)
    {
        // we would overflow the buffer, just return 0
        return 0;
    }

    for (size_t i = 0; i < size; ++i)
    {
        buffer[i] = m_write_buffer[i];
    }
    
    m_write_buffer.clear(); // Clear the buffer after retrieval
    return size;
}

void MockSerial::flush()
{
    m_write_buffer.clear();
    m_read_buffer.clear();
}
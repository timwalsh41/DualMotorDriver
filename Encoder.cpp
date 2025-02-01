#include "Encoder.h"

// This function is meant to be very lightweight to act as the ISR for
// encoder channel rising and falling edges
void Encoder::update_count(bool channel_A, bool channel_B)
{
  if (channel_A)
  {
    if (!channel_B)
      // A is rising, B is low => forward direction
      m_count += 1;
    else
      // A is rising, B is high => reverse direction
      m_count -= 1;
  }
  else
  {
    if (channel_B)
      // A is falling, B is high => forward direction
      m_count += 1;
    else
      // A is falling, B is low => reverse direction
      m_count -= 1;
  }
}

int32_t Encoder::update_delta()
{
  int64_t delta = m_count - m_last_count;
  m_last_count = m_count;
  return delta;
}

void Encoder::reset_count()
{
  m_count = 0;
  m_last_count = 0;
}
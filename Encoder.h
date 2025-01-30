// #include <queue>

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

class Encoder
{
  public:
    Encoder() {}
    ~Encoder() {}

    // update_count takes in the current values on the encoder A and B channels and determines whether to
    // increment the count (forward direction) or decrement the count (reverse direction). This function
    // should be used as the ISR for rising and falling edges on the channel A input
    void update_count(bool channel_A, bool channel_B);

    // get_delta calculates the delta between the prior count and the current count. Note that for signed
    // integer types, rollover will be implicity handled here automatically as the subtraction across the
    // rollover boundary at the positive and negative limits will return the correct difference.
    int32_t get_delta();

    void reset_count();
    int64_t get_count() const {return m_count;}
    int64_t get_last_count() const {return m_last_count;}

  private:
    int32_t m_count{0};
    int32_t m_last_count{0};
};

#endif
#ifndef IENCODER_H
#define IENCODER_H

#include "stdint.h"

class IEncoder
{
    public:
        virtual ~IEncoder() {}

        // update_count takes in the current values on the encoder A and B channels and determines whether to
        // increment the count (forward direction) or decrement the count (reverse direction). This function
        // should be used as the ISR for rising and falling edges on the channel A input
        virtual void update_count(bool channel_A, bool channel_B) = 0;

        // get_delta calculates the delta between the prior count and the current count. Note that for signed
        // integer types, rollover will be implicity handled here automatically as the subtraction across the
        // rollover boundary at the positive and negative limits will return the correct difference.
        virtual int32_t update_delta() = 0;

        virtual void reset_count() = 0;
        virtual int64_t get_count() const = 0;
        virtual int64_t get_last_count() const  = 0;
        virtual void set_count(int32_t count) = 0;
};

#endif // IENCODER_H
#ifndef UTILITY_H
#define UTILITY_H

#include "stdint.h"

// Calculate elapsed time, accounting for integer rollover
uint32_t time_elapsed(uint32_t last_time, uint32_t current_time);

// Reinterpret 4 bytes as a float
float bytes_to_float(uint8_t *bytes);

// Reinterpret a float to 4 bytes
void float_to_bytes(float val, uint8_t* bytes);

// Reinterpret an int32_t type to 4 bytes
void int32_to_bytes(int32_t val, uint8_t* bytes);

#endif
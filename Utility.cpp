#include "Utility.h"
#include "stddef.h"

uint32_t time_elapsed(uint32_t last_time, uint32_t current_time)
{
  if (current_time < last_time)
    return UINT32_MAX - last_time + current_time + 1;
  else
    return current_time - last_time;
}

typedef union
{
  uint8_t bytes[4];
  float f;
  int32_t i;
} ConversionData;

float bytes_to_float(uint8_t *bytes)
{
  ConversionData data;

  for (size_t i = 0; i < 4; i++)
    data.bytes[i] = bytes[i];
  return data.f;
}

void float_to_bytes(float val, uint8_t* bytes)
{
  ConversionData data;

  data.f = val;

  for (size_t i = 0; i < 4; i++)
    bytes[i] = data.bytes[i];
}

void int32_to_bytes(int32_t val, uint8_t* bytes)
{
  ConversionData data;

  data.i = val;

  for (size_t i = 0; i < 4; i++)
    bytes[i] = data.bytes[i];
}
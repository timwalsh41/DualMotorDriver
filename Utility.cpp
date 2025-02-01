#include "Utility.h"

uint32_t time_elapsed(uint32_t last_time, uint32_t current_time)
{
  if (current_time < last_time)
    return UINT32_MAX - last_time + current_time + 1;
  else
    return current_time - last_time;
}

typedef union
{
  uint8_t *bytes;
  float f;
} FloatConversionData;

typedef union
{
  uint8_t *bytes;
  int32_t i;
} Int32ConversionData;

float bytes_to_float(uint8_t *bytes)
{
  FloatConversionData data;

  data.bytes = bytes;
  return data.f;
}

void float_to_bytes(float val, uint8_t* bytes)
{
  FloatConversionData data;

  data.f = val;
  bytes[0] = data.bytes[0];
  bytes[1] = data.bytes[1];
  bytes[2] = data.bytes[2];
  bytes[3] = data.bytes[3];
}

void int32_to_bytes(int32_t val, uint8_t* bytes)
{
  Int32ConversionData data;

  data.i = val;
  data.bytes[0] = bytes[0];
  data.bytes[1] = bytes[1];
  data.bytes[2] = bytes[2];
  data.bytes[3] = bytes[3];
}
#ifndef DUALMOTORDRIVER_H_
#define DUALMOTORDRIVER_H_

#include "stdint.h"

constexpr float PID_LOOP_TIME_MS = 1;

using PWM = uint8_t;

// Global limit for PWM values
constexpr PWM MAX_PWM_VALUE = UINT8_MAX;

// Pin definitions
#define ENCODER_LEFT_CHANNEL_A    2
#define ENCODER_RIGHT_CHANNEL_A   3
#define MOTOR_LEFT_DIR            4
#define MOTOR_LEFT_PWM            5
#define ENCODER_LEFT_CHANNEL_B    6
#define ENCODER_RIGHT_CHANNEL_B   7
#define MOTOR_RIGHT_DIR           8
#define MOTOR_RIGHT_PWM           9
#define DRIVER_OUTPUT_ENABLE      10
#define DRIVER_FAULT              11
#define FLAG                      12

#endif // DUALMOTORDRIVER_H
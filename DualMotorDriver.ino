#include "DualMotorDriver.h"
#include "Encoder.h"
#include "PidController.h"
#include "MotorController.h"

constexpr float PID_LOOP_TIME_S = 0.025;

// declared as global for access in ISRs
Encoder left_channel_encoder;
Encoder right_channel_encoder;

void left_encoder_isr()
{
  left_channel_encoder.update_count(digitalRead(ENCODER_LEFT_CHANNEL_A), digitalRead(ENCODER_LEFT_CHANNEL_B));
}

void right_encoder_isr()
{
  right_channel_encoder.update_count(digitalRead(ENCODER_RIGHT_CHANNEL_A), digitalRead(ENCODER_RIGHT_CHANNEL_B));
}

void configure_hardware()
{
  // encoder input GPIO setup
  pinMode(ENCODER_LEFT_CHANNEL_A, INPUT);
  pinMode(ENCODER_LEFT_CHANNEL_B, INPUT);
  pinMode(ENCODER_RIGHT_CHANNEL_A, INPUT);
  pinMode(ENCODER_RIGHT_CHANNEL_B, INPUT);

  // set up interrupts for left/right encoder
  attachInterrupt(ENCODER_LEFT_CHANNEL_A, left_encoder_isr, CHANGE);
  attachInterrupt(ENCODER_RIGHT_CHANNEL_A, right_encoder_isr, CHANGE);

  // Motor drive outputs
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(DRIVER_OUTPUT_ENABLE, OUTPUT);
  pinMode(DRIVER_FAULT, INPUT);
}

void setup() {
  PidController pid_controller_left(10, 0, 0, &left_channel_encoder);
  PidController pid_controller_right(10, 0, 0, &right_channel_encoder);

  pid_controller_left.set_target_speed(0.);
  pid_controller_right.set_target_speed(0.);

  MotorController controller(&left_channel_encoder, &right_channel_encoder, &pid_controller_left, &pid_controller_right);

  configure_hardware();
}

void loop() {

}

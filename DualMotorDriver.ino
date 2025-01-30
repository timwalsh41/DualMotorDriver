#include "Encoder.h"
#include "PidController.h"

constexpr float PID_LOOP_TIME_S = 0.025;

void setup() {
  // put your setup code here, to run once:

  Encoder left_channel_encoder;
  Encoder right_channel_encoder;

  PidController pid_controller_left(10, 0, 0, &left_channel_encoder, PID_LOOP_TIME_S);
  PidController pid_controller_right(10, 0, 0, &right_channel_encoder, PID_LOOP_TIME_S);

  pid_controller_left.set_target_speed(0.);
  pid_controller_right.set_target_speed(0.);

  // set up interrupts for left/right encoder

}

void loop() {

}

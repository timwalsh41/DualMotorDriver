#include "Arduino.h"
#include "DualMotorDriver.h"
#include "Encoder.h"
#include "PidController.h"
#include "MotorController.h"
#include "SerialInterface.h"
#include "Utility.h"

Encoder left_channel_encoder;
Encoder right_channel_encoder;

PidController pid_controller_left(0.1, 0., 0., 1., &left_channel_encoder);
PidController pid_controller_right(0.1, 0., 0., 1., &right_channel_encoder);

MotorController controller(&left_channel_encoder, &right_channel_encoder, &pid_controller_left, &pid_controller_right);

SerialInterface serial_interface(&controller);

uint32_t last_loop_time_ms{0};

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
  pinMode(ENCODER_LEFT_CHANNEL_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_CHANNEL_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_CHANNEL_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_CHANNEL_B, INPUT_PULLUP);

  // set up interrupts for left/right encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CHANNEL_A), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CHANNEL_A), right_encoder_isr, CHANGE);

  // Motor drive outputs
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(DRIVER_OUTPUT_ENABLE, OUTPUT);
  pinMode(DRIVER_FAULT, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FLAG, OUTPUT);

  digitalWrite(MOTOR_LEFT_DIR, 0);
  digitalWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, 0);
  digitalWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(FLAG, 0);
}

void setup() {
  pid_controller_left.set_target_speed(0.);
  pid_controller_right.set_target_speed(0.);
  
  configure_hardware();

  Serial.begin(115200);
}

void loop() {
  static uint32_t last_blink_time_ms = 0;
  // static uint8_t speed_bytes[4];

  if (time_elapsed(last_blink_time_ms, millis()) >= 250)
  {
    last_blink_time_ms = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Serial.println(controller.get_status().left_encoder_counts);
    // Serial.println(controller.get_status().fault);
    // Serial.println(controller.get_status().left_motor_current_speed_cps);

    /*
    float_to_bytes(controller.get_status().left_motor_current_speed_cps, speed_bytes);
    Serial.print("Speed bytes: ");
    for (size_t i = 0; i < 4; ++i) {
      Serial.print("0x");
      Serial.print(speed_bytes[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    */
  }

  if (time_elapsed(last_loop_time_ms, millis()) >= PID_LOOP_TIME_MS)
  {
    digitalWrite(FLAG, 1);
    last_loop_time_ms = millis();
    controller.update_controller();
    digitalWrite(FLAG, 0);
  }
  
  serial_interface.process_commands();
}

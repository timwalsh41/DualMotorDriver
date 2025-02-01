#include "Arduino.h"
#include "MotorDriver.h"

MotorDriver::MotorDriver()
{
  disable_driver();
  set_channel_pwm(MotorChannel::LEFT, 0);
  set_channel_pwm(MotorChannel::RIGHT, 0);
  set_channel_direction(MotorChannel::LEFT, true);
  set_channel_direction(MotorChannel::RIGHT, true);
}

bool m_is_enabled{false};
uint8_t m_left_pwm{0}, m_right_pwm{0};
bool m_left_direction{true}, m_right_direction{true};
bool m_fault{false};

void MotorDriver::update()
{
  digitalWrite(DRIVER_OUTPUT_ENABLE, m_is_enabled);
  digitalWrite(MOTOR_LEFT_DIR, m_left_direction);
  digitalWrite(MOTOR_RIGHT_DIR, m_right_direction);

  analogWrite(MOTOR_LEFT_PWM, m_left_pwm);
  analogWrite(MOTOR_RIGHT_PWM, m_right_pwm);

  // fault signal is active low
  m_fault = !digitalRead(DRIVER_FAULT);
}

void MotorDriver::set_channel_pwm(MotorChannel channel, uint8_t pwm)
{
  if (!m_is_enabled)
    return;

  if (channel == MotorChannel::LEFT)
  {
    m_left_pwm = pwm;
  }
  else if (channel == MotorChannel::RIGHT)
  {
    m_right_pwm = pwm;
  }
}

void MotorDriver::set_channel_direction(MotorChannel channel, bool direction)
{
  if (channel == MotorChannel::LEFT)
  {
    m_left_direction = direction;
  }
  else if (channel == MotorChannel::RIGHT)
  {
    m_right_direction = direction;
  }
}

void MotorDriver::enable_driver()
{
  m_is_enabled = true;
}

void MotorDriver::disable_driver()
{
  m_is_enabled = false;
  m_left_pwm = 0;
  m_right_pwm = 0;
}
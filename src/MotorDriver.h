#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include "DualMotorDriver.h"
#include "stdint.h"

enum MotorChannel
{
  LEFT,
  RIGHT
};

class MotorDriver
{
  public:
    MotorDriver();
    ~MotorDriver() = default;

    // regular loop update, writes state to driver
    void update();
    void set_channel_pwm(MotorChannel channel, PWM pwm);
    void set_channel_direction(MotorChannel channel, bool direction);
    void enable_driver();
    void disable_driver();
    bool is_faulted() const {return m_fault;}

  private:
    bool m_is_enabled{false};
    PWM m_left_pwm{0}, m_right_pwm{0};
    bool m_left_direction{true}, m_right_direction{true};
    bool m_fault{false};
};

#endif
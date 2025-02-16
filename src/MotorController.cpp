#include "Arduino.h"
#include "MotorController.h"

MotorController::MotorController(Encoder* p_left_encoder, Encoder* p_right_encoder, PidController* p_left_controller, PidController* p_right_controller) :
      m_p_left_encoder{p_left_encoder},
      m_p_right_encoder{p_right_encoder},
      m_p_left_pid_controller{p_left_controller},
      m_p_right_pid_controller{p_right_controller}
    {
      m_p_left_pid_controller->set_mode(m_mode);
      m_p_right_pid_controller->set_mode(m_mode);
    }

void MotorController::update_controller()
{
  float candidate_control_value;
  uint8_t target_pwm;

  // left channel control and status updates
  candidate_control_value = m_p_left_pid_controller->run_controller();

  if (candidate_control_value < 0)
  {
    m_driver.set_channel_direction(MotorChannel::LEFT, false);
    target_pwm = static_cast<uint8_t>(min(-1*candidate_control_value, PidController::MAX_CONTROL_VALUE));
  }
  else
  {
    m_driver.set_channel_direction(MotorChannel::LEFT, true);
    target_pwm = static_cast<uint8_t>(min(candidate_control_value, PidController::MAX_CONTROL_VALUE));
  }

  m_driver.set_channel_pwm(MotorChannel::LEFT, target_pwm);
  m_system_status.left_motor_current_speed_cps = m_p_left_pid_controller->get_current_speed_cps();
  m_system_status.left_encoder_counts = m_p_left_encoder->get_count();
  m_system_status.left_motor_pwm = target_pwm;

  // right channel control and status updates
  candidate_control_value = m_p_right_pid_controller->run_controller();

  if (candidate_control_value < 0)
  {
    m_driver.set_channel_direction(MotorChannel::RIGHT, false);
    target_pwm = static_cast<uint8_t>(min(-1*candidate_control_value, PidController::MAX_CONTROL_VALUE));
  }
  else
  {
    m_driver.set_channel_direction(MotorChannel::RIGHT, true);
    target_pwm = static_cast<uint8_t>(min(candidate_control_value, PidController::MAX_CONTROL_VALUE));
  }

  m_driver.set_channel_pwm(MotorChannel::RIGHT, target_pwm);
  m_system_status.right_motor_current_speed_cps = m_p_right_pid_controller->get_current_speed_cps();
  m_system_status.right_encoder_counts = m_p_right_encoder->get_count();
  m_system_status.right_motor_pwm = target_pwm;

  // update motor driver state
  if (m_system_status.enabled)
    m_driver.enable_driver();
  else
    m_driver.disable_driver();

  m_driver.update();
  m_system_status.fault = m_driver.is_faulted();
  m_system_status.mode = m_mode;
}

void MotorController::enable_drive()
{
  m_system_status.enabled = true;
}

void MotorController::disable_drive()
{
  m_system_status.enabled = false;
}

void MotorController::set_mode(ControlMode mode)
{
  m_mode = mode;
  m_p_left_pid_controller->set_mode(mode);
  m_p_right_pid_controller->set_mode(mode);
}

void MotorController::set_target_speed(MotorChannel channel, float target_speed_cps)
{
  if (channel == MotorChannel::LEFT)
  {
    m_p_left_pid_controller->set_target_speed(target_speed_cps);
    m_system_status.left_motor_target_speed_cps = target_speed_cps;
  }
  else if (channel == MotorChannel::RIGHT)
  {
    m_p_right_pid_controller->set_target_speed(target_speed_cps);
    m_system_status.right_motor_target_speed_cps = target_speed_cps;
  }
}

void MotorController::clear_encoders()
{
  m_p_left_encoder->reset_count();
  m_p_right_encoder->reset_count();
}

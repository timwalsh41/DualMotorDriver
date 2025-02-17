#include "Arduino.h"
#include "MotorController.h"
#include "DualMotorDriver.h"

MotorController::MotorController(Encoder* p_left_encoder, Encoder* p_right_encoder, PidController* p_left_controller, PidController* p_right_controller) :
      m_p_left_encoder{p_left_encoder},
      m_p_right_encoder{p_right_encoder},
      m_p_left_pid_controller{p_left_controller},
      m_p_right_pid_controller{p_right_controller}
    {
      m_p_left_pid_controller->set_mode(m_mode);
      m_p_right_pid_controller->set_mode(m_mode);

      m_system_status.kP = m_p_left_pid_controller->get_kP();
      m_system_status.kI = m_p_left_pid_controller->get_kI();
      m_system_status.kD = m_p_left_pid_controller->get_kD();
    }

void MotorController::update_controller()
{
  PWM target_pwm;

  target_pwm = update_pid_channel(m_p_left_pid_controller, MotorChannel::LEFT);
  m_system_status.left_motor_current_speed_cps = m_p_left_pid_controller->get_current_speed_cps();
  m_system_status.left_encoder_counts = m_p_left_encoder->get_count();
  m_system_status.left_motor_pwm = target_pwm;

  
  target_pwm = update_pid_channel(m_p_right_pid_controller, MotorChannel::RIGHT);
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

void MotorController::set_pid_gains(float kP, float kI, float kD)
{
  m_p_left_pid_controller->set_kP(kP);
  m_p_left_pid_controller->set_kI(kI);
  m_p_left_pid_controller->set_kD(kD);

  m_p_right_pid_controller->set_kP(kP);
  m_p_right_pid_controller->set_kI(kI);
  m_p_right_pid_controller->set_kD(kD);

  m_system_status.kP = kP;
  m_system_status.kI = kI;
  m_system_status.kD = kD;
}

PWM MotorController::update_pid_channel(PidController* p_pid_controller, MotorChannel channel)
{
  // Run channel control and status updates, getting nominal control value
  float candidate_control_value = p_pid_controller->run_controller();
  PWM target_pwm;

  if (candidate_control_value < 0)
  {
    m_driver.set_channel_direction(channel, false);
    target_pwm = static_cast<PWM>(min(-1*candidate_control_value, MAX_PWM_VALUE));
  }
  else
  {
    m_driver.set_channel_direction(channel, true);
    target_pwm = static_cast<PWM>(min(candidate_control_value, MAX_PWM_VALUE));
  }

  // don't apply a control signal if it conflicts with the direction of motion
  if (candidate_control_value*p_pid_controller->get_target_speed_cps() < 0)
  {
    target_pwm = 0;
  }

  m_driver.set_channel_pwm(channel, target_pwm);
  return target_pwm;
}
#include "Arduino.h"
#include "PidController.h"
#include "Utility.h"
#include "DualMotorDriver.h"
// #include <iostream>

PidController::PidController(float K_p, float K_i, float K_d, Encoder *p_encoder) : 
      m_K_p{K_p}, m_K_i{K_i}, m_K_d{K_d}, 
      m_p_encoder{p_encoder}
{
  // TODO: load via argument
  m_K_ff = 1;

  for (size_t i = 0; i < SPEED_SAMPLE_BUFFER_LENGTH; i++)
  {
    m_speed_sample_buffer[i] = 0;
  }

  for (size_t i = 0; i < PWM_SAMPLE_BUFFER_LENGTH; i++)
  {
    m_pwm_sample_buffer[i] = 0;
  }

  for (size_t i = 0; i < FILTERED_SPEED_BUFFER_LENGTH; i++)
  {
    m_filtered_speed_buffer[i] = 0;
  }
}

// Run controller updates the current speed and provides the output control signal
// (open or closed loop mode)
float PidController::run_controller()
{
  // read the instantaneous speed from the encoder and update the filtered speed
  // value
  update_current_speed();

  // apply target velocity ramp, if needed
  if (m_current_target_speed_cps < m_final_target_speed_cps)
  {
    m_current_target_speed_cps += SPEED_RAMP_RATE_CPS_PER_MS*PID_LOOP_TIME_MS;
    if (m_current_target_speed_cps > m_final_target_speed_cps)
      m_current_target_speed_cps = m_final_target_speed_cps;
  }
  else if (m_current_target_speed_cps > m_final_target_speed_cps)
  {
    m_current_target_speed_cps -= SPEED_RAMP_RATE_CPS_PER_MS*PID_LOOP_TIME_MS;
    if (m_current_target_speed_cps < m_final_target_speed_cps)
      m_current_target_speed_cps = m_final_target_speed_cps;
  }

  if (m_mode == CLOSED_LOOP)
  {
    float control_signal;

    // update error values
    float error = m_current_target_speed_cps - m_current_speed_cps;
    float filtered_error = m_current_target_speed_cps - m_filtered_speed_cps;
    update_accumulated_error(filtered_error);
    m_differential_error = error - m_last_error;

    control_signal = m_K_ff*CONTROL_SPEED_RATIO*m_current_target_speed_cps + m_K_p*error + m_K_i*m_accumulated_error + m_K_d*m_differential_error;
    m_last_error = error;

    // std::cout << "target = " << m_current_target_speed_cps << " current = " << m_current_speed_cps << " error = " << error << " raw control = " << control_signal;;

    // Update the filtered control signal
    update_filtered_pwm(control_signal);

    // std::cout << " filtered control " << m_filtered_pwm << std::endl;

    return m_filtered_pwm;
  }
  else
  {
    return CONTROL_SPEED_RATIO*m_current_target_speed_cps;
  }
}

// We maintain a buffer of instantaneous speed readings and take the average of that buffer
// as the instantaneous speed. Rather than running the average each time, we simply subtract
// the contribution from the sample being dropped from the buffer, and add the contribution
// from the new sample, then placing the new sample into the buffer at the location being
// vacated by the dropped sample.
void PidController::update_current_speed()
{
  float instantaneous_speed = get_instantaneous_speed();

  m_current_speed_cps += (instantaneous_speed - m_speed_sample_buffer[m_speed_sample_buffer_ptr])/SPEED_SAMPLE_BUFFER_LENGTH;
  m_speed_sample_buffer[m_speed_sample_buffer_ptr++] = instantaneous_speed;

  if (m_speed_sample_buffer_ptr >= SPEED_SAMPLE_BUFFER_LENGTH)
    m_speed_sample_buffer_ptr = 0;

  m_filtered_speed_cps += (m_current_speed_cps - m_filtered_speed_buffer[m_filtered_speed_buffer_ptr])/FILTERED_SPEED_BUFFER_LENGTH;
  m_filtered_speed_buffer[m_filtered_speed_buffer_ptr++] = m_current_speed_cps;

  if (m_filtered_speed_buffer_ptr >= FILTERED_SPEED_BUFFER_LENGTH)
    m_filtered_speed_buffer_ptr = 0;
}

// This function calculates the instantaneous speed. Note that since calling Encoder::get_delta()
// will update the last count, we should only call this once per update loop.
float PidController::get_instantaneous_speed()
{
  uint32_t current_time_us = micros();
  uint32_t time_delta_us;

  time_delta_us = time_elapsed(m_last_update_time_us, current_time_us);
  m_last_update_time_us = current_time_us;

  return static_cast<float>(m_p_encoder->update_delta())/static_cast<float>(time_delta_us)*1e6;
}

// When setting target speed, we'll clear the accumulated error
void PidController::set_target_speed(float target_speed_cps)
{
  m_final_target_speed_cps = target_speed_cps;
  m_accumulated_error = 0;
}

void PidController::update_accumulated_error(float error)
{
  m_accumulated_error += error;

  if (m_accumulated_error > MAX_ACCUMULATED_ERROR)
    m_accumulated_error = MAX_ACCUMULATED_ERROR;
  else if (m_accumulated_error < -MAX_ACCUMULATED_ERROR)
    m_accumulated_error = -MAX_ACCUMULATED_ERROR;
}

void PidController::set_mode(ControlMode mode)
{
  m_mode = mode;
  set_target_speed(0);
}

bool PidController::set_kP(float kP)
{
  if (kP >= 0)
  {
    m_K_p = kP;
    return true;
  }

  return false;
}

bool PidController::set_kI(float kI)
{
  if (kI >= 0)
  {
    m_K_i = kI;
    return true;
  }

  return false;
}

bool PidController::set_kD(float kD)
{
  if (kD >= 0)
  {
    m_K_d = kD;
    return true;
  }

  return false;
}

bool PidController::set_kFF(float kFF)
{
  if (kFF >= 0 && kFF <= 1)
  {
    m_K_ff = kFF;
    return true;
  }

  return false;
}

void PidController::update_filtered_pwm(float pwm_value)
{
  m_filtered_pwm += (pwm_value - m_pwm_sample_buffer[m_pwm_sample_buffer_ptr])/PWM_SAMPLE_BUFFER_LENGTH;
  m_pwm_sample_buffer[m_pwm_sample_buffer_ptr++] = pwm_value;

  if (m_pwm_sample_buffer_ptr >= PWM_SAMPLE_BUFFER_LENGTH)
    m_pwm_sample_buffer_ptr = 0;
}

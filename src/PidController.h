#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Encoder.h"
#include "stddef.h"

#define SPEED_SAMPLE_BUFFER_LENGTH 16

enum ControlMode
{
  OPEN_LOOP,
  CLOSED_LOOP
};

class PidController
{
  public:
    PidController(float K_p, float K_i, float K_d, Encoder *p_encoder);
    ~PidController() {}

    float run_controller();
    float get_current_speed_cps() const {return m_current_speed_cps;}
    void set_target_speed(float target_speed_cps);
    void set_mode(ControlMode mode);

    float get_kP() const {return m_K_p;}
    float get_kI() const {return m_K_i;}
    float get_kD() const {return m_K_d;}
    float get_target_speed_cps() const {return m_target_speed_cps;}

    void set_kP(float kP);
    void set_kI(float kI);
    void set_kD(float kD);

    // motor parameters
    static constexpr uint32_t ENCODER_CPR = 64;         // counts per revolution
    static constexpr uint32_t MOTOR_GEAR_RATIO = 50;
    static constexpr float MAX_OUTPUT_SPEED_RPM = 200.;  // unloaded speed
    static constexpr uint32_t MAX_CONTROL_VALUE = UINT8_MAX;  // equal to the maximum PWM duty cycle

    // motor max speed, encoder counts per second
    static constexpr float MAX_SPEED_CPS = MAX_OUTPUT_SPEED_RPM*static_cast<float>(MOTOR_GEAR_RATIO)*static_cast<float>(ENCODER_CPR)/3600.;

    // the ratio of control signal to speed for open-loop unloaded operation
    static constexpr float CONTROL_SPEED_RATIO = static_cast<float>(MAX_CONTROL_VALUE)/MAX_SPEED_CPS;

    static constexpr float MAX_ACCUMULATED_ERROR = 100.;

  private:
    uint32_t m_last_update_time_us{0};
    
    float m_K_p;
    float m_K_i;
    float m_K_d;

    float m_accumulated_error{0.};
    float m_differential_error{0.};
    float m_last_error{0.};

    ControlMode m_mode{CLOSED_LOOP};

    float m_target_speed_cps{0.};

    // velocities are in units of encoder counts per second
    float m_current_speed_cps{0.};
    float m_speed_sample_buffer[SPEED_SAMPLE_BUFFER_LENGTH];

    size_t m_speed_sample_buffer_ptr;

    Encoder *m_p_encoder;

    // calculate the instantaneous speed sample
    float get_instantaneous_speed();

    // update the filtered speed estimate
    void update_current_speed();

    // update accumulated error, clamping within a max range
    void update_accumulated_error(float error);
};

#endif
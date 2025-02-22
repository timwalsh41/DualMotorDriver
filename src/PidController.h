#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Encoder.h"
#include "stddef.h"

#define SPEED_SAMPLE_BUFFER_LENGTH 5      // buffer length for "real-time" speed used for P-gain calculation
#define PWM_SAMPLE_BUFFER_LENGTH 8        // buffer length for output control signal calculation
#define FILTERED_SPEED_BUFFER_LENGTH 64   // buffer length for integral gain calculation

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
    float get_kFF() const {return m_K_ff;}
    float get_target_speed_cps() const {return m_current_target_speed_cps;}

    bool set_kP(float kP);
    bool set_kI(float kI);
    bool set_kD(float kD);
    bool set_kFF(float kFF);

    // motor parameters
    static constexpr uint32_t ENCODER_CPR = 64/2;         // nominal encoder counts per revolution / 2
    static constexpr uint32_t MOTOR_GEAR_RATIO = 50;
    static constexpr float MAX_OUTPUT_SPEED_RPM = 200.;  // unloaded speed

    // motor max speed, encoder counts per second
    static constexpr float MAX_SPEED_CPS = MAX_OUTPUT_SPEED_RPM*static_cast<float>(MOTOR_GEAR_RATIO)*static_cast<float>(ENCODER_CPR)/60.;

    // the ratio of control signal to speed for open-loop unloaded operation
    static constexpr float CONTROL_SPEED_RATIO = static_cast<float>(UINT8_MAX)/MAX_SPEED_CPS;

    static constexpr float MAX_ACCUMULATED_ERROR = 100000.;

    static constexpr float SPEED_RAMP_RATE_CPS_PER_MS = 50;

  private:
    uint32_t m_last_update_time_us{0};
    
    float m_K_p;
    float m_K_i;
    float m_K_d;
    float m_K_ff;

    float m_accumulated_error{0.};
    float m_differential_error{0.};
    float m_last_error{0.};

    ControlMode m_mode{CLOSED_LOOP};

    // final target speed is the setpoint given by the user
    float m_final_target_speed_cps{0.};

    // current target speed is used to ramp from prior setpoint to new setpoint
    float m_current_target_speed_cps{0.};

    // velocities are in units of encoder counts per second
    // the "current speed" is filtered with a shorter buffer length/time constant
    // this value is meant to give a quicker response to changes in speed while
    // still providing some smoothing for low-resolution encoders
    float m_current_speed_cps{0.};
    float m_speed_sample_buffer[SPEED_SAMPLE_BUFFER_LENGTH];
    size_t m_speed_sample_buffer_ptr{0};

    // we also maintain a filtered speed estimate with larger buffer size/longer
    // time constant to use for integral gain calculation
    float m_filtered_speed_cps{0.};
    float m_filtered_speed_buffer[FILTERED_SPEED_BUFFER_LENGTH];  
    size_t m_filtered_speed_buffer_ptr{0};

    Encoder *m_p_encoder;

    float m_pwm_sample_buffer[PWM_SAMPLE_BUFFER_LENGTH];
    size_t m_pwm_sample_buffer_ptr{0};
    float m_filtered_pwm{0.};

    // calculate the instantaneous speed sample
    float get_instantaneous_speed();

    // update the filtered speed estimate
    void update_current_speed();

    // update accumulated error, clamping within a max range
    void update_accumulated_error(float error);

    // update the filtered PWM value
    void update_filtered_pwm(float pwm_value);
};

#endif
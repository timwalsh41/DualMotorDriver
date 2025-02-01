#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Encoder.h"
#include "PidController.h"
#include "MotorDriver.h"

typedef struct
{
  float left_motor_target_speed_cps{0.};
  float right_motor_target_speed_cps{0.};
  float left_motor_current_speed_cps{0.};
  float right_motor_current_speed_cps{0.};
  int32_t left_encoder_counts_cps{0};
  int32_t right_encoder_counts_cps{0};
  uint8_t left_motor_pwm{0};
  uint8_t right_motor_pwm{0};

  Mode mode{CLOSED_LOOP};  
  bool enabled{false};
  bool fault{false};
}
SystemStatus;

class MotorController
{
  public:
    MotorController(Encoder* p_left_encoder, Encoder* p_right_encoder, PidController* p_left_controller, PidController* p_right_controller);
    ~MotorController() {}

    // main update function, executes from main loop
    void UpdateController();

    // public API for serial commands
    const SystemStatus& get_status() const {return m_system_status;}
    void enable_drive();
    void disable_drive();
    void set_mode(Mode mode);
    void set_target_speed(MotorChannel channel, float speed_cps);
    void clear_encoders();

  private:
    Encoder *m_p_left_encoder, *m_p_right_encoder;
    PidController *m_p_left_pid_controller, *m_p_right_pid_controller;
    MotorDriver m_driver;

    Mode m_mode{CLOSED_LOOP};
    SystemStatus m_system_status;
};

#endif
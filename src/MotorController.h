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
  int32_t left_encoder_counts{0};
  int32_t right_encoder_counts{0};
  uint8_t left_motor_pwm{0};
  uint8_t right_motor_pwm{0};

  float kP{0.};
  float kI{0.};
  float kD{0.};

  bool enabled{false};
  bool fault{false};
  ControlMode mode{CLOSED_LOOP};
}
SystemStatus;

class IMotorController
{
  public:
    virtual ~IMotorController() {}
    virtual void enable_drive() = 0;
    virtual void disable_drive() = 0;
    virtual void set_mode(ControlMode mode) = 0;
    virtual void set_target_speed(MotorChannel channel, float speed_cps) = 0;
    virtual void clear_encoders() = 0;
    virtual const SystemStatus& get_status() const = 0;
    virtual void set_pid_gains(float kP, float kI, float kD) = 0;
};

class MotorController : public IMotorController
{
  public:
    MotorController(Encoder* p_left_encoder, Encoder* p_right_encoder, PidController* p_left_controller, PidController* p_right_controller);
    ~MotorController() {}

    // main update function, executes from main loop
    void update_controller();

    // public API for serial commands
    const SystemStatus& get_status() const override {return m_system_status;}
    void enable_drive() override;
    void disable_drive() override;
    void set_mode(ControlMode mode) override;
    void set_target_speed(MotorChannel channel, float speed_cps) override;
    void clear_encoders() override;
    void set_pid_gains(float kP, float kI, float kD) override;

  private:
    Encoder *m_p_left_encoder, *m_p_right_encoder;
    PidController *m_p_left_pid_controller, *m_p_right_pid_controller;
    MotorDriver m_driver;

    ControlMode m_mode{CLOSED_LOOP};
    SystemStatus m_system_status;
};

#endif
r'''
This module is used to do a closed loop simulation of the PID controller with a 
simple motor model. This gives us better visibility into the state of the PID
controller for testing without having to stream all the high-bandwidth data
back to the host.

Note that, for now, this only works for positive target speed
'''

import PidControllerWrapper
from MotorLoadModel import GearMotorLoadModel
import numpy as np
import matplotlib.pyplot as plt

class Encoder(PidControllerWrapper.IEncoder):
    def __init__(self, counts_per_revolution):
        super().__init__()

        self.cpr = counts_per_revolution
        self.count = 0
        self.last_count = 0
        self.last_int_delta = 0

    def update_count(self, channel_A, channel_B):
        pass

    def update_delta(self):
        delta = self.count - self.last_count

        # since, for lower speeds, many times we might just get a fraction of a count during an update
        # period, we only report a delta if we have accumulated more than 1 full count
        if (delta > 1):
            # we'll update the last count to maintain the fractional portion of the delta
            self.last_count = self.count - (delta - int(delta))

            self.last_int_delta = int(delta)
        
            # and return the truncated integer portion
            return int(delta)
        
        self.last_int_delta = 0
        return int(0)
        
    def reset_count(self):
        self.count = 0
        self.last_count = 0

    def get_count(self):
        return self.count

    def get_last_count(self):
        return self.last_count

    def set_count(self, count):
        self.count = count

def main():
    # Control parameters
    Kp = 0.0
    Ki = 0.0008
    Kd = 0.0
    Kff = 1.0

    # Gearmotor parameters
    gear_ratio = 50
    max_output_speed_rpm = 200
    encoder_cpr = 32
    supply_V = 12
    stall_current_A = 5
    
    # the time delta, this should match PID_LOOP_TIME in the
    # PidController module
    # Note this is not a real-time simulation
    dt_s = 1e-3

    N_SIMULATION_STEPS = 5000

    # Target speed at motor, starting from zero
    fixed_target_motor_speed_cps = 1800

    # PWM limit  - 8 bit
    PWM_MAX = 255

    # Output arrays
    target_motor_speed_cps = np.zeros(N_SIMULATION_STEPS)
    estimated_motor_speed_cps = np.zeros(N_SIMULATION_STEPS)
    actual_motor_speed_cps = np.zeros(N_SIMULATION_STEPS)
    output_speed_rpm = np.zeros(N_SIMULATION_STEPS)
    control_signal = np.zeros(N_SIMULATION_STEPS)
    error = np.zeros(N_SIMULATION_STEPS)
    accumulated_error = np.zeros(N_SIMULATION_STEPS)
    encoder_exact_delta = np.zeros(N_SIMULATION_STEPS)
    encoder_int_delta = np.zeros(N_SIMULATION_STEPS)
    time_s = np.arange(0, N_SIMULATION_STEPS*dt_s, dt_s)

    encoder = Encoder(encoder_cpr)
    gearmotor = GearMotorLoadModel(gear_ratio, max_output_speed_rpm, encoder_cpr, supply_V, stall_current_A)

    # The ratio between the PWM value and the motor speed (in counts per second)
    CONTROL_SPEED_RATIO = PWM_MAX/gearmotor.max_motor_speed_cps

    pid = PidControllerWrapper.PidController(Kp, Ki, Kd, Kff, encoder)
    pid.set_mode(PidControllerWrapper.ControlMode.CLOSED_LOOP)
    pid.set_target_speed(fixed_target_motor_speed_cps)

    def print_pid_info():
        print(f'Target speed: {pid.get_target_speed_cps()}')
        print(f'Current speed: {pid.get_current_speed_cps()}')
        print(f'Error: {pid.get_target_speed_cps() - pid.get_current_speed_cps()}')
        print(f'Accumulated error: {pid.get_accumulated_error()}')

    idx = 0
    for _ in range(N_SIMULATION_STEPS):
        control_signal[idx] = pid.run_controller()
        target_motor_speed_cps[idx] = pid.get_target_speed_cps()
        estimated_motor_speed_cps[idx] = pid.get_current_speed_cps()
        actual_motor_speed_cps[idx] = gearmotor.current_motor_speed_cps
        error[idx] = target_motor_speed_cps[idx] - estimated_motor_speed_cps[idx]
        accumulated_error[idx] = pid.get_accumulated_error()

        control_signal = np.clip(control_signal, 0, PWM_MAX)
        gearmotor.update(control_signal[idx]*supply_V/PWM_MAX, dt_s)
        output_speed_rpm[idx] = gearmotor.current_output_speed_rpm

        delta = gearmotor.current_motor_speed_cps*dt_s
        encoder.set_count(encoder.get_count() + delta)
        encoder_exact_delta[idx] = delta
        encoder_int_delta[idx] = encoder.last_int_delta

        idx += 1

    fig, ax = plt.subplots(5,1, figsize=(10,10), sharex=True)
    ax[0].plot(time_s, output_speed_rpm, label='output speed')
    ax[0].plot(time_s, gearmotor.motor_speed_cps_to_output_speed_rpm(target_motor_speed_cps), label='target speed')
    ax[0].set_ylabel('Output speed (RPM)')
    ax[0].legend()
    ax[0].grid()

    ax[1].plot(time_s, target_motor_speed_cps, label='target speed')
    ax[1].plot(time_s, actual_motor_speed_cps, label='actual speed')
    ax[1].plot(time_s, estimated_motor_speed_cps, label='estimated speed')
    ax[1].set_ylabel('Motor speed (cps)')
    ax[1].legend()
    ax[1].grid()

    ax[2].plot(time_s, control_signal, label='control signal')
    ax[2].stackplot(time_s, Kp*error, Ki*accumulated_error, Kff*target_motor_speed_cps*CONTROL_SPEED_RATIO, labels=['Proportional', 'Integral', 'Feedforward'])
    ax[2].set_ylabel('Control (PWM)')
    ax[2].legend()
    ax[2].grid()

    ax[3].plot(time_s, error, label='error')
    ax[3].plot(time_s, accumulated_error, label='accumulated error')
    ax[3].set_ylabel('Error (cs)')
    ax[3].legend()
    ax[3].grid()

    ax[4].plot(time_s, encoder_exact_delta, label='Exact delta')
    ax[4].plot(time_s, encoder_int_delta, label='Int delta')
    ax[4].set_ylabel('Encoder count')
    ax[4].set_xlabel('Time (s)')
    ax[4].legend()
    ax[4].grid()

    plt.show()

if __name__ == "__main__":
    main()

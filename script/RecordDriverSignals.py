# Send a velocity step input and capture and plot the measured velocity and PWM signals
# over some time window

# TO DO:
# - Also take an open loop response to look at system dynamics

from DualMotorCommander import DualMotorCommander
import time
import numpy as np
import matplotlib.pyplot as plt

Kp = 0.
Ki = 0.0002
Kd = 0.
Kff = 1.0

initial_target_velocity_cps = 0
final_target_velocity_cps = 2500

step_time_s = 0.5    # time to record before sending the step input
run_time_s = 10      # time to record after sending the step input

port = '/dev/ttyUSB0'
baud_rate = 115200

array_size = 200

commander = DualMotorCommander(port, baud_rate)
commander.set_pid_gains(Kp, Ki, Kd, Kff)
commander.set_target_speeds(0, 0)
commander.enable_driver()

# preallocate arrays for speed
time_vals = np.zeros(array_size)
target_speed = np.zeros(array_size)
measured_speed = np.zeros(array_size)
measured_pwm = np.zeros(array_size)
encoder_counts = np.zeros(array_size)

def record_values(target, idx):
    time_vals[idx] = time.time() - start_time
    target_speed[idx] = target
    
    result, speeds = commander.get_motor_speeds()
    if result:
        measured_speed[idx] = speeds[0]
    else:
        measured_speed[idx] = np.nan
    
    result, pwms = commander.get_pwm_values()
    if result:
        measured_pwm[idx] = pwms[0]
    else:
        measured_pwm[idx] = np.nan

    result, counts = commander.get_encoder_counts()
    if result:
        encoder_counts[idx] = counts[0]
    else:
        encoder_counts[idx] = np.nan

    idx += 1

idx = 0

commander.set_target_speeds(initial_target_velocity_cps, initial_target_velocity_cps)
start_time = time.time()
while time.time() - start_time < step_time_s:
    record_values(initial_target_velocity_cps, idx)
    idx += 1

commander.set_target_speeds(final_target_velocity_cps, 0)
while time.time() - start_time < (step_time_s + run_time_s):
    record_values(final_target_velocity_cps, idx)
    idx += 1

commander.set_target_speeds(0, 0)
time.sleep(0.2)
commander.disable_driver()

dt = np.mean(time_vals[1:10] - time_vals[:9])
print(f'Total array size is {idx}, dt = {dt:.4f} s')

if idx < array_size:
    time_vals = time_vals[:idx]
    measured_speed = measured_speed[:idx]
    measured_pwm = measured_pwm[:idx]
    encoder_counts = encoder_counts[:idx]
    target_speed = target_speed[:idx]

fig,ax = plt.subplots(3,1,sharex=True)
ax[0].plot(time_vals, encoder_counts)
ax[0].set_ylabel('Encoder Counts')
ax[0].grid()

ax[1].plot(time_vals, target_speed)
ax[1].plot(time_vals, measured_speed)
ax[1].set_ylabel('Speed (cps)')
ax[1].legend(['Target','Measured'])
ax[1].grid()

ax[2].plot(time_vals, measured_pwm)
ax[2].set_ylabel('PWM')
ax[2].grid()
ax[2].set_xlabel('Time (s)')

pass

plt.show()

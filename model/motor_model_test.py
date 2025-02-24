from MotorLoadModel import GearMotorLoadModel
import numpy as np
import matplotlib.pyplot as plt

# gearmotor parameters
max_output_speed_rpm = 200
gear_ratio = 50
encoder_cpr = 32
supply_V = 12
stall_current_A = 5

PWM_MAX = 255
DRIVE_DIVIDER=1

# simulation parameters
dt_s = 0.001
N_samples = 4000

# our gearmotor model will handle all of the transformations for speed at the motor/output
gearmotor = GearMotorLoadModel(gear_ratio, max_output_speed_rpm, encoder_cpr, supply_V, stall_current_A)

target_pwm = int(PWM_MAX/DRIVE_DIVIDER)

print(f'Max speed at motor is {gearmotor.max_motor_speed_cps} cps = {gearmotor.max_motor_speed_rad_per_s} rad/s')
print(f'Target output speed is {gearmotor.max_output_speed_rpm/DRIVE_DIVIDER} rpm = {gearmotor.max_output_speed_rad_per_s/DRIVE_DIVIDER} rad/s')
print(f'Target motor speed is {gearmotor.max_motor_speed_cps/DRIVE_DIVIDER} cps = {gearmotor.max_motor_speed_rad_per_s/DRIVE_DIVIDER} rad/s')

motor_speed_rad_per_s = np.zeros(N_samples)
output_speed_rpm = np.zeros(N_samples)
acceleration = np.zeros(N_samples)
current = np.zeros(N_samples)
time_s = np.arange(0, N_samples*dt_s, dt_s)

for i in range(N_samples):
    motor_speed_rad_per_s[i] = gearmotor.current_motor_speed_rad_per_s
    output_speed_rpm[i] = gearmotor.current_output_speed_rpm
    acceleration[i] = gearmotor.current_motor_acceleration_rad_per_s2
    current[i] = gearmotor.current_A
    gearmotor.update(12*target_pwm/255, dt_s)

fig,ax = plt.subplots(4,1, figsize=(10,10), sharex=True)
ax[0].plot(time_s, motor_speed_rad_per_s, label='Motor speed')
ax[0].set_title('Motor Speed (rad/s)')
ax[0].grid()

ax[1].plot(time_s, output_speed_rpm, label='Output speed')
ax[1].set_title('Output Speed (rpm)')
ax[1].grid()

ax[2].plot(time_s, acceleration, label='Motor acceleration')
ax[2].set_title('Motor Acceleration (rad/s^2)')
ax[2].grid()

ax[3].plot(time_s, current, label='Motor current')
ax[3].set_title('Motor Current (A)')
ax[3].grid()
ax[3].set_xlabel('Time (s)')

plt.show()


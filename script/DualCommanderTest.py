from DualMotorCommander import DualMotorCommander

commander = DualMotorCommander('/dev/ttyUSB1', 115200)

result = commander.get_motor_speeds()
print(f'Motor speeds are left={result[1][0]:.3f} cps, right={result[1][1]:.3f} cps')

result = commander.get_encoder_counts()
print(f'Encoder counts are left={result[1][0]}, right={result[1][1]}')

result = commander.read_fault()
print(f'Fault status is {result[1][0]}')

result = commander.set_target_speeds(1.5762, 2.3198)
print(f'Set target speeds: {result}')

result = commander.set_closed_loop_mode()
print(f'Set closed loop mode: {result}')
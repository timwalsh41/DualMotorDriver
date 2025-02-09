from DualMotorCommander import DualMotorCommander
import time

commander = DualMotorCommander('/dev/ttyUSB0', 115200)

r'''
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
'''

while True:
    encoder_result = commander.get_encoder_counts()
    speed_result = commander.get_motor_speeds()
    
    _, fault = commander.read_fault()
    print(f'{time.time():.2f} Status={speed_result[0]}; encoder left={encoder_result[1][0]} right={encoder_result[1][1]} speeds left={speed_result[1][0]} cps right={speed_result[1][1]} cps; fault = {fault}')

    time.sleep(0.25)
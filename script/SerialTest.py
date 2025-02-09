import serial
import time
import pdb

print('Opening the serial port')
sp = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.05)
sp.flush()
print(sp.is_open)

print('Reading from the port')

# short sleep to wait for startup message
time.sleep(2)

if sp.in_waiting > 0:
    b = sp.read(sp.in_waiting)
    print(b)

time.sleep(0.05)

# manually build the packet to request the motor speeds
get_speed_packet = bytearray([0x16, 0x80, 0x1b])
get_encoder_packet = bytearray([37, 128, 57])

while True:
    sp.write(get_speed_packet)
    time.sleep(0.05)

    if sp.in_waiting > 0:
        b = sp.read(sp.in_waiting)
        print(b)

    sp.write(get_encoder_packet)
    time.sleep(0.05)

    if sp.in_waiting > 0:
        b = sp.read(sp.in_waiting)
        print(b)

    time.sleep(0.5)

    
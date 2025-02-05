import serial
import time
import pdb

print('Opening the serial port')
sp = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.05)
sp.dtr = False
sp.flush()
print(sp.is_open)

print('Reading from the port')

# short sleep to wait for startup message
time.sleep(2)

if sp.in_waiting > 0:
    b = sp.read(sp.in_waiting)
    print(b)

time.sleep(0.05)

while True:
    sp.write(b'a')
    time.sleep(0.25)

    if sp.in_waiting > 0:
        b = sp.read(sp.in_waiting)
        pdb.set_trace()

    
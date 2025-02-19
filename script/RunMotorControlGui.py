from MotorControlGui import MotorControlGUI
from DualMotorCommander import DualMotorCommander

PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

controller = DualMotorCommander(PORT, BAUD_RATE)
gui = MotorControlGUI(controller)
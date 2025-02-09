from MotorControlGui import MotorControlGUI
from DualMotorCommander import DualMotorCommander

PLOT_UPDATE_TIME_S = 0.05
LABEL_UPDATE_TIME_S = 1.
PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

controller = DualMotorCommander(PORT, BAUD_RATE)
gui = MotorControlGUI(controller)
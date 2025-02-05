from crc import Calculator, Configuration
from SerialParser import SerialParser
import CommandInterface
import time

port = '/dev/ttyUSB0'

# parameters for the CRC16 used by default in the Arduino library
config = Configuration(
    width=16,
    polynomial=0x8001,
    init_value=0x0000,
    final_xor_value=0x0000,
    reverse_input=False,
    reverse_output=False,
)

calculator = Calculator(config)
parser = SerialParser(calculator, port, 115200)

time.sleep(2)

command = CommandInterface.CommandDefinitionMap[CommandInterface.CMD_CLEAR_ENCODER_COUNTS]
parser.send_command(command)

time.sleep(0.05)
response = parser.poll()

print(response)

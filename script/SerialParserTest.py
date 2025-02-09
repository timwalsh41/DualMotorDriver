from crc import Calculator, Configuration
from SerialParser import SerialParser
import CommandInterface
import time
from struct import pack

port = '/dev/ttyUSB1'

# parameters for the CRC16 used by default in the Arduino library
config = Configuration(
    width=16,
    polynomial=0x8001,
    init_value=0x0000,
    final_xor_value=0x0000,
    reverse_input=False,
    reverse_output=False,
)

SERIAL_DELAY_TIME_S = 0.01

calculator = Calculator(config)
parser = SerialParser(calculator, port, 115200)

time.sleep(2)

command = CommandInterface.CommandDefinitionMap[CommandInterface.CMD_CLEAR_ENCODER_COUNTS]
parser.send_command(command)

time.sleep(SERIAL_DELAY_TIME_S)
response = parser.poll()

print(response)

time.sleep(SERIAL_DELAY_TIME_S)

command = CommandInterface.CommandDefinitionMap[CommandInterface.CMD_GET_ENCODER_COUNTS]
parser.send_command(command)

time.sleep(SERIAL_DELAY_TIME_S)
response = parser.poll()

print(response)

time.sleep(SERIAL_DELAY_TIME_S)

command = CommandInterface.CommandDefinitionMap[CommandInterface.CMD_SET_TARGET_SPEEDS]
payload = pack('<ff', 1.5762, 2.3198)
parser.send_command(command, payload)

time.sleep(SERIAL_DELAY_TIME_S)
response = parser.poll()

print(response)
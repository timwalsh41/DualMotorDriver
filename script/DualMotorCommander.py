from SerialParser import SerialParser
from crc import Calculator, Configuration
from struct import pack, unpack
import CommandInterface
import time

SERIAL_DELAY_TIME_S = 0.01

# format string for byte packing, big-endian, 2 4-byte floats
DUAL_FLOAT_PACK_FMT = '>ff'

# format string for byte packing, big-endian, 2 4-byte signed integers
DUAL_INT32_PACK_FMT = '>ll'

class DualMotorCommander:
    r'''
    This class abstracts the command interface to the Arduino dual motor driver. It presents
    a set of functions that implement the command-response protocol between this host and
    the Arduino motor driver target. Each function implements a single command. The functions
    take in a payload, if necessary for the function. They use the SerialParser class to build
    the command packet (with payload), send the packet to the Arduino, wait for the response,
    and then parse the response and return a two-element tuple where the first element is a
    bool representing the command status (True = success), and the second is a list of the
    response payload (which may be empty).
    '''

    def __init__(self, port, baudrate=115200):
        # we'll use the default CRC16 parameters from the Arduino library
        crc_config = Configuration(
            width=16,
            polynomial=0x8001,
            init_value=0x0000,
            final_xor_value=0x0000,
            reverse_input=False,
            reverse_output=False,
        )
        self._crc = Calculator(crc_config)

        self._parser = SerialParser(self._crc, port, baudrate)

        # Arduinos reset upon establishing serial connection, so we give a couple
        # of seconds here to allow it to come up
        time.sleep(2)

    def _execute_command(self, command: CommandInterface.CommandDefinition, payload_data: bytearray = bytearray()):
        self._parser.send_command(command, payload_data)
        time.sleep(SERIAL_DELAY_TIME_S)
        response = self._parser.poll()

        if response is None:
            return (False, [])
        elif response['status']:
            return (True, response['payload'])

        return (False, [])

    def set_target_speeds(self, left_speed_cps: float, right_speed_cps: float):
        payload = pack(DUAL_FLOAT_PACK_FMT, left_speed_cps, right_speed_cps)
        return self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_SET_TARGET_SPEEDS], payload)[0]

    def get_motor_speeds(self):
        result = self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_GET_MOTOR_SPEEDS])

        if result[0]:
            motor_speeds = unpack(DUAL_FLOAT_PACK_FMT, result[1])
            return (True, motor_speeds)

        return (False, [])

    def set_open_loop_mode(self):
        return self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_SET_OPEN_LOOP_MODE])[0]

    def set_closed_loop_mode(self):
        return self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_SET_CLOSED_LOOP_MODE])[0]

    def get_encoder_counts(self):
        result = self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_GET_ENCODER_COUNTS])

        if result[0]:
            encoder_counts = unpack(DUAL_INT32_PACK_FMT, result[1])
            return (True, encoder_counts)

        return (False, [])

    def clear_encoder_counts(self):
        return self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_CLEAR_ENCODER_COUNTS])[0]

    def enable_driver(self):
        return self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_ENABLE_DRIVER])[0]

    def disable_driver(self):
        return self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_DISABLE_DRIVER])[0]

    def read_fault(self):
        result = self._execute_command(CommandInterface.CommandDefinitionMap[CommandInterface.CMD_READ_FAULT])

        if result[0]:
            # the payload in the second element has the fault status in a single element byte array
            return (True, [result[1][0] > 0])

        return (False, [])

from dataclasses import dataclass

@dataclass
class CommandDefinition:
    name: str
    description: str
    command_byte: int
    command_payload_length: int
    response_payload_length: int
    callback: callable = None

# We reserve the two most significant bits for the ACK and STATUS bits
ACK_BIT = 0x80
STATUS_BIT = 0x40

# Valid command codes are therefore 0x00 - 0x3F
CMD_SET_TARGET_SPEEDS = 0x15
CMD_GET_MOTOR_SPEEDS = 0x16
CMD_SET_OPEN_LOOP_MODE = 0x05
CMD_SET_CLOSED_LOOP_MODE = 0x06
CMD_GET_ENCODER_COUNTS = 0x25
CMD_CLEAR_ENCODER_COUNTS = 0x26
CMD_ENABLE_DRIVER = 0x35
CMD_DISABLE_DRIVER = 0x36
CMD_READ_FAULT = 0x37

CommandDefinitionMap = {
    CMD_SET_TARGET_SPEEDS:      CommandDefinition('CMD_SET_TARGET_SPEEDS',
                                                  'Set target speeds for each motor',
                                                  CMD_SET_TARGET_SPEEDS, 8, 0, None),
    CMD_GET_MOTOR_SPEEDS:       CommandDefinition('CMD_GET_MOTOR_SPEEDS',
                                                  'Get current speeds of each motor',
                                                  CMD_GET_MOTOR_SPEEDS, 0, 8, None),
    CMD_SET_OPEN_LOOP_MODE:     CommandDefinition('CMD_SET_OPEN_LOOP_MODE',
                                                  'Set controller to open loop mode',
                                                  CMD_SET_OPEN_LOOP_MODE, 0, 0, None),
    CMD_SET_CLOSED_LOOP_MODE:   CommandDefinition('CMD_SET_CLOSED_LOOP_MODE',
                                                  'Set controller to closed loop mode',
                                                  CMD_SET_CLOSED_LOOP_MODE, 0, 0, None),
    CMD_GET_ENCODER_COUNTS:     CommandDefinition('CMD_GET_ENCODER_COUNTS',
                                                  'Get current encoder counts of each motor',
                                                  CMD_GET_ENCODER_COUNTS, 0, 8, None),
    CMD_CLEAR_ENCODER_COUNTS:   CommandDefinition('CMD_CLEAR_ENCODER_COUNTS',
                                                  'Clear encoder counts of each motor',
                                                  CMD_CLEAR_ENCODER_COUNTS, 0, 0, None),
    CMD_ENABLE_DRIVER:          CommandDefinition('CMD_ENABLE_DRIVER',
                                                  'Enable motor driver',
                                                  CMD_ENABLE_DRIVER, 0, 0, None),
    CMD_DISABLE_DRIVER:         CommandDefinition('CMD_DISABLE_DRIVER',
                                                  'Disable motor driver',
                                                  CMD_DISABLE_DRIVER, 0, 0, None),
    CMD_READ_FAULT:             CommandDefinition('CMD_READ_FAULT',
                                                  'Read fault status',
                                                  CMD_READ_FAULT, 0, 1, None)
}
from dataclasses import dataclass

@dataclass
class CommandDefinition:
    name: str
    description: str
    command: int
    command_payload_length: int
    response_payload_length: int
    callback: callable = None

CommandDefinitionMap = {
    'CMD_SET_TARGET_SPEEDS': CommandDefinition('CMD_SET_TARGET_SPEEDS', 'Set target speeds for each motor', 0x15, 8, 0, None),
    'CMD_GET_MOTOR_SPEEDS': CommandDefinition('CMD_GET_MOTOR_SPEEDS', 'Get current speeds of each motor', 0x16, 0, 8, None),
    'CMD_SET_OPEN_LOOP_MODE': CommandDefinition('CMD_SET_OPEN_LOOP_MODE', 'Set controller to open loop mode', 0x05, 0, 0, None),
    'CMD_SET_CLOSED_LOOP_MODE': CommandDefinition('CMD_SET_CLOSED_LOOP_MODE', 'Set controller to closed loop mode', 0x06, 0, 0, None),
    'CMD_GET_ENCODER_COUNTS': CommandDefinition('CMD_GET_ENCODER_COUNTS', 'Get current encoder counts of each motor', 0x25, 0, 8, None),
    'CMD_CLEAR_ENCODER_COUNTS': CommandDefinition('CMD_CLEAR_ENCODER_COUNTS', 'Clear encoder counts of each motor', 0x26, 0, 0, None),
    'CMD_ENABLE_DRIVER': CommandDefinition('CMD_ENABLE_DRIVER', 'Enable motor driver', 0x35, 0, 0, None),
    'CMD_DISABLE_DRIVER': CommandDefinition('CMD_DISABLE_DRIVER', 'Disable motor driver', 0x36, 0, 0, None),
    'CMD_READ_FAULT': CommandDefinition('CMD_READ_FAULT', 'Read fault status', 0x37, 0, 1, None)
}

ACK_BIT = 0x80
STATUS_BIT = 0x40
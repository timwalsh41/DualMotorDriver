import serial
from crc import Calculator
import CommandInterface
import os
import pdb

class SerialParser:
    def __init__(self, crc: Calculator, port: str, baudrate: int = 115200):
        if not isinstance(crc, Calculator):
            raise ValueError("CRC must be an instance of crc.Calculator")
        if not isinstance(port, str):
            raise ValueError("Port must be a string")
        if not port.startswith('/dev/') or not os.path.exists(port):
            raise ValueError("Port must be a valid path in '/dev/'")
        if not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("Baudrate must be a positive integer")
        
        self._crc = crc
        self._port = port
        self._baudrate = baudrate

        self._buffer = bytearray()
        self._serial = serial.Serial(self._port, baudrate=self._baudrate, timeout=1)

    def send_command(self, command_definition: CommandInterface, *payload):
        packet = self._build_packet(command_definition, payload)
        print("Outgoing packet: " + packet.hex())
        self._serial.write(packet)

    def poll(self):
        if self._serial.in_waiting > 0:
            self._buffer.extend(self._serial.read(self._serial.in_waiting))
            print(self._buffer)
            return self._parse_response()
        
        print('No response')
        return None

    def _build_packet(self, command_definition: CommandInterface, payload):
        packet = bytearray()
        packet.append(command_definition.command_byte)
        packet.extend(payload)
        crc = self._crc.checksum(packet)
        packet.extend(crc.to_bytes(2, 'big'))
        return packet

    def _parse_response(self):
        if len(self._buffer) < 4:
            return None

        command_byte = self._buffer[0]
        payload_length = self._buffer[1]
        if len(self._buffer) < 4 + payload_length:
            return None

        payload = self._buffer[2:2 + payload_length]
        received_crc = int.from_bytes(self._buffer[2 + payload_length:4 + payload_length], 'big')
        calculated_crc = self._crc.checksum(self._buffer[:2 + payload_length])

        # pdb.set_trace()

        self._buffer = bytearray()

        if received_crc == calculated_crc:
            status = (command_byte & 0x80) != 0
            return {'status': status, 'payload': payload}    
        
        return None
        
#include "Arduino.h"
#include "SerialInterface.h"
#include "MotorController.h"
#include "Utility.h"
#include "CRC16.h"
#include "CRC.h"

SerialInterface::SerialInterface(MotorController* p_controller) :
  m_p_controller{p_controller}
{
  build_command_map();
}

SerialInterface::~SerialInterface()
{
  if (m_command_map != nullptr)
  {
    delete[] m_command_map;
    m_command_map = nullptr;
  }
}

void SerialInterface::process_commands()
{
  parse_serial_input();

  if (m_command_packet_ready)
  {
    
    CommandDefinition* incoming_command;

    // we can set the ready flag to false at this point, once we exit this function we will
    // not revisit the current command
    m_command_packet_ready = false;

    if (!get_command_definition(incoming_command))
    {
      // no command match found, discard the packet
      Serial.write("Command not found\n");
      return;
    }

    // apply the ack bit; until we also apply the success bit this will represent
    // a failed command
    uint8_t command_byte = incoming_command->command | ACK_BIT;

    // check incoming payload length (equal to the buffer byte count minus
    // one for the command byte and two for the CRC)
    size_t payload_length = m_processing_buffer_char_count - 1 - 2;
    if (payload_length != incoming_command->payload_length)
    {
      // payload length mismatch, send back the ack'd command byte, without success flag
      Serial.write("Invalid payload length\n");
      Serial.write(command_byte);
      return;
    }

    // check CRC
    uint16_t crc = calcCRC16(m_processing_buffer, m_processing_buffer_char_count - 2);
    uint16_t payload_crc = m_processing_buffer[m_processing_buffer_char_count-2] << 8 + 
                  m_processing_buffer[m_processing_buffer_char_count-1];
    
    if (crc != payload_crc)
    {
      // CRC mismatch, send back the ack'd command byte, without success flag
      Serial.write("CRC mismatch\n");
      Serial.write(command_byte);
      return;
    }

    uint8_t outbound_payload[16];

    // run command
    bool result = incoming_command->callback(command_byte, &m_processing_buffer[1], incoming_command->payload_length, outbound_payload, m_p_controller);

    // send response
    if (result)
    {
      Serial.write("Command succeeded, sending response\n");
      command_byte |= SUCCESS_BIT;
      uint8_t outbound_packet[20];

      outbound_packet[0] = command_byte;
      outbound_packet[1] = incoming_command->response_length;

      if (incoming_command->response_length > 0)
        memcpy(&outbound_packet[1], outbound_payload, incoming_command->response_length);

      uint16_t outbound_crc = calcCRC16(outbound_packet, incoming_command->response_length + 2);
      memcpy(&outbound_packet[incoming_command->response_length+2], outbound_crc, 2);
      Serial.write(static_cast<unsigned char*>(&outbound_packet[0]), incoming_command->response_length + 4);
    }
    else
      // command failed, send back the ack'd command byte, without success flag
      Serial.write("Command failed\n");
      Serial.write(command_byte);
  }
}

void SerialInterface::parse_serial_input()
{
  while (Serial.available())
  {
    if (m_incoming_buffer_char_count == 0)
    {
      uint8_t candidate_byte = Serial.read();
      if (is_valid_command(candidate_byte))
      {
        m_incoming_buffer[m_incoming_buffer_char_count++] = candidate_byte;

        // I don't think this will work - the initial string will have a terminator, and that's all that will print
        Serial.write("Received a valid command byte\n");
      }
    }
    else
    {
      m_incoming_buffer[m_incoming_buffer_char_count++] = Serial.read();
      Serial.write("Received a byte for an open buffer\n");

      if (m_incoming_buffer_char_count == SERIAL_BUFFER_SIZE)
      {
        Serial.write("Incoming buffer full, handing off\n");
        hand_off_command_packet();
      }
    }
  }

  if (time_elapsed(m_last_character_time, millis()) > SERIAL_TIMEOUT_MS && m_incoming_buffer_char_count > 0)
  {
    Serial.write("Timeout with non-empty buffer, handing off packet\n");
    hand_off_command_packet();
  }
}

bool SerialInterface::is_valid_command(uint8_t candidate_byte)
{
  for (size_t i = 0; i < NUM_COMMANDS; i++)
    if (m_command_map[i].command == candidate_byte)
      return true;

  return false;
}

void SerialInterface::hand_off_command_packet()
{
  m_command_packet_ready = true;

  if (m_incoming_buffer == m_serial_buffer_a)
  {
    m_incoming_buffer = m_serial_buffer_b;
    m_processing_buffer = m_serial_buffer_a;
  }
  else
  {
    m_incoming_buffer = m_serial_buffer_a;
    m_processing_buffer = m_serial_buffer_b;
  }

  m_processing_buffer_char_count = m_incoming_buffer_char_count;
  m_incoming_buffer_char_count = 0;
}

bool SerialInterface::get_command_definition(CommandDefinition* command)
{
  for (size_t i = 0; i < NUM_COMMANDS; i++)
    if (m_processing_buffer[0] == m_command_map[i].command)
    {
      command = &m_command_map[i];
      return true;
    }

  return false;
}

void SerialInterface::build_command_map()
{
  size_t map_idx = 0;
  m_command_map = new CommandDefinition[NUM_COMMANDS];

  // SET MOTOR SPEEDS
  // Incoming payload length: 8
  // Payload[0:3]: left motor speed, counts per second, float
  // Payload[4:7]: right motor speed, counts per second, float
  // No outgoing payload
  m_command_map[map_idx].command = CMD_SET_MOTOR_SPEEDS;
  m_command_map[map_idx].payload_length = 8;
  m_command_map[map_idx].response_length = 0;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      p_controller->set_target_speed(MotorChannel::LEFT, bytes_to_float(&incoming_payload[0]));
      p_controller->set_target_speed(MotorChannel::RIGHT, bytes_to_float(&incoming_payload[4]));

      return true;
    };
  map_idx++;

  // GET MOTOR SPEEDS
  // No incoming payload length
  // Outgoing payload length: 8
  // Payload[0:3]: left motor speed, counts per second, float
  // Payload[4:7]: right motor speed, counts per second, float
  m_command_map[map_idx].command = CMD_GET_MOTOR_SPEEDS;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 8;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      outgoing_payload = new uint8_t[8];

      float_to_bytes(p_controller->get_status().left_motor_current_speed_cps, &outgoing_payload[0]);
      float_to_bytes(p_controller->get_status().right_motor_current_speed_cps, &outgoing_payload[4]);

      return true;
    };
  map_idx++;

  // SET OPEN LOOP MODE
  // No incoming payload
  // No  outgoing payload
  m_command_map[map_idx].command = CMD_SET_OPEN_LOOP_MODE;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 0;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      p_controller->set_mode(ControlMode::OPEN_LOOP);

      return true;
    };
  map_idx++;
  
  // SET CLOSED LOOP MODE
  // No incoming payload
  // No outgoing payload
  m_command_map[map_idx].command = CMD_SET_CLOSED_LOOP_MODE;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 0;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      p_controller->set_mode(ControlMode::CLOSED_LOOP);

      return true;
    };
  map_idx++;

  // GET ENCODER COUNTS
  // No incoming payload
  // Outgoing payload length: 8
  // Payload[0:3]: left motor encoder count, uint32_t
  // Payload[4:7]: right motor encoder count, uint32_t
  m_command_map[map_idx].command = CMD_GET_ENCODER_COUNTS;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 8;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      outgoing_payload = new uint8_t[8];

      int32_to_bytes(p_controller->get_status().left_encoder_counts, &outgoing_payload[0]);
      int32_to_bytes(p_controller->get_status().right_encoder_counts, &outgoing_payload[4]);

      return true;
    };
  map_idx++;

  // CLEAR ENCODER COUNTS
  // No incoming payload
  // No outgoing payload
  m_command_map[map_idx].command = CMD_CLEAR_ENCODER_COUNTS;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 0;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      p_controller->clear_encoders();

      return true;
    };
  map_idx++;

  // ENABLE DRIVER
  // No incoming payload
  // No outgoing payload
  m_command_map[map_idx].command = CMD_ENABLE_DRIVER;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 0;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      p_controller->enable_drive();

      return true;
    };
  map_idx++;

  // DISABLE DRIVER
  // No incoming payload
  // No outgoing payload
  m_command_map[map_idx].command = CMD_DISABLE_DRIVER;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 0;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      p_controller->disable_drive();

      return true;
    };
  map_idx++;

  // READ FAULT
  // No incoming payload
  // Outgoing payload length: 1
  // Payload[0] = fault status (1 if faulted, 0 otherwise)
  m_command_map[map_idx].command = CMD_READ_FAULT;
  m_command_map[map_idx].payload_length = 0;
  m_command_map[map_idx].response_length = 1;
  m_command_map[map_idx].callback = [](Command& command, uint8_t* incoming_payload, size_t incoming_payload_length, uint8_t* outgoing_payload, MotorController* p_controller)
    {
      outgoing_payload = new uint8_t[1];
      outgoing_payload[0] = p_controller->get_status().fault;

      return true;
    };
  map_idx++;
}
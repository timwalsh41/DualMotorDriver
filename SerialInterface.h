#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include "stdint.h"
#include "stddef.h"

// forward declaration of motor controller class
class MotorController;

using Command = uint8_t;

// Command callback arguments are:
// - Command byte, passed by reference to allow setting ack and status bits
// - Incoming payload (pointer)
// - Incoming payload length
// - Outgoing payload length, passed by reference
// - MotorController pointer (with Arduino Nano, we don't have standard library support and can't use std::function
//   to define this type; using a function pointer means we can't use lambdas with captures, so we'll just pass
//   the member variable motor controller pointer instead)
using CommandCallback = bool (*)(Command&, uint8_t*, size_t, uint8_t*, MotorController*);

typedef struct
{
  Command command;
  size_t payload_length;
  size_t response_length;
  CommandCallback callback;
} CommandDefinition;

#define NUM_COMMANDS 9
#define SERIAL_BUFFER_SIZE 16

class SerialInterface
{
  public:
    SerialInterface(MotorController*);
    ~SerialInterface();

    void process_commands();

    static constexpr uint8_t ACK_BIT = 0x01 << 7;
    static constexpr uint8_t SUCCESS_BIT = 0x01 << 6;

    static constexpr Command CMD_SET_OPEN_LOOP_MODE = 0x05;
    static constexpr Command CMD_SET_CLOSED_LOOP_MODE = 0x06;

    static constexpr Command CMD_SET_MOTOR_SPEEDS = 0x15;
    static constexpr Command CMD_GET_MOTOR_SPEEDS = 0x16;

    static constexpr Command CMD_GET_ENCODER_COUNTS = 0x25;
    static constexpr Command CMD_CLEAR_ENCODER_COUNTS = 0x26;

    static constexpr Command CMD_ENABLE_DRIVER = 0x35;
    static constexpr Command CMD_DISABLE_DRIVER = 0x36;
    static constexpr Command CMD_READ_FAULT = 0x37;

  private:
    void build_command_map();
    void parse_serial_input();
    bool is_valid_command(uint8_t candidate_byte);
    void hand_off_command_packet();
    bool get_command_definition(CommandDefinition** command);

    CommandDefinition* m_command_map{nullptr};
    MotorController* m_p_controller;

    bool m_command_packet_ready{false};
    int32_t m_last_character_time{0};

    uint8_t m_serial_buffer_a[SERIAL_BUFFER_SIZE];
    uint8_t m_serial_buffer_b[SERIAL_BUFFER_SIZE];

    uint8_t* m_incoming_buffer{m_serial_buffer_a};
    uint8_t* m_processing_buffer{m_serial_buffer_b};

    size_t m_incoming_buffer_char_count{0};
    size_t m_processing_buffer_char_count{0};

    static constexpr uint32_t SERIAL_TIMEOUT_MS = 4;
};

#endif
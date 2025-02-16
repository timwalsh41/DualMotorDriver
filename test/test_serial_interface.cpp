#include "MotorController.h"
#include "SerialInterface.h"
#include "Arduino.h"    // Mock Arduino.h
#include "unity.h"      // Include Unity test framework
#include "CRC16.h"      // Include Rob Tillaart CRC16 library
#include "CRC.h"        // Include Rob Tillaart CRC library
#include <cstring>
#include <iostream>
#include <thread>
class MockController : public IMotorController
{
    public:
        MockController() {}
        ~MockController() {}

        void enable_drive() {}
        void disable_drive() {}
        void set_mode(ControlMode mode) {}
        void set_target_speed(MotorChannel channel, float speed_cps)
        {
            if (channel == MotorChannel::LEFT)
            {
                m_system_status.left_motor_target_speed_cps = speed_cps;
            }
            else if (channel == MotorChannel::RIGHT)
            {
                m_system_status.right_motor_target_speed_cps = speed_cps;
            }
        }
        void clear_encoders() {}
        const SystemStatus& get_status() const {return m_system_status;}
    private:
        SystemStatus m_system_status;
};

MockController controller;

void serial_test_setup()
{

}

void serial_test_teardown()
{

}

void test_serial_type()
{
    SerialInterface interface(&controller);
}

// This test will send a valid command packet to the serial interface, and verify that the command
// is processed properly, including extracting the payload (2 floats), and then sending the command to
// the motor controller to set the motor speeds. Finally, it confirms that the correct response was 
// written back to the serial bus
void test_valid_command_sequence()
{
    SerialInterface interface(&controller);

    // Create a valid command packet for setting motor speeds
    uint8_t command_packet[11];
    command_packet[0] = SerialInterface::CMD_SET_MOTOR_SPEEDS;

    float left_speed = 123.45f;
    float right_speed = 678.90f;
    memcpy(&command_packet[1], &left_speed, sizeof(float));
    memcpy(&command_packet[5], &right_speed, sizeof(float));

    uint16_t crc = calcCRC16(command_packet, 9);
    command_packet[9] = static_cast<uint8_t>(crc >> 8);
    command_packet[10] = static_cast<uint8_t>(crc & 0xFF);

    // Simulate receiving the command packet via serial    
    Serial.load_read_buffer(command_packet, sizeof(command_packet));

    // Process the commands - this will process the incoming command packet, but we
    // need to wait for the command timeout to actually process the command
    interface.process_commands();

    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS+2));
    interface.process_commands();

    // Verify the motor speeds were set correctly
    TEST_ASSERT_FLOAT_WITHIN(0.01f, left_speed, controller.get_status().left_motor_target_speed_cps);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, right_speed, controller.get_status().right_motor_target_speed_cps);

    // Now make sure we got the right response written out on the serial bus
    uint8_t response[20];
    size_t response_length = Serial.retrieve_write_buffer(response, 20);

    // the response should be 4 bytes (command ack, response length, and 2 byte CRC)
    TEST_ASSERT_EQUAL(4, response_length);

    // The command ack should be the same as the incoming command, with the ack & success bits set
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_MOTOR_SPEEDS | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);

    // The response length should be 0
    TEST_ASSERT_EQUAL(0, response[1]);

    // The CRC should be correct
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
}

// Confirm that the command fails and we get no response on the serial bus
void test_invalid_command()
{

}

// Confirm that the command fails and we get the proper response on the serial bus, 
// that is an ack'd command byte but no payload, and no CRC
void test_invalid_payload()
{
    SerialInterface interface(&controller);

    uint8_t bad_command[3];
    bad_command[0] = SerialInterface::CMD_SET_MOTOR_SPEEDS; // Valid command byte
    bad_command[1] = 0x01; // Invalid payload (should be 8 bytes)
    bad_command[2] = 0x02;

    Serial.load_read_buffer(bad_command, sizeof(bad_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(1, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_MOTOR_SPEEDS | SerialInterface::ACK_BIT, response[0]);
}

// Likewise in the case of an invalid CRC, we should get the ack'd command byte without
// success bit set, and no crc
void test_invalid_crc()
{

}

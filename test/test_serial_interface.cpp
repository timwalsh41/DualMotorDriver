#include "MotorController.h"
#include "SerialInterface.h"
#include "Arduino.h"    // Mock Arduino.h
#include "unity.h"      // Include Unity test framework
#include "CRC16.h"      // Include Rob Tillaart CRC16 library
#include "CRC.h"        // Include Rob Tillaart CRC library
#include <cstring>
#include <iostream>
#include <thread>

// Mock MotorController class
// This mock implementation just updates the system status based on the command calls
class MockController : public IMotorController
{
    public:
        MockController() 
        {
            // put some dummy data in the encoder and speed
            m_system_status.left_encoder_counts = 1234;
            m_system_status.right_encoder_counts = 5678;

            m_system_status.left_motor_current_speed_cps = 1.234f;
            m_system_status.right_motor_current_speed_cps = 5.678f;
        }
        ~MockController() {}

        void enable_drive() { m_system_status.enabled = true; }
        void disable_drive() { m_system_status.enabled = false; }
        void set_mode(ControlMode mode) { m_system_status.mode = mode; }
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
        void clear_encoders() { m_system_status.left_encoder_counts = m_system_status.right_encoder_counts = 0; }
        const SystemStatus& get_status() const {return m_system_status;}

        // some extra functions required for test
        bool is_closed_loop_mode() const {return m_system_status.mode == ControlMode::CLOSED_LOOP;}
        void set_fault_status(bool fault) {m_system_status.fault = fault;}

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
void test_set_motor_speed_command()
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

    // We add this sleep and another process commands call to have the parser actually handle
    // the command and send the response back
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

    // The response payload length should be 0
    TEST_ASSERT_EQUAL(0, response[1]);

    // The CRC should be correct
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
}

void test_enable_disable_driver()
{
    SerialInterface interface(&controller);

    // Enable the driver
    uint8_t enable_command[3];
    enable_command[0] = SerialInterface::CMD_ENABLE_DRIVER;
    uint16_t crc = calcCRC16(enable_command, 1);
    enable_command[1] = static_cast<uint8_t>(crc >> 8);
    enable_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(enable_command, sizeof(enable_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_ENABLE_DRIVER | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_TRUE(controller.get_status().enabled);

    // Disable the driver
    uint8_t disable_command[3];
    disable_command[0] = SerialInterface::CMD_DISABLE_DRIVER;
    crc = calcCRC16(disable_command, 1);
    disable_command[1] = static_cast<uint8_t>(crc >> 8);
    disable_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(disable_command, sizeof(disable_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_DISABLE_DRIVER | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_FALSE(controller.get_status().enabled);

    // Enable the driver again
    Serial.load_read_buffer(enable_command, sizeof(enable_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();
    
    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_ENABLE_DRIVER | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_TRUE(controller.get_status().enabled);
}

void test_read_fault()
{
    SerialInterface interface(&controller);

    // Set the fault status
    controller.set_fault_status(true);

    // Read the fault status
    uint8_t read_fault_command[3];
    read_fault_command[0] = SerialInterface::CMD_READ_FAULT;
    uint16_t crc = calcCRC16(read_fault_command, 1);
    read_fault_command[1] = static_cast<uint8_t>(crc >> 8);
    read_fault_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(read_fault_command, sizeof(read_fault_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(5, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_READ_FAULT | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(1, response[1]);  // payload length
    TEST_ASSERT_EQUAL(1, response[2]);  // fault status
    TEST_ASSERT_EQUAL(calcCRC16(response, 3), (response[3] << 8) | response[4]);

    // Clear the fault status
    controller.set_fault_status(false);

    // Read the fault status again
    Serial.load_read_buffer(read_fault_command, sizeof(read_fault_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(5, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_READ_FAULT | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(1, response[1]);  // payload length
    TEST_ASSERT_EQUAL(0, response[2]);  // fault status
    TEST_ASSERT_EQUAL(calcCRC16(response, 3), (response[3] << 8) | response[4]);
}

void test_read_clear_encoders()
{
    SerialInterface interface(&controller);

    // Read the encoders to confirm they have the values initialized in the constructor
    // for the mock controller
    uint8_t read_encoders_command[3];
    read_encoders_command[0] = SerialInterface::CMD_GET_ENCODER_COUNTS;
    uint16_t crc = calcCRC16(read_encoders_command, 1);
    read_encoders_command[1] = static_cast<uint8_t>(crc >> 8);
    read_encoders_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(read_encoders_command, sizeof(read_encoders_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[20];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(12, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_GET_ENCODER_COUNTS | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(8, response[1]);  // payload length
    
    int32_t count;
    memcpy(&count, &response[2], sizeof(int32_t));
    TEST_ASSERT_EQUAL(controller.get_status().left_encoder_counts, count);
    memcpy(&count, &response[6], sizeof(int32_t));
    TEST_ASSERT_EQUAL(controller.get_status().right_encoder_counts, count);

    TEST_ASSERT_EQUAL(calcCRC16(response, 10), (response[10] << 8) | response[11]);

    // Clear the encoders
    uint8_t clear_encoders_command[3];
    clear_encoders_command[0] = SerialInterface::CMD_CLEAR_ENCODER_COUNTS;
    crc = calcCRC16(clear_encoders_command, 1);
    clear_encoders_command[1] = static_cast<uint8_t>(crc >> 8);
    clear_encoders_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(clear_encoders_command, sizeof(clear_encoders_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_CLEAR_ENCODER_COUNTS | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);

    // Read the encoders again to confirm they have been cleared
    TEST_ASSERT_EQUAL(0, controller.get_status().left_encoder_counts);
    TEST_ASSERT_EQUAL(0, controller.get_status().right_encoder_counts);
}

void test_get_motor_speeds()
{
    SerialInterface interface(&controller);

    // Read the motor speeds to confirm they have the values initialized in the constructor
    // for the mock controller
    uint8_t read_speeds_command[3];
    read_speeds_command[0] = SerialInterface::CMD_GET_MOTOR_SPEEDS;
    uint16_t crc = calcCRC16(read_speeds_command, 1);
    read_speeds_command[1] = static_cast<uint8_t>(crc >> 8);
    read_speeds_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(read_speeds_command, sizeof(read_speeds_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[20];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(12, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_GET_MOTOR_SPEEDS | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(8, response[1]);  // payload length

    float speed;
    memcpy(&speed, &response[2], sizeof(float));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, controller.get_status().left_motor_current_speed_cps, speed);
    memcpy(&speed, &response[6], sizeof(float));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, controller.get_status().right_motor_current_speed_cps, speed);

    TEST_ASSERT_EQUAL(calcCRC16(response, 10), (response[10] << 8) | response[11]);
}

void test_set_control_mode()
{
    SerialInterface interface(&controller);

    // Set the control mode to closed loop
    uint8_t set_mode_command[3];
    set_mode_command[0] = SerialInterface::CMD_SET_CLOSED_LOOP_MODE;
    uint16_t crc = calcCRC16(set_mode_command, 1);
    set_mode_command[1] = static_cast<uint8_t>(crc >> 8);
    set_mode_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(set_mode_command, sizeof(set_mode_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_CLOSED_LOOP_MODE | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_TRUE(controller.is_closed_loop_mode());

    // Set the control mode to open loop
    set_mode_command[0] = SerialInterface::CMD_SET_OPEN_LOOP_MODE;
    crc = calcCRC16(set_mode_command, 1);
    set_mode_command[1] = static_cast<uint8_t>(crc >> 8);
    set_mode_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(set_mode_command, sizeof(set_mode_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_OPEN_LOOP_MODE | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_FALSE(controller.is_closed_loop_mode());

    // Set the control mode back to closed loop
    set_mode_command[0] = SerialInterface::CMD_SET_CLOSED_LOOP_MODE;
    crc = calcCRC16(set_mode_command, 1);
    set_mode_command[1] = static_cast<uint8_t>(crc >> 8);
    set_mode_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(set_mode_command, sizeof(set_mode_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_CLOSED_LOOP_MODE | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_TRUE(controller.is_closed_loop_mode());
}


// Confirm that the command fails and we get no response on the serial bus
void test_invalid_command()
{
    SerialInterface interface(&controller);

    uint8_t bad_command[3];
    bad_command[0] = SerialInterface::ACK_BIT; // Invalid command byte
    
    uint16_t crc = calcCRC16(bad_command, 1);
    bad_command[1] = static_cast<uint8_t>(crc >> 8);
    bad_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(bad_command, sizeof(bad_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(0, resp_len);
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
    SerialInterface interface(&controller);

    uint8_t bad_crc[11];
    bad_crc[0] = SerialInterface::CMD_SET_MOTOR_SPEEDS; // Valid command byte
    
    float left_speed = 123.45f;
    float right_speed = 678.90f;
    memcpy(&bad_crc[1], &left_speed, sizeof(float));
    memcpy(&bad_crc[5], &right_speed, sizeof(float));
    
    // get an invalid CRC
    uint16_t crc = calcCRC16(bad_crc, 9);
    bad_crc[9] = static_cast<uint8_t>(++crc >> 8);
    bad_crc[10] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(bad_crc, sizeof(bad_crc));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(1, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_MOTOR_SPEEDS | SerialInterface::ACK_BIT, response[0]);
}

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

        void enable_drive() override { m_system_status.enabled = true; }
        void disable_drive() override { m_system_status.enabled = false; }
        void set_mode(ControlMode mode) override { m_system_status.mode = mode; }
        void set_target_speed(MotorChannel channel, float speed_cps) override
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
        void clear_encoders() override { m_system_status.left_encoder_counts = m_system_status.right_encoder_counts = 0; }
        const SystemStatus& get_status() const override {return m_system_status;}
        void set_pid_gains(float kP, float kI, float kD, float kFF) override {m_system_status.kP = kP; m_system_status.kI = kI; m_system_status.kD = kD; m_system_status.kFF = kFF;}

        // some extra functions required for test
        bool is_closed_loop_mode() const {return m_system_status.mode == ControlMode::CLOSED_LOOP;}
        void set_fault_status(bool fault) {m_system_status.fault = fault;}
        void set_pwm_values(uint8_t left_pwm, uint8_t right_pwm) {m_system_status.left_motor_pwm = left_pwm; m_system_status.right_motor_pwm = right_pwm;}

    private:
        SystemStatus m_system_status;

};

MockController controller;

void serial_test_setup()
{
    Serial.flush();
}

void serial_test_teardown()
{

}

void test_serial_type()
{
    SerialInterface interface(&controller);
    Serial.flush();
}

// This test will send a valid command packet to the serial interface, and verify that the command
// is processed properly, including extracting the payload (2 floats), and then sending the command to
// the motor controller to set the motor speeds. Finally, it confirms that the correct response was 
// written back to the serial bus
void test_set_motor_speed_command()
{
    SerialInterface interface(&controller);
    Serial.flush();

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
    Serial.flush();

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
    Serial.flush();

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
    Serial.flush();

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
    Serial.flush();

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
    Serial.flush();

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

void test_set_get_pid_gains()
{
    SerialInterface interface(&controller);
    Serial.flush();

    float kP = 1.23f;
    float kI = 4.56f;
    float kD = 7.89f;
    float kFF = 0.5f;

    // Set the PID gains
    uint8_t set_gains_command[19];
    set_gains_command[0] = SerialInterface::CMD_SET_PID_GAINS;
    memcpy(&set_gains_command[1], &kP, sizeof(float));
    memcpy(&set_gains_command[5], &kI, sizeof(float));
    memcpy(&set_gains_command[9], &kD, sizeof(float));
    memcpy(&set_gains_command[13], &kFF, sizeof(float));
    uint16_t crc = calcCRC16(set_gains_command, 17);
    set_gains_command[17] = static_cast<uint8_t>(crc >> 8);
    set_gains_command[18] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(set_gains_command, sizeof(set_gains_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[20];       // size 20 for the read response test below
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_SET_PID_GAINS | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(0, response[1]);
    TEST_ASSERT_EQUAL(calcCRC16(response, 2), (response[2] << 8) | response[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kP, controller.get_status().kP);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kI, controller.get_status().kI);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kD, controller.get_status().kD);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kFF, controller.get_status().kFF);

    // Get the PID gains
    uint8_t get_gains_command[3];
    get_gains_command[0] = SerialInterface::CMD_GET_PID_GAINS;
    crc = calcCRC16(get_gains_command, 1);
    get_gains_command[1] = static_cast<uint8_t>(crc >> 8);
    get_gains_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(get_gains_command, sizeof(get_gains_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    TEST_ASSERT_EQUAL(20, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_GET_PID_GAINS | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(16, response[1]);  // payload length

    float k;
    memcpy(&k, &response[2], sizeof(float));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kP, k);
    memcpy(&k, &response[6], sizeof(float));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kI, k);
    memcpy(&k, &response[10], sizeof(float));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kD, k);
    memcpy(&k, &response[14], sizeof(float));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kFF, k);

    TEST_ASSERT_EQUAL(calcCRC16(response, 18), (response[18] << 8) | response[19]);
}

void test_get_pwm()
{
    SerialInterface interface(&controller);
    Serial.flush();

    // set up the controller to have non-zero PWM values
    controller.set_pwm_values(123, 234);

    // Send the read PWM command
    uint8_t get_PWM_command[3];
    get_PWM_command[0] = SerialInterface::CMD_GET_PWM_VALUES;
    uint16_t crc = calcCRC16(get_PWM_command, 1);
    get_PWM_command[1] = static_cast<uint8_t>(crc >> 8);
    get_PWM_command[2] = static_cast<uint8_t>(crc & 0xFF);

    Serial.load_read_buffer(get_PWM_command, sizeof(get_PWM_command));
    interface.process_commands();
    std::this_thread::sleep_for(std::chrono::milliseconds(SerialInterface::SERIAL_TIMEOUT_MS + 2));
    interface.process_commands();

    // Check the response
    uint8_t response[10];
    size_t resp_len = Serial.retrieve_write_buffer(response, sizeof(response));
    
    // should have 6 bytes (command, payload length, 2 PWM values, and 2 byte CRC)
    TEST_ASSERT_EQUAL(6, resp_len);
    TEST_ASSERT_EQUAL(SerialInterface::CMD_GET_PWM_VALUES | SerialInterface::ACK_BIT | SerialInterface::SUCCESS_BIT, response[0]);
    TEST_ASSERT_EQUAL(2, response[1]);
    TEST_ASSERT_EQUAL(response[2], controller.get_status().left_motor_pwm);
    TEST_ASSERT_EQUAL(response[3], controller.get_status().right_motor_pwm);
    TEST_ASSERT_EQUAL(calcCRC16(response, 4), (response[4] << 8) | response[5]);
}

// Confirm that the command fails and we get no response on the serial bus
void test_invalid_command()
{
    SerialInterface interface(&controller);
    Serial.flush();

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
    Serial.flush();

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
    Serial.flush();

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

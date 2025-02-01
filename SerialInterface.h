#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#define CMD_SET_OPEN_LOOP_MODE      0x05
#define CMD_SET_CLOSED_LOOP_MODE    0x06

// SET MOTOR SPEEDS
// Payload length: 2
// Payload[0]: left motor speed, counts per second, float
// Payload[1]: right motor speed, counts per second, float
// Ack payload length: 1
// Payload[0]: command status
#define CMD_SET_MOTOR_SPEEDS        0x15


#define CMD_GET_MOTOR_SPEEDS        0x16

#define CMD_GET_ENCODER_COUNTS      0x25
#define CMD_CLEAR_ENCODER_COUNTS    0x26

#define ENABLE_DRIVER               0x35
#define DISABLE_DRIVER              0x36
#define READ_FAULT                  0x37

#endif
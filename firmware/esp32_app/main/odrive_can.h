#pragma once
#include <stdint.h>

// refer to https://docs.odriverobotics.com/can-protocol
#define ODRIVE_MSG_ID(AXIS_ID, CMD_ID) ((AXIS_ID << 5) | CMD_ID)

#define ODRIVE_CMD_CANOPEN_NMT 0x00
#define ODRIVE_CMD_HEARTBEAT 0x01
#define ODRIVE_CMD_ESTOP 0x02
#define ODRIVE_CMD_GET_MOTOR_ERROR 0x03
#define ODRIVE_CMD_GET_ENCODER_ERROR 0x04
#define ODRIVE_CMD_GET_SENSORLESS_ERROR 0x05
#define ODRIVE_CMD_SET_AXIS_NODE_ID 0x06
#define ODRIVE_CMD_SET_REQUESTED_STATE 0x07
#define ODRIVE_CMD_SET_STARTUP_CONFIG 0x08
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES 0x09
#define ODRIVE_CMD_GET_ENCODER_COUNT 0x0A
#define ODRIVE_CMD_SET_CONTROL_MODES 0x0B
#define ODRIVE_CMD_SET_INPUT_POS 0x0C
#define ODRIVE_CMD_SET_INPUT_VEL 0x0D
#define ODRIVE_CMD_SET_INPUT_TORQUE 0x0E
#define ODRIVE_CMD_SET_LIMITS 0x0F
#define ODRIVE_CMD_START_ANTICOGGING 0x10
#define ODRIVE_CMD_SET_TRAJ_VEL_LIMIT 0x11
#define ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITs 0x12
#define ODRIVE_CMD_SET_TRAJ_INERTIA 0x13
#define ODRIVE_CMD_GET_IQ 0x14
#define ODRIVE_CMD_GET_SENSORLESS_ESTIMATES 0x15
#define ODRIVE_CMD_REBOOT 0x16
#define ODRIVE_CMD_GET_VBUS_VOLTAGE 0x17
#define ODRIVE_CMD_CLEAR_ERRORS 0x18
#define ODRIVE_CMD_SET_LINEAR_COUNT 0x19
#define ODRIVE_CMD_CANOPEN_HEARTBEAT 0x700

typedef struct {
    uint32_t axis_error;
    uint32_t axis_current_state;
} ODriveHeartbeat;

typedef struct {
    float position;
    float velocity;
} ODriveEncoderEstimates;

typedef struct {
    int32_t shadow_count;
    int32_t count_cpr;
} ODriveEncoderCount;

typedef struct {
    int32_t control_mode;
    int32_t input_mode;
} ODriveControllerModes;

typedef struct {
    float position;
    int16_t velocity_x1000;
    int16_t torque_x1000;
} ODriveInputPosition;

typedef struct {
    float velocity;
    float torque;
} ODriveInputVelocity;

typedef struct {
    float velocity_limit;
    float current_limit;
} ODriveInputLimits;

typedef struct {
    float accel_limit;
    float decel_limit;
} ODriveTrajAccelLimit;

typedef struct {
    float setpoint;
    float measured;
} ODriveIq;

typedef ODriveEncoderEstimates ODriveSensorlessEstimates;

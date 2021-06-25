#include <stdint.h>
#include <esp_log.h>
#include "driver/twai.h"
#include "msg_defs.h"

// refer to https://docs.odriverobotics.com/can-protocol
#define ODRIVE_MSG_ID(AXIS_ID, CMD_ID) ((AXIS_ID << 5) | CMD_ID)
#define ODRIVE_AXIS_ID(MSG_ID) (MSG_ID >> 5)
#define ODRIVE_CMD_ID(MSG_ID) (MSG_ID & 0x1F)

// Command IDs
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
#define ODRIVE_CMD_SET_CONTROLLER_MODES 0x0B
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

typedef enum {
    ODRIVE_CONTROL_MODE_VOLTAGE,
    ODRIVE_CONTROL_MODE_TORQUE,
    ODRIVE_CONTROL_MODE_VELOCITY,
    ODRIVE_CONTROL_MODE_POSITION,
} ODriveControlMode;

// Protocol Messages
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

// private functions
static void log_msg(const twai_message_t* msg, const char* str) {
    ESP_LOGW("ODriveCan", "%s (id %x flags %x len %d)", str, msg->identifier, msg->flags,
            msg->data_length_code);
}

// public functions
esp_err_t odrive_set_velocity(uint8_t axis_id, const MotorInput* input) {
    assert(input);
    twai_message_t msg = {};
    msg.identifier = ODRIVE_MSG_ID(axis_id, ODRIVE_CMD_SET_INPUT_VEL);
    msg.data_length_code = sizeof(ODriveInputVelocity);
    *(ODriveInputVelocity*) msg.data = *(ODriveInputVelocity*) &input->velocity;
    return twai_transmit(&msg, 0);
}

esp_err_t odrive_set_position(uint8_t axis_id, const MotorInput* input) {
    assert(input);
    twai_message_t msg = {};
    msg.identifier = ODRIVE_MSG_ID(axis_id, ODRIVE_CMD_SET_INPUT_POS);
    msg.data_length_code = sizeof(ODriveInputPosition);
    ODriveInputPosition* input_pos = (ODriveInputPosition*) msg.data;
    input_pos->position = input->position;
    input_pos->velocity_x1000 = input->velocity * 1000;
    input_pos->torque_x1000 = input->torque * 1000;
    return twai_transmit(&msg, 0);
}

esp_err_t odrive_set_control_mode(uint8_t axis_id, const MotorMode* mode) {
    assert(mode);
    twai_message_t msg = {};
    msg.identifier = ODRIVE_MSG_ID(axis_id, ODRIVE_CMD_SET_CONTROLLER_MODES);
    msg.data_length_code = sizeof(MotorMode);
    *(MotorMode*) msg.data = *mode;
    return twai_transmit(&msg, 0);
}

esp_err_t odrive_send_inputs(MotorMsg* motors, uint8_t len) {
    assert(motors);
    while (len--) {
        switch (motors[len].mode.control) {
            case ODRIVE_CONTROL_MODE_VELOCITY:
                ESP_ERROR_CHECK_WITHOUT_ABORT(odrive_set_velocity(len, &motors[len].input));
                break;
            case ODRIVE_CONTROL_MODE_POSITION:
                ESP_ERROR_CHECK_WITHOUT_ABORT(odrive_set_position(len, &motors[len].input));
                break;
            default:
                assert(0 && "control mode not implemented");
                break;
        }
    }
    return ESP_OK;
}

esp_err_t odrive_receive_updates(MotorMsg* motors, uint8_t len) {
    assert(motors);
    assert(len < 32);
    static esp_err_t error;
    static twai_message_t msg;
    uint32_t updated = 0;
    while ((error = twai_receive(&msg, 0)) == ESP_OK) {
        if (msg.flags) {
            log_msg(&msg, "received message with invalid flag");
            continue;
        }
        uint8_t axis_id = ODRIVE_AXIS_ID(msg.identifier);
        if (axis_id >= len) {
            log_msg(&msg, "received message with invalid identifier");
            continue;
        }
        if (msg.data_length_code != 8) {
            log_msg(&msg, "received message with invalid length");
            continue;
        }
        switch (ODRIVE_CMD_ID(msg.identifier)) {
            case ODRIVE_CMD_HEARTBEAT:
                motors[axis_id].status = *(MotorStatus*) msg.data;
                break;
            case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
                motors[axis_id].estimate = *(MotorEstimate*) msg.data;
                updated |= 1 << axis_id;
                break;
            default:
                log_msg(&msg, "received message with invalid command");
                break;
        }
    }
    // check if estimates from all motors have been received
    return updated == (1 << len) - 1 ? ESP_OK : error;
}

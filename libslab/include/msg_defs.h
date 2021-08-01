#pragma once
#include "mxgen.h"

////////////////////////////////////////////////////////////////////////////////
// IMU

// acceleration in units of (g)
// angular velocity in units of (rad/s)

#define TYPEDEF_QuaternionF(X, _) \
    X(float, qw, )                \
    X(float, qx, )                \
    X(float, qy, )                \
    X(float, qz, )
MXGEN(struct, QuaternionF)

#define TYPEDEF_Vector3F(X, _) \
    X(float, x, )              \
    X(float, y, )              \
    X(float, z, )
MXGEN(struct, Vector3F)

#define TYPEDEF_ImuMsg(X, _)           \
    X(QuaternionF, orientation, )      \
    X(Vector3F, linear_acceleration, ) \
    X(Vector3F, angular_velocity, )
MXGEN(struct, ImuMsg)

////////////////////////////////////////////////////////////////////////////////
// Motor

// position in units of (rad)
// velocity in units of (rad/s)
// torque in units of (Nm)

typedef enum {
    MOTOR_CONTROL_MODE_POSITION = 1 << 0,
    MOTOR_CONTROL_MODE_VELOCITY = 1 << 1,
    MOTOR_CONTROL_MODE_TORQUE = 1 << 2,
} MotorControlMode;

#define TYPEDEF_MotorInput(X, _) \
    X(float, position, )         \
    X(float, velocity, )         \
    X(float, torque, )           \
    X(uint8_t, control_mode, )
MXGEN(struct, MotorInput)

#define TYPEDEF_MotorEstimate(X, _) \
    X(float, position, )            \
    X(float, velocity, )
MXGEN(struct, MotorEstimate)

#define TYPEDEF_MotorMsg(X, _) \
    X(MotorInput, input, )     \
    X(MotorEstimate, estimate, )
MXGEN(struct, MotorMsg)

////////////////////////////////////////////////////////////////////////////////
// Gamepad

typedef enum {
    GAMEPAD_BUTTON_SELECT = 1 << 0,
    GAMEPAD_BUTTON_L3 = 1 << 1,
    GAMEPAD_BUTTON_R3 = 1 << 2,
    GAMEPAD_BUTTON_START = 1 << 3,

    GAMEPAD_BUTTON_UP = 1 << 4,
    GAMEPAD_BUTTON_RIGHT = 1 << 5,
    GAMEPAD_BUTTON_DOWN = 1 << 6,
    GAMEPAD_BUTTON_LEFT = 1 << 7,

    GAMEPAD_BUTTON_L2 = 1 << 8,
    GAMEPAD_BUTTON_R2 = 1 << 9,
    GAMEPAD_BUTTON_L1 = 1 << 10,
    GAMEPAD_BUTTON_R1 = 1 << 11,

    GAMEPAD_BUTTON_TRIANGLE = 1 << 12,
    GAMEPAD_BUTTON_CIRCLE = 1 << 13,
    GAMEPAD_BUTTON_CROSS = 1 << 14,
    GAMEPAD_BUTTON_SQUARE = 1 << 15,
} GamepadButton;

#define TYPEDEF_GamepadStick(X, _) \
    X(int8_t, x, )                 \
    X(int8_t, y, )
MXGEN(struct, GamepadStick)

#define TYPEDEF_GamepadMsg(X, _)   \
    X(uint16_t, buttons, )         \
    X(GamepadStick, left_stick, )  \
    X(GamepadStick, right_stick, ) \
    X(uint8_t, left_trigger, )     \
    X(uint8_t, right_trigger, )
MXGEN(struct, GamepadMsg)

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
    MOTOR_CONTROL_MODE_POSITION = 1,
    MOTOR_CONTROL_MODE_VELOCITY = 2,
    MOTOR_CONTROL_MODE_TORQUE = 4,
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
    GAMEPAD_BUTTON_SELECT,
    GAMEPAD_BUTTON_L3,
    GAMEPAD_BUTTON_R3,
    GAMEPAD_BUTTON_START,

    GAMEPAD_BUTTON_UP,
    GAMEPAD_BUTTON_RIGHT,
    GAMEPAD_BUTTON_DOWN,
    GAMEPAD_BUTTON_LEFT,

    GAMEPAD_BUTTON_L2,
    GAMEPAD_BUTTON_R2,
    GAMEPAD_BUTTON_L1,
    GAMEPAD_BUTTON_R1,

    GAMEPAD_BUTTON_TRIANGLE,
    GAMEPAD_BUTTON_CIRCLE,
    GAMEPAD_BUTTON_CROSS,
    GAMEPAD_BUTTON_SQUARE,
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

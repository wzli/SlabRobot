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

#define TYPEDEF_MotorStatus(X, _) \
    X(uint32_t, error, )          \
    X(uint32_t, state, )
MXGEN(struct, MotorStatus)

#define TYPEDEF_MotorEstimate(X, _) \
    X(float, position, )            \
    X(float, velocity, )
MXGEN(struct, MotorEstimate)

#define TYPEDEF_MotorMode(X, _) \
    X(int32_t, control, )       \
    X(int32_t, input, )
MXGEN(struct, MotorMode)

#define TYPEDEF_MotorInput(X, _) \
    X(float, position, )         \
    X(float, velocity, )         \
    X(float, torque, )
MXGEN(struct, MotorInput)

#define TYPEDEF_MotorMsg(X, _)   \
    X(MotorStatus, status, )     \
    X(MotorEstimate, estimate, ) \
    X(MotorMode, mode, )         \
    X(MotorInput, input, )
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
    X(uint8_t, right_trigger, )    \
    X(uint8_t, stick_threshold, )
MXGEN(struct, GamepadMsg)

////////////////////////////////////////////////////////////////////////////////
typedef enum {
    MOTOR_ID_FRONT_LEGS,
    MOTOR_ID_BACK_LEGS,
    MOTOR_ID_FRONT_LEFT_WHEEL,
    MOTOR_ID_FRONT_RIGHT_WHEEL,
    MOTOR_ID_BACK_LEFT_WHEEL,
    MOTOR_ID_BACK_RIGHT_WHEEL,
} MotorId;

typedef enum {
    CONTROLLER_MODE_GROUND,
    CONTROLLER_MODE_BALANCE,
} ControllerMode;

typedef enum {
    AXIS_REMAP_X,
    AXIS_REMAP_Y,
    AXIS_REMAP_Z,
    AXIS_REMAP_NEG_FLAG = 4,
    AXIS_REMAP_NEG_X = 4,
    AXIS_REMAP_NEG_Y,
    AXIS_REMAP_NEG_Z,
} AxisRemap;

// wheel speed in (rad/s)
// wheel radius in (m)
// wheel distance in (m)
#define TYPEDEF_SlabConfig(X, _)  \
    X(float, max_wheel_speed, )   \
    X(float, wheel_diameter, )    \
    X(float, wheel_distance, )    \
    X(float, body_length, )       \
    X(float, leg_length, )        \
    X(float, max_leg_position, )  \
    X(float, min_leg_position, )  \
    X(float, leg_position_gain, ) \
    X(int8_t, imu_axis_remap, [3])
MXGEN(struct, SlabConfig)

#define TYPEDEF_SlabInput(X, _)  \
    X(float, linear_velocity, )  \
    X(float, angular_velocity, ) \
    X(float, leg_positions, [2]) \
    X(float, pitch_angle, )
MXGEN(struct, SlabInput)

#define TYPEDEF_Slab(X, _)        \
    X(uint32_t, tick, )           \
    X(MotorMsg, motors, [6])      \
    X(ImuMsg, imu, )              \
    X(GamepadMsg, gamepad, )      \
    X(uint8_t, controller_mode, ) \
    X(bool, inverted, )           \
    X(QuaternionF, orientation, ) \
    X(Vector3F, wheel_to_wheel, ) \
    X(SlabConfig, config, )       \
    X(SlabInput, input, )
MXGEN(struct, Slab)

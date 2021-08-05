#pragma once
#include "msg_defs.h"

typedef enum {
    MOTOR_ID_FRONT_LEGS,
    MOTOR_ID_BACK_LEGS,
    MOTOR_ID_FRONT_LEFT_WHEEL,
    MOTOR_ID_FRONT_RIGHT_WHEEL,
    MOTOR_ID_BACK_LEFT_WHEEL,
    MOTOR_ID_BACK_RIGHT_WHEEL,
} MotorId;

typedef enum {
    AXIS_REMAP_X,
    AXIS_REMAP_Y,
    AXIS_REMAP_Z,
    AXIS_REMAP_NEG_FLAG = 4,
    AXIS_REMAP_NEG_X = 4,
    AXIS_REMAP_NEG_Y,
    AXIS_REMAP_NEG_Z,
} AxisRemap;

// all angular velocities in (rad/s)
// all angular positions in (rad)
// all lengths in (m)
#define TYPEDEF_SlabConfig(X, _)      \
    X(float, max_wheel_speed, )       \
    X(float, wheel_diameter, )        \
    X(float, wheel_distance, )        \
    X(float, body_length, )           \
    X(float, leg_length, )            \
    X(float, max_leg_position, )      \
    X(float, min_leg_position, )      \
    X(int8_t, imu_axis_remap, [3])    \
    X(uint8_t, joystick_threshold, )  \
    X(float, ground_rise_threshold, ) \
    X(float, ground_fall_threshold, ) \
    X(float, incline_p_gain, )        \
    X(float, speed_p_gain, )          \
    X(float, speed_i_gain, )
MXGEN(struct, SlabConfig)

#define TYPEDEF_SlabInput(X, _)  \
    X(float, linear_velocity, )  \
    X(float, angular_velocity, ) \
    X(float, leg_positions, [2]) \
    X(float, body_incline, )     \
    X(bool, balance_enable, )
MXGEN(struct, SlabInput)

#define TYPEDEF_SlabState(X, _)      \
    X(QuaternionF, orientation, )    \
    _(Vector3F, verticies, [4])      \
    X(float, body_incline, )         \
    X(float, linear_velocity, )      \
    X(float, angular_velocity, )     \
    X(float, speed_error_integral, ) \
    X(uint8_t, ground_contacts, )    \
    X(bool, balance_active, )
MXGEN(struct, SlabState)

#define TYPEDEF_Slab(X, _)   \
    X(uint32_t, tick, )      \
    X(MotorMsg, motors, [6]) \
    X(ImuMsg, imu, )         \
    X(GamepadMsg, gamepad, ) \
    _(SlabConfig, config, )  \
    X(SlabInput, input, )    \
    X(SlabState, state, )
MXGEN(struct, Slab)

void slab_update(Slab* slab);

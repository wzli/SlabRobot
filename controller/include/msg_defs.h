#pragma once
#include "mxgen.h"

// IMU
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

// acceleration in units of (g)
// angular velocity in units of (rad/s)
#define TYPEDEF_ImuMsg(X, _)           \
    X(QuaternionF, orientation, )      \
    X(Vector3F, linear_acceleration, ) \
    X(Vector3F, angular_velocity, )
MXGEN(struct, ImuMsg)

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

typedef enum {
    MOTOR_ID_FRONT_LEGS,
    MOTOR_ID_BACK_LEGS,
    MOTOR_ID_FRONT_LEFT_WHEEL,
    MOTOR_ID_FRONT_RIGHT_WHEEL,
    MOTOR_ID_BACK_LEFT_WHEEL,
    MOTOR_ID_BACK_RIGHT_WHEEL,
} MotorID;

typedef enum {
    CONTROLLER_MODE_GROUND,
    CONTROLLER_MODE_BALANCE,
} ControllerMode;

// wheel speed in (rad/s)
// wheel radius in (m)
// wheel distance in (m)
#define TYPEDEF_SlabConfig(X, _) \
    X(float, max_wheel_speed, )  \
    X(float, wheel_diameter, )   \
    X(float, wheel_distance, )
MXGEN(struct, SlabConfig)

#define TYPEDEF_SlabInput(X, _)  \
    X(float, linear_velocity, )  \
    X(float, angular_velocity, ) \
    X(float, leg_positions, [2]) \
    X(float, pitch_angle, )
MXGEN(struct, SlabInput)

#define TYPEDEF_Slab(X, _)        \
    X(uint32_t, tick, )           \
    X(ImuMsg, imu, )              \
    X(MotorMsg, motors, [6])      \
    X(uint8_t, controller_mode, ) \
    X(bool, inverted, )           \
    X(SlabConfig, config, )       \
    X(SlabInput, input, )
MXGEN(struct, Slab)

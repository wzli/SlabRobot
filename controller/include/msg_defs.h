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

#define TYPEDEF_ImuMsg(X, _)           \
    X(QuaternionF, orientation, )      \
    X(Vector3F, linear_acceleration, ) \
    X(Vector3F, angular_velocity, )
MXGEN(struct, ImuMsg)

// Motor
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

// Controller
typedef enum {
    RECOVERY_MODE,
    SLAB_MODE,
    TABLE_MODE,
    BALANCE_MODE,
} ControllerMode;

#define TYPEDEF_ControllerContext(X, _) \
    X(ImuMsg, imu, )                    \
    X(MotorMsg, motors, [6])            \
    X(uint8_t, mode, )
// add params for each mode
// add reference body velocities, target modes, and desired trajectories
MXGEN(struct, ControllerContext)

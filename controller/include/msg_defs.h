#pragma once
#include "mxgen.h"

#define TYPEDEF_QuaternionI32(X, _) \
    X(int32_t, qw, )         \
    X(int32_t, qx, )         \
    X(int32_t, qy, )         \
    X(int32_t, qz, )
MXGEN(struct, QuaternionI32)

#define TYPEDEF_Vector3I16(X, _) \
    X(int16_t, x, )              \
    X(int16_t, y, )              \
    X(int16_t, z, )
MXGEN(struct, Vector3I16)

#define TYPEDEF_ImuMsg(X, _)             \
    X(QuaternionI32, orientation, )      \
    X(Vector3I16, linear_acceleration, ) \
    X(Vector3I16, angular_velocity, )
MXGEN(struct, ImuMsg)

#pragma once
#include "mxgen.h"

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

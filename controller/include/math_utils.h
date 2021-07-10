#pragma once
//#include <stddef.h>
//#include <stdint.h>
#include "msg_defs.h"
#include <math.h>

#define ABS(X) ((X) < 0 ? -(X) : (X))
#define SQR(X) ((X) * (X))
#define SGN(X) (((X) > 0) - ((X) < 0))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#define CLAMP(X, MIN, MAX) ((X) < (MIN) ? (MIN) : (X) > (MAX) ? (MAX) : (X))

#define VEC_DOT(OUT, A, B, LEN, STRIDE)                   \
    for (int dot_i = ((OUT) = 0); dot_i < (LEN); ++dot_i) \
    (OUT) += (A)[dot_i] * (B)[dot_i * (STRIDE)]

#define VEC_MUL(OUT, M, V, ROWS, COLS, STRIDE)         \
    for (int mul_row = 0; mul_row < (ROWS); ++mul_row) \
    VEC_DOT((OUT)[mul_row * STRIDE], (M) + mul_row * (COLS), V, COLS, STRIDE)

#define MAT_MUL(OUT, A, B, A_ROWS, A_COLS, B_COLS)       \
    for (int mul_col = 0; mul_col < (B_COLS); ++mul_col) \
    VEC_MUL((OUT) + mul_col, A, (B) + mul_col, A_ROWS, A_COLS, B_COLS)

void* memcpy(void*, const void*, size_t);

#define MAT_TRANSPOSE(MAT, DIM)                                       \
    for (int trans_row = 1; trans_row < (DIM); ++trans_row)           \
        for (int trans_col = 0; trans_col < trans_row; ++trans_col) { \
            uint8_t temp[sizeof((MAT)[0])];                           \
            void* trans_a = (MAT) + trans_col + (DIM) *trans_row;     \
            void* trans_b = (MAT) + trans_row + (DIM) *trans_col;     \
            memcpy(temp, trans_a, sizeof(temp));                      \
            memcpy(trans_a, trans_b, sizeof(temp));                   \
            memcpy(trans_b, temp, sizeof(temp));                      \
        }

// Q [w, x, y, z]
#define ROT_FROM_QUAT(R, Q)                           \
    (R)[0] = 1 - 2 * (SQR((Q)[2]) + SQR((Q)[3]));     \
    (R)[1] = 2 * ((Q)[1] * (Q)[2] - (Q)[0] * (Q)[3]); \
    (R)[2] = 2 * ((Q)[1] * (Q)[3] + (Q)[0] * (Q)[2]); \
    (R)[3] = 2 * ((Q)[1] * (Q)[2] + (Q)[0] * (Q)[3]); \
    (R)[4] = 1 - 2 * (SQR((Q)[1]) + SQR((Q)[3]));     \
    (R)[5] = 2 * ((Q)[2] * (Q)[3] + (Q)[0] * (Q)[1]); \
    (R)[6] = 2 * ((Q)[1] * (Q)[3] - (Q)[0] * (Q)[2]); \
    (R)[7] = 2 * ((Q)[2] * (Q)[3] - (Q)[0] * (Q)[1]); \
    (R)[8] = 1 - 2 * (SQR((Q)[1]) + SQR((Q)[2]));

static inline float rot_to_yaw(float* rot) {
    return atan2f(rot[1 * 3 + 0], rot[0 * 3 + 0]);
}

static inline float rot_to_pitch(float* rot) {
    return atan2f(-rot[2 * 3 + 0], sqrtf(SQR(rot[2 * 3 + 1]) + SQR(rot[2 * 3 + 2])));
}

static inline float rot_to_roll(float* rot) {
    return atan2f(rot[2 * 3 + 1], rot[2 * 3 + 2]);
}

static inline void to_euler(QuaternionF orn) {
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (orn.qw * orn.qx + orn.qy * orn.qz);
    float cosr_cosp = 1 - 2 * (orn.qx * orn.qx + orn.qy * orn.qy);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (orn.qw * orn.qy - orn.qz * orn.qx);
    float pitch;
    if (fabs(sinp) >= 1)
        pitch = M_PI / 2 * SGN(sinp);  // use 90 degrees if out of range
    else
        pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (orn.qw * orn.qz + orn.qx * orn.qy);
    float cosy_cosp = 1 - 2 * (orn.qy * orn.qy + orn.qz * orn.qz);
    float yaw = atan2f(siny_cosp, cosy_cosp);
    printf("r %f p %f y %f\n", roll, pitch, yaw);
}

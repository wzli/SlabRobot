#pragma once
#include <assert.h>
#include <stdint.h>
#include <math.h>

// generic functions

#define ABS(X) ((X) < 0 ? -(X) : (X))
#define SQR(X) ((X) * (X))
#define SGN(X) (((X) > 0) - ((X) < 0))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#define CLAMP(X, LO, HI) ((X) < (LO) ? (LO) : (X) > (HI) ? (HI) : (X))

#define XOR_SWAP_CASE(X, Y, W)                        \
    case (W) / 8:                                     \
        *(uint##W##_t*) &(X) ^= *(uint##W##_t*) &(Y); \
        *(uint##W##_t*) &(Y) ^= *(uint##W##_t*) &(X); \
        *(uint##W##_t*) &(X) ^= *(uint##W##_t*) &(Y); \
        break

#define XOR_SWAP(X, Y)                              \
    switch (sizeof(X) * (sizeof(X) == sizeof(Y))) { \
        XOR_SWAP_CASE(X, Y, 8);                     \
        XOR_SWAP_CASE(X, Y, 16);                    \
        XOR_SWAP_CASE(X, Y, 32);                    \
        XOR_SWAP_CASE(X, Y, 64);                    \
        default:                                    \
            assert(0);                              \
    }

// vector functions

#define VEC_ITERATE(INIT, LEN) for (int VEC_I = INIT; VEC_I < (LEN); ++VEC_I)

#define VEC_ADD(OUT, IN, SCALAR, LEN) \
    VEC_ITERATE(0, LEN) { (OUT)[VEC_I] = (IN)[VEC_I] + (SCALAR); }

#define VEC_MULTIPLY(OUT, IN, SCALAR, LEN) \
    VEC_ITERATE(0, LEN) { (OUT)[VEC_I] = (IN)[VEC_I] * (SCALAR); }

#define VEC_CLAMP(OUT, IN, LO, HI, LEN) \
    VEC_ITERATE(0, LEN) { (OUT)[VEC_I] = CLAMP((IN)[VEC_I], LO, HI); }

#define VEC_MAX(OUT, IN, LEN) \
    VEC_ITERATE(((OUT) = (IN)[0], 1), LEN) { (OUT) = MAX(OUT, (IN)[VEC_I]); }

#define VEC_MIN(OUT, IN, LEN) \
    VEC_ITERATE(((OUT) = (IN)[0], 1), LEN) { (OUT) = MIN(OUT, (IN)[VEC_I]); }

#define VEC_SUM(OUT, IN, LEN) \
    VEC_ITERATE(((OUT) = (IN)[0], 1), LEN) { (OUT) += (IN)[VEC_I]; }

#define VEC_DOT(OUT, A, B, LEN, STRIDE) \
    VEC_ITERATE(((OUT) = 0), LEN) { (OUT) += (A)[VEC_I] * (B)[VEC_I * (STRIDE)]; }

#define VEC3_DOT(A, B) ((A)[0] * (B)[0] + (A)[1] * (B)[1] + (A)[2] * (B)[2])

#define VEC3_CROSS(OUT, A, B)                         \
    do {                                              \
        (OUT)[0] = (A)[1] * (B)[2] - (A)[2] * (B)[1]; \
        (OUT)[1] = (A)[2] * (B)[0] - (A)[0] * (B)[2]; \
        (OUT)[2] = (A)[0] * (B)[1] - (A)[1] * (B)[0]; \
    } while (0)

#define VEC_OUTER(OUT, A, B, ROWS, COLS)                   \
    for (int out_row = 0; out_row < (ROWS); ++out_row)     \
        for (int out_col = 0; out_col < (COLS); ++out_col) \
    ((OUT)[out_col + (COLS) *out_row] = (A)[out_row] * (B)[out_col])

#define VEC_TRANSFORM(OUT, M, V, ROWS, COLS, STRIDE)   \
    for (int mul_row = 0; mul_row < (ROWS); ++mul_row) \
    VEC_DOT((OUT)[mul_row * STRIDE], (M) + mul_row * (COLS), V, COLS, STRIDE)

static inline float vec_norm(const float* v, int len) {
    float v_sqr;
    VEC_DOT(v_sqr, v, v, len, 1);
    return sqrtf(v_sqr);
}

static inline float vec_distance(const float* a, const float* b, int len) {
    float ab[len];
    VEC_ADD(ab, b, -a[VEC_I], len);
    return vec_norm(ab, len);
}

static inline void vec_normalize(float* out, const float* in, int len) {
    float inv_norm = 1 / vec_norm(in, len);
    VEC_MULTIPLY(out, in, inv_norm, len);
}

static inline float vec_average(const float* v, int len) {
    float sum;
    VEC_SUM(sum, v, len);
    return sum / len;
}

// matrix functions

#define MAT_MULTIPLY(OUT, A, B, A_ROWS, A_COLS, B_COLS)  \
    for (int mul_col = 0; mul_col < (B_COLS); ++mul_col) \
    VEC_TRANSFORM((OUT) + mul_col, A, (B) + mul_col, A_ROWS, A_COLS, B_COLS)

#define MAT_TRANSPOSE(MAT, DIM)                                     \
    for (int trans_row = 1; trans_row < (DIM); ++trans_row)         \
        for (int trans_col = 0; trans_col < trans_row; ++trans_col) \
    XOR_SWAP((MAT)[trans_col + (DIM) *trans_row], (MAT)[trans_row + (DIM) *trans_col])

static inline float rot_to_yaw(const float* rot) {
    return atan2f(rot[3], rot[0]);
}

static inline float rot_to_pitch(const float* rot) {
    return atan2f(-rot[6], sqrtf(SQR(rot[7]) + SQR(rot[8])));
}

static inline float rot_to_roll(const float* rot) {
    return atan2f(rot[7], rot[8]);
}

// quaternion functions [w, x, y, z]

#define QUAT_MULTIPLY(OUT, A, B)                                                         \
    do {                                                                                 \
        (OUT)[0] = (A)[0] * (B)[0] - VEC3_DOT((A) + 1, (B) + 1);                         \
        VEC3_CROSS((OUT) + 1, (A) + 1, (B) + 1);                                         \
        VEC_ITERATE(1, 4) { (OUT)[VEC_I] += (A)[0] * (B)[VEC_I] + (B)[0] * (A)[VEC_I]; } \
    } while (0)

#define QUAT_TRANSFORM(OUT, Q, V)                                                                \
    do {                                                                                         \
        VEC3_CROSS(OUT, (Q) + 1, V);                                                             \
        VEC_ITERATE(0, 3) {                                                                      \
            (OUT)[VEC_I] = 2 * ((Q)[0] * (OUT)[VEC_I] + VEC3_DOT((Q) + 1, V) * (Q)[VEC_I + 1]) + \
                           ((2 * SQR((Q)[0]) - 1) * (V)[VEC_I]);                                 \
        }                                                                                        \
    } while (0)

#define QUAT_TO_ROTATION_MATRIX(R, Q)                     \
    do {                                                  \
        (R)[0] = 1 - 2 * (SQR((Q)[2]) + SQR((Q)[3]));     \
        (R)[1] = 2 * ((Q)[1] * (Q)[2] - (Q)[0] * (Q)[3]); \
        (R)[2] = 2 * ((Q)[1] * (Q)[3] + (Q)[0] * (Q)[2]); \
        (R)[3] = 2 * ((Q)[1] * (Q)[2] + (Q)[0] * (Q)[3]); \
        (R)[4] = 1 - 2 * (SQR((Q)[1]) + SQR((Q)[3]));     \
        (R)[5] = 2 * ((Q)[2] * (Q)[3] - (Q)[0] * (Q)[1]); \
        (R)[6] = 2 * ((Q)[1] * (Q)[3] - (Q)[0] * (Q)[2]); \
        (R)[7] = 2 * ((Q)[2] * (Q)[3] + (Q)[0] * (Q)[1]); \
        (R)[8] = 1 - 2 * (SQR((Q)[1]) + SQR((Q)[2]));     \
    } while (0)

#define QUAT_CONJUGATE(OUT, IN) VEC_ITERATE(((OUT)[0] = (IN)[0], 1), 4)(OUT)[VEC_I] = -(IN)[VEC_I]

static inline void quat_normalize(float* q) {
    vec_normalize(q, q, 4);
}

static inline void quat_from_angle_axis(float* q, float angle, const float* axis) {
    float sin_half = sinf(angle / 2);
    vec_normalize(q + 1, axis, 3);
    VEC_MULTIPLY(q + 1, q + 1, sin_half, 3);
    q[0] = cosf(angle / 2);
}

static inline float quat_to_yaw(const float* q) {
    float siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    return atan2f(siny_cosp, cosy_cosp);
}

static inline float quat_to_pitch(const float* q) {
    float sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    return fabs(sinp) < 1 ? asinf(sinp) : M_PI / 2 * SGN(sinp);
}

static inline float quat_to_roll(const float* q) {
    float sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    return atan2f(sinr_cosp, cosr_cosp);
}

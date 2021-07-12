#include <assert.h>
#include "math_utils.h"
#include "slab_controller.h"

void matrix_print(float* m, int rows, int cols) {
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            printf("%g\t", m[col + row * cols]);
        }
        puts("");
    }
    puts("");
}

static void axis_remap(Vector3F* out, const Vector3F* in, const int8_t* remap) {
    for (int i = 0; i < 3; ++i) {
        int axis = remap[i] & 3;
        assert(axis < 3);
        ((float*) out)[i] = ((float*) in)[axis];
        if (remap[i] & AXIS_REMAP_NEG_FLAG) {
            ((float*) out)[i] *= -1;
        }
    }
}

static void imu_axis_remap(ImuMsg* imu, const int8_t* remap) {
    ImuMsg tmp = *imu;
    axis_remap((Vector3F*) &imu->orientation.qx, (Vector3F*) &tmp.orientation.qx, remap);
    axis_remap(&imu->linear_acceleration, &tmp.linear_acceleration, remap);
    axis_remap(&imu->angular_velocity, &tmp.angular_velocity, remap);
}

static void slab_gamepad_input_update(Slab* slab) {
    // parse dpad to normalized vector
    float dpad_x = ((slab->gamepad.buttons >> GAMEPAD_BUTTON_LEFT) & 1) -
                   ((slab->gamepad.buttons >> GAMEPAD_BUTTON_RIGHT) & 1);
    float dpad_y = ((slab->gamepad.buttons >> GAMEPAD_BUTTON_UP) & 1) -
                   ((slab->gamepad.buttons >> GAMEPAD_BUTTON_DOWN) & 1);
    if (dpad_x != 0 && dpad_y != 0) {
        float inv_norm = 1.0f / sqrtf(SQR(dpad_x) + SQR(dpad_y));
        dpad_x *= inv_norm;
        dpad_y *= inv_norm;
    }
    // parse stick to [lx, ly, rx, ry] normalized to 1
    float stick[4] = {0};
    for (int i = 0; i < 4; ++i) {
        if (ABS((&slab->gamepad.left_stick.x)[i]) >= slab->gamepad.stick_threshold) {
            stick[i] = -((&slab->gamepad.left_stick.x)[i] + 0.5f) / 127.5f;
        }
    }
    // scale velocity input by right trigger)
    float speed_scale = 0.5f * slab->config.wheel_diameter * slab->config.max_wheel_speed *
                        slab->gamepad.right_trigger / 255.0f;
    // map control to velocity input (dpad overrides stick)
    if (dpad_x != 0 || dpad_y != 0) {
        slab->input.linear_velocity = dpad_y * speed_scale;
        slab->input.angular_velocity = dpad_x * speed_scale / (0.5f * slab->config.wheel_distance);
    } else {
        slab->input.linear_velocity = stick[1] * speed_scale;
        slab->input.angular_velocity = stick[0] * speed_scale / slab->config.wheel_distance;
    }
    // map stick y-axis to leg positions
    float leg_positions[2] = {slab->input.leg_positions[0], slab->input.leg_positions[1]};
    if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_START) & 1) {
        leg_positions[MOTOR_ID_FRONT_LEGS] = -M_PI;
        leg_positions[MOTOR_ID_BACK_LEGS] = -M_PI;
    } else if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_TRIANGLE) & 1) {
        leg_positions[MOTOR_ID_FRONT_LEGS] = -M_PI / 2;
        leg_positions[MOTOR_ID_BACK_LEGS] = M_PI / 2;
    } else if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_SQUARE) & 1) {
        leg_positions[MOTOR_ID_FRONT_LEGS] = M_PI / 2;
        leg_positions[MOTOR_ID_BACK_LEGS] = -M_PI / 2;
    } else {
#if 0
        for (int i = 0; i < 2; ++i) {
            leg_positions[i] = (slab->gamepad.buttons >> (GAMEPAD_BUTTON_L3 + i)) & 1
                                       ? 0
                                       : leg_positions[i] - (stick[1 + 2 * i]);
        }
#else
        leg_positions[MOTOR_ID_FRONT_LEGS] -= (stick[2] - stick[3]);
        leg_positions[MOTOR_ID_BACK_LEGS] -= (stick[2] + stick[3]);

#endif
    }
    for (int i = 0; i < 2; ++i) {
        slab->input.leg_positions[i] +=
                (leg_positions[i] - slab->input.leg_positions[i]) * slab->config.leg_position_gain;
        slab->input.leg_positions[i] = CLAMP(slab->input.leg_positions[i],
                slab->config.min_leg_position, slab->config.max_leg_position);
    }
    // TODO try j
    // map controller mode to L1 button
    slab->controller_mode = (slab->gamepad.buttons >> GAMEPAD_BUTTON_L1) & 1;
}

static void slab_ground_controller_update(Slab* slab) {
    const float r = slab->config.wheel_diameter / 2;
    const float R = slab->config.wheel_distance / 2;
    const float v_max = slab->config.max_wheel_speed;
    const float v_lin = slab->input.linear_velocity;
    const float v_ang = slab->input.angular_velocity;
    float wheel_speeds[2] = {(v_lin - v_ang * R) / r, (v_lin + v_ang * R) / r};
    for (int i = 0; i < 2; ++i) {
        wheel_speeds[i] = CLAMP(wheel_speeds[i], -v_max, v_max);
        slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL + i].input.velocity = wheel_speeds[i];
        slab->motors[MOTOR_ID_BACK_LEFT_WHEEL + i].input.velocity = wheel_speeds[i];
        slab->motors[i].input.position = slab->input.leg_positions[i];
    }
}

static void slab_balance_controller_update(Slab* slab) {}

void slab_update(Slab* slab) {
    slab_gamepad_input_update(slab);
    switch (slab->controller_mode) {
        case CONTROLLER_MODE_GROUND:
            slab_ground_controller_update(slab);
            break;
        case CONTROLLER_MODE_BALANCE:
            slab_balance_controller_update(slab);
            break;
    }
    imu_axis_remap(&slab->imu, slab->config.imu_axis_remap);
    float* q = (float*) &slab->imu.orientation;
    float rot[3 * 3];
    QUAT_TO_ROTATION_MATRIX(rot, q);
    printf("a r %f p %f y %f\n", quat_to_roll(q), quat_to_pitch(q), quat_to_yaw(q));
    printf("b r %f p %f y %f\n", rot_to_roll(rot), rot_to_pitch(rot), rot_to_yaw(rot));

#if 1
    // float a[2 * 2] = {3, 5, -1, 1};
    // float b[2 * 3] = {-2, 2, 3, 3, 5, -2};
    // float c[4] = {-5, 3, 4, 3};
    // float d[4] = {4, 3.9, -1, -3};

    quat_normalize(q);
    float e[4] = {1, 2, 3, 4};
    float f[4] = {0};
    float g[4] = {0};
    QUAT_CONJUGATE(f, q);
    QUAT_MULTIPLY(g, e, f);
    QUAT_MULTIPLY(f, q, g);
    VEC_TRANSFORM(g + 1, rot, e + 1, 3, 3, 1);
    g[0] = 0;

    // QUAT_TRANSFORM(f + 1, q, e + 1);

    // MAT_TRANSPOSE(a, 2);

    // MAT_ADD(d, a, a, 2, 2);
    // matrix_print(c, 3, 1);
    // quat_from_angle_axis(d, M_PI / 2, c);
    // QUAT_MULTIPLY(e, c, d);
    puts("");
    // matrix_print(f, 1, 4);
    // matrix_print(g, 1, 4);

    VEC_MULTIPLY(e, e, e[elem_i], 4);
    matrix_print(e, 1, 4);
    float k[4] = {0};
    float dist = vec_distance(k, e, 4);
    float max, min;
    VEC_MAX(max, e, 4);
    VEC_MIN(min, e, 4);
    printf("%f %f %f\n", dist, max, min);

    // VEC_MUL(c, a, b, 2, 2, 3);
    // MAT_MULTIPLY(c, a, b, 2, 2, 3);

    // matrix_print(c, 2, 3);

    // float out = 0;
    // DOT(out, a, b, 3, 1);
    // printf("= %f\n", out);
#endif
}

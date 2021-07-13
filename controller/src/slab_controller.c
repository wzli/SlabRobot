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

static void slab_gamepad_input_update(Slab* slab) {
    // parse stick to [lx, ly, rx, ry] normalized to 1
    float stick[4] = {0};
    for (int i = 0; i < 4; ++i) {
        if (ABS((&slab->gamepad.left_stick.x)[i]) >= slab->config.joystick_threshold) {
            stick[i] = -((&slab->gamepad.left_stick.x)[i] + 0.5f) / 127.5f;
        }
    }
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
    // scale velocity input to max wheel speed
    float speed_scale = 0.5f * slab->config.wheel_diameter * slab->config.max_wheel_speed;
    // map L-stick to velocity input (dpad overrides stick)
    if (dpad_x != 0 || dpad_y != 0) {
        speed_scale *= slab->gamepad.left_trigger / 255.0f;
        slab->input.linear_velocity = dpad_y * speed_scale;
        slab->input.angular_velocity = dpad_x * speed_scale / (0.5f * slab->config.wheel_distance);
    } else {
        slab->input.linear_velocity = stick[1] * speed_scale;
        slab->input.angular_velocity = stick[0] * speed_scale / slab->config.wheel_distance;
    }
    // map R-stick and buttons to leg positions
    float smoothing = 0.05f * (slab->gamepad.right_trigger / 255.0f);
    for (int i = 0, sign = -1; i < 2; ++i, sign *= -1) {
        if (slab->state.ground_contact == i + 1) {
            continue;
        }
        if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_START) & 1) {
            LOW_PASS_FILTER(slab->input.leg_positions[i], -M_PI, smoothing);
        } else if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_TRIANGLE) & 1) {
            LOW_PASS_FILTER(slab->input.leg_positions[i], sign * M_PI / 2, smoothing);
        } else if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_SQUARE) & 1) {
            LOW_PASS_FILTER(slab->input.leg_positions[i], -sign * M_PI / 2, smoothing);
        } else {
            slab->input.leg_positions[i] -= 0.05f * (stick[2] + stick[3] * sign);
            slab->input.leg_positions[i] = CLAMP(slab->input.leg_positions[i],
                    slab->config.min_leg_position, slab->config.max_leg_position);
        }
    }
    if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_CIRCLE) & 1) {
        slab->input.body_incline += 0.005;
        slab->state.speed_error_integral += 0.01 / slab->config.speed_i_gain;
    } else if ((slab->gamepad.buttons >> GAMEPAD_BUTTON_CROSS) & 1) {
        slab->input.body_incline -= 0.005;
        slab->state.speed_error_integral -= 0.01 / slab->config.speed_i_gain;
    }
}

static void slab_differential_drive_update(Slab* slab, float v_lin, float v_ang) {
    const float r = slab->config.wheel_diameter / 2;
    const float R = slab->config.wheel_distance / 2;
    const float v_max = slab->config.max_wheel_speed;
    float left_wheel_speed = (v_lin - v_ang * R) / r;
    float right_wheel_speed = (v_lin + v_ang * R) / r;
    left_wheel_speed = CLAMP(left_wheel_speed, -v_max, v_max);
    right_wheel_speed = CLAMP(right_wheel_speed, -v_max, v_max);
    uint8_t legs = slab->state.ground_contact;
    slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL].input.velocity = (legs & 1) ? left_wheel_speed : 0;
    slab->motors[MOTOR_ID_FRONT_RIGHT_WHEEL].input.velocity = (legs & 1) ? right_wheel_speed : 0;
    slab->motors[MOTOR_ID_BACK_LEFT_WHEEL].input.velocity = (legs & 2) ? left_wheel_speed : 0;
    slab->motors[MOTOR_ID_BACK_RIGHT_WHEEL].input.velocity = (legs & 2) ? right_wheel_speed : 0;
}

static void slab_ground_controller_update(Slab* slab) {
    slab_differential_drive_update(slab, slab->input.linear_velocity, slab->input.angular_velocity);
    for (int i = 0; i < 2; ++i) {
        slab->motors[MOTOR_ID_FRONT_LEGS + i].input.position = CLAMP(slab->input.leg_positions[i],
                slab->config.min_leg_position, slab->config.max_leg_position);
    }
}

static void slab_balance_controller_update(Slab* slab) {
    float incline_error = slab->input.body_incline - slab->state.body_incline;
    float speed = incline_error * slab->config.incline_p_gain;
    // TODO compute speed feedback from encoders instead
    float speed_error = slab->input.linear_velocity - speed;
    slab->state.speed_error_integral += speed_error;
    slab_differential_drive_update(slab, speed, slab->input.angular_velocity);
    float leg_positions[2] = {slab->input.leg_positions[0], slab->input.leg_positions[1]};
    leg_positions[slab->state.ground_contact - 1] -=
            (speed_error * slab->config.speed_p_gain) +
            (slab->state.speed_error_integral * slab->config.speed_i_gain);
    for (int i = 0; i < 2; ++i) {
        slab->motors[MOTOR_ID_FRONT_LEGS + i].input.position = CLAMP(
                leg_positions[i], slab->config.min_leg_position, slab->config.max_leg_position);
    }

    // slab_ground_controller_update(slab);
    printf("r %f e %f v %f\n", slab->state.body_incline, incline_error,
            slab->motors[2].input.velocity);
}

static void slab_state_update(Slab* slab) {
    // remap axis such that [x -> right, y -> forward, z -> up]
    slab->state.orientation.qw = slab->imu.orientation.qw;
    axis_remap((Vector3F*) &slab->state.orientation.qx, (Vector3F*) &slab->imu.orientation.qx,
            slab->config.imu_axis_remap);
    // calculate roll
    slab->state.body_incline = quat_to_roll((float*) &slab->state.orientation);
    // calculate wheel to wheel vector
    float wheel_to_wheel[3] = {0,
            slab->config.body_length +
                    slab->config.leg_length *
                            (cosf(slab->motors[MOTOR_ID_FRONT_LEGS].estimate.position) +
                                    cosf(slab->motors[MOTOR_ID_BACK_LEGS].estimate.position)),
            slab->config.leg_length *
                    (sinf(slab->motors[MOTOR_ID_FRONT_LEGS].estimate.position) +
                            sinf(slab->motors[MOTOR_ID_BACK_LEGS].estimate.position))};
    QUAT_TRANSFORM((float*) &slab->state.wheel_to_wheel, (float*) &slab->state.orientation,
            wheel_to_wheel);
    // TODO transform wheel velocity to body frame

    // map controller mode to L1 button
    GroundContact ground_contact = (slab->gamepad.buttons >> GAMEPAD_BUTTON_L1) & 1
                                           ? (slab->state.wheel_to_wheel.z > 0) + 1
                                           : GROUND_CONTACT_BOTH;
    if (slab->state.ground_contact != ground_contact) {
        slab->input.body_incline = slab->state.body_incline;
        slab->input.linear_velocity = 0;
        slab->input.leg_positions[0] = slab->motors[0].estimate.position;
        slab->input.leg_positions[1] = slab->motors[1].estimate.position;
        slab->state.speed_error_integral = 0;
    }
    slab->state.ground_contact = ground_contact;
}

void slab_update(Slab* slab) {
    slab_state_update(slab);
    slab_gamepad_input_update(slab);
    switch (slab->state.ground_contact) {
        case GROUND_CONTACT_FRONT:
        case GROUND_CONTACT_BACK:
            slab_balance_controller_update(slab);
            break;
        default:
            slab_ground_controller_update(slab);
            break;
    }
#if 0
    imu_axis_remap(&slab->imu, slab->config.imu_axis_remap);
    float* q = (float*) &slab->imu.orientation;
    float rot[3 * 3];
    QUAT_TO_ROTATION_MATRIX(rot, q);
    printf("a r %f p %f y %f\n", quat_to_roll(q), quat_to_pitch(q), quat_to_yaw(q));
    printf("b r %f p %f y %f\n", rot_to_roll(rot), rot_to_pitch(rot), rot_to_yaw(rot));

    // float a[2 * 2] = {3, 5, -1, 1};
    // float b[2 * 3] = {-2, 2, 3, 3, 5, -2};
    // float c[4] = {-5, 3, 4, 3};
    // float d[4] = {4, 3.9, -1, -3};

    quat_normalize(q);
    float e[4] = {0, 1, 0, 0};
    // float e[4] = {1, 2, 3, 4};
    float f[4] = {0};
    float g[4] = {0};
    QUAT_CONJUGATE(f, q);
    QUAT_MULTIPLY(g, e, f);
    QUAT_MULTIPLY(f, q, g);
    VEC_TRANSFORM(g + 1, rot, e + 1, 3, 3, 1);
    // QUAT_TRANSFORM(f + 1, q, e + 1);
    f[0] = 0;
    g[0] = 0;

    // MAT_TRANSPOSE(a, 2);

    // MAT_ADD(d, a, a, 2, 2);
    // matrix_print(c, 3, 1);
    // quat_from_angle_axis(d, M_PI / 2, c);
    // QUAT_MULTIPLY(e, c, d);
    puts("");
    matrix_print(f, 1, 4);
    matrix_print(g, 1, 4);

    VEC_OUTER(e, e, e, 4, 4);
    VEC_MULTIPLY(e, e, e[VEC_I], 4);
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

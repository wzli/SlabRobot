#include "math_utils.h"
#include "slab.h"

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
    enum { X, Y, RX, RY };
    // parse stick to [lx, ly, rx, ry] normalized to 1
    float sticks[4] = {0};
    for (int i = 0; i < 4; ++i) {
        if (ABS((&slab->gamepad.left_stick.x)[i]) >= slab->config.joystick_threshold) {
            sticks[i] = -((&slab->gamepad.left_stick.x)[i] + 0.5f) / 127.5f;
        }
    }
    // parse buttons to axis vector
    float buttons[4] = {(bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_LEFT) -
                                (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_RIGHT),
            (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_UP) -
                    (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_DOWN),
            (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_SQUARE) -
                    (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_CIRCLE),
            (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_TRIANGLE) -
                    (bool) (slab->gamepad.buttons & GAMEPAD_BUTTON_CROSS)};
    // normalize button axis and scale by trigger
    for (int i = 0; i < 2; ++i) {
        float* axis = buttons + (2 * i);
        if (axis[X] != 0 && axis[Y] != 0) {
            float inv_norm = 1.0f / vec_norm(axis, 2);
            axis[X] *= inv_norm;
            axis[Y] *= inv_norm;
        }
        float trigger = (&slab->gamepad.left_trigger)[i] / 255.0f;
        axis[X] *= trigger;
        axis[Y] *= trigger;
    }
    // disable balance control with L1
    slab->input.balance_enable = !(slab->gamepad.buttons & GAMEPAD_BUTTON_L1);
    // scale velocity input to max wheel speed
    float speed_scale = 0.5f * slab->config.wheel_diameter * slab->config.max_wheel_speed;
    if (slab->state.balance_active) {
        speed_scale /= 4;
    }
    // map L-stick to velocity input (dpad overrides stick)
    float* velocity_input = buttons[X] == 0 && buttons[Y] == 0 ? sticks : buttons;
    slab->input.linear_velocity = velocity_input[Y] * speed_scale;
    slab->input.angular_velocity =
            velocity_input[X] * speed_scale / (0.5f * slab->config.wheel_distance);
    // map R-stick to velocity input (button overrides stick)
    float* legs_input = (buttons[RX] == 0 && buttons[RY] == 0 ? sticks : buttons) + 2;
    // leg position control logic
    const float leg_speed = 0.02f;
    for (int i = 0, sign = -1; i < 2; ++i, sign *= -1) {
        // holding R1 targets preset angles
        if (slab->gamepad.buttons & GAMEPAD_BUTTON_R1) {
            // exclude the leg that the robot is balancing on
            if (slab->state.balance_active && slab->state.ground_contacts == (1 << i)) {
                continue;
            }
            // if input lies in horizontal vs vertical regions
            if (SQR(legs_input[X]) > SQR(legs_input[Y])) {
                // target PI angle presets (fold down)
                LOW_PASS_FILTER(slab->input.leg_positions[i], SGN(legs_input[X]) * M_PI,
                        leg_speed * ABS(legs_input[X]));
                // target vertical body incline (only effective when balancing)
                LOW_PASS_FILTER(slab->input.body_incline, SGN(slab->input.body_incline) * M_PI_2,
                        leg_speed * ABS(legs_input[X]));
            } else {
                // target PI/2 angle presets (right angle)
                LOW_PASS_FILTER(slab->input.leg_positions[i], sign * SGN(legs_input[Y]) * M_PI_2,
                        leg_speed * ABS(legs_input[Y]));
                // target horizontal body incline (only effective when balancing)
                LOW_PASS_FILTER(slab->input.body_incline,
                        (SGN(slab->input.body_incline) + sign * SGN(legs_input[Y])) * M_PI_2,
                        0.5f * leg_speed * ABS(legs_input[Y]));
            }
        }
        // R1 released for direct control
        else {
            // map X to arm and Y to incline during balance
            if (slab->state.balance_active) {
                // remap controls of balancing leg to body incline
                if (slab->state.ground_contacts == (1 << i)) {
                    slab->input.body_incline -= 0.5f * leg_speed * legs_input[Y];
                    slab->input.body_incline = CLAMP(slab->input.body_incline, -M_PI, M_PI);
                    // compensate balancing leg toward the direction of the incline movement
                    slab->input.leg_positions[i] += leg_speed * legs_input[Y];
                } else {
                    slab->input.leg_positions[i] += leg_speed * legs_input[X];
                }
            } else {
                // map legs movement to diagonal axis
                slab->input.leg_positions[i] += leg_speed * (legs_input[X] + legs_input[Y] * sign);
            }
            // clamp leg positions
            slab->input.leg_positions[i] = CLAMP(slab->input.leg_positions[i],
                    slab->config.min_leg_position, slab->config.max_leg_position);
        }
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
    uint8_t legs = slab->state.ground_contacts;
    slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL].input.velocity = (legs & 1) ? left_wheel_speed : 0;
    slab->motors[MOTOR_ID_FRONT_RIGHT_WHEEL].input.velocity = (legs & 1) ? right_wheel_speed : 0;
    slab->motors[MOTOR_ID_BACK_LEFT_WHEEL].input.velocity = (legs & 2) ? left_wheel_speed : 0;
    slab->motors[MOTOR_ID_BACK_RIGHT_WHEEL].input.velocity = (legs & 2) ? right_wheel_speed : 0;
    // velocity control mode
    for (int i = 0; i < 4; ++i) {
        slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL + i].input.control_mode =
                MOTOR_CONTROL_MODE_VELOCITY;
    }
}

static void slab_ground_controller_update(Slab* slab) {
    slab_differential_drive_update(
            slab, slab->input.linear_velocity, slab->input.angular_velocity);
    for (int i = 0; i < 2; ++i) {
        slab->motors[i].input.position = CLAMP(slab->input.leg_positions[i],
                slab->config.min_leg_position, slab->config.max_leg_position);
        slab->motors[i].input.control_mode = MOTOR_CONTROL_MODE_POSITION;
    }
}

static void slab_balance_controller_update(Slab* slab) {
    // use body speed to control body incline
    float incline_error = slab->input.body_incline - slab->state.body_incline;
    float speed_output = incline_error * slab->config.incline_p_gain;
    slab_differential_drive_update(slab, speed_output, slab->input.angular_velocity);
    // use leg with ground contact to control body speed
    float speed_error = slab->input.linear_velocity - slab->state.linear_velocity;
    slab->state.speed_error_integral += speed_error;
    float leg_position_bias = (speed_error * slab->config.speed_p_gain) +
                              (slab->state.speed_error_integral * slab->config.speed_i_gain);
    for (int i = 0; i < 2; ++i) {
        float leg_position_output = slab->input.leg_positions[i] -
                                    GET_BIT(slab->state.ground_contacts, i) * leg_position_bias;
        slab->motors[i].input.position = CLAMP(
                leg_position_output, slab->config.min_leg_position, slab->config.max_leg_position);
        slab->motors[i].input.control_mode = MOTOR_CONTROL_MODE_POSITION;
    }
}

static void slab_state_update(Slab* slab) {
    // remap axis such that [x -> right, y -> forward, z -> up]
    slab->state.orientation.qw = slab->imu.orientation.qw;
    axis_remap((Vector3F*) &slab->state.orientation.qx, (Vector3F*) &slab->imu.orientation.qx,
            slab->config.imu_axis_remap);
    // calculate roll
    slab->state.body_incline = quat_to_roll((float*) &slab->state.orientation);
    // calculate ground contacts from verticies
    const float L = slab->config.body_length / 2;
    const float l = slab->config.leg_length;
    const float a = slab->motors[MOTOR_ID_FRONT_LEGS].estimate.position;
    const float b = slab->motors[MOTOR_ID_BACK_LEGS].estimate.position;
    Vector3F verticies_local[4] = {
            {0, L + l * cosf(a), l * sinf(a)},
            {0, -(L + l * cosf(b)), -(l * sinf(b))},
            {0, L, 0},
            {0, -L, 0},
    };
    // convert verticies to global frame
    for (int i = 0; i < 4; ++i) {
        QUAT_TRANSFORM((float*) &slab->state.verticies[i], (float*) &slab->state.orientation,
                (float*) &verticies_local[i]);
    }
    // find ground level
    float ground_z = slab->state.verticies[0].z;
    for (int i = 1; i < 4; ++i) {
        ground_z = MIN(ground_z, slab->state.verticies[i].z);
    }
    for (int i = 0; i < 4; ++i) {
        slab->state.verticies[i].z -= ground_z;
        if (GET_BIT(slab->state.ground_contacts, i)) {
            if (slab->state.verticies[i].z > slab->config.ground_rise_threshold) {
                slab->state.ground_contacts &= ~(1 << i);
            }
        } else {
            if (slab->state.verticies[i].z < slab->config.ground_fall_threshold) {
                slab->state.ground_contacts |= 1 << i;
            }
        }
    }
    // compute body velocity based on speed feedback of motors with ground
    // contact
    // TODO compute angular velocity, maybe based on gyro
    // keep in mind skid stear ratio is different depending on leg positions
    slab->state.linear_velocity = 0;
    int ground_wheels = 0;
    for (int i = 0; i < 4; ++i) {
        if (GET_BIT(slab->state.ground_contacts, i / 2)) {
            ++ground_wheels;
            slab->state.linear_velocity += slab->motors[2 + i].estimate.velocity;
        }
    }
    if (ground_wheels) {
        slab->state.linear_velocity *= 0.5f * slab->config.wheel_diameter / ground_wheels;
    }
    // turn on balance control when only one leg has ground contact
    bool balance_active = slab->input.balance_enable &&
                          (slab->state.ground_contacts == 1 || slab->state.ground_contacts == 2);
    // reset control states on transition
    if (slab->state.balance_active != balance_active) {
        slab->state.balance_active = balance_active;
        slab->input.body_incline = slab->state.body_incline;
        slab->input.linear_velocity = 0;
        slab->input.leg_positions[0] = slab->motors[0].estimate.position;
        slab->input.leg_positions[1] = slab->motors[1].estimate.position;
        slab->state.speed_error_integral = 0;
    }
}

void slab_update(Slab* slab) {
    assert(slab);
    slab_state_update(slab);
    slab_gamepad_input_update(slab);
    if (slab->state.balance_active) {
        slab_balance_controller_update(slab);
    } else {
        slab_ground_controller_update(slab);
    }
}

#include <assert.h>
#include <math.h>

#include "slab_controller.h"

#define ABS(X) ((X) < 0 ? -(X) : (X))
#define SQR(X) ((X) * (X))

static inline float clampf(float x, float min, float max) {
    return fmax(min, fmin(max, x));
}

void slab_gamepad_input_update(Slab* slab) {
    assert(slab);
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
    // map dpad to velocity input (scaled by right trigger)
    float speed_scale = 0.5f * slab->config.wheel_diameter * slab->config.max_wheel_speed *
                        slab->gamepad.trigger[1] / 255.0f;
    slab->input.linear_velocity = dpad_y * speed_scale;
    slab->input.angular_velocity = dpad_x * speed_scale / (0.5f * slab->config.wheel_distance);
    // parse stick to [lx, ly, rx, ry] with values between +-1
    float stick[4] = {0};
    for (int i = 0; i < 4; ++i) {
        if (ABS(slab->gamepad.left_stick[i]) >= slab->gamepad.stick_threshold) {
            stick[i] = (slab->gamepad.left_stick[i] + 0.5f) / 127.5f;
        }
    }
    // map stick y-axis to leg positions
    // TODO add sensitivity as config
    for (int i = 0; i < 2; ++i) {
        slab->input.leg_positions[i] -= stick[1 + 2 * i] / 200;
        slab->input.leg_positions[i] = clampf(slab->input.leg_positions[i],
                slab->config.min_leg_position, slab->config.max_leg_position);
    }
}

void slab_ground_controller_update(Slab* slab) {
    assert(slab);
    const float r = slab->config.wheel_diameter / 2;
    const float R = slab->config.wheel_distance / 2;
    const float v_max = slab->config.max_wheel_speed;
    const float v_lin = slab->input.linear_velocity;
    const float v_ang = slab->input.angular_velocity;
    float wheel_speeds[2] = {(v_lin - v_ang * R) / r, (v_lin + v_ang * R) / r};
    for (int i = 0; i < 2; ++i) {
        wheel_speeds[i] = clampf(wheel_speeds[i], -v_max, v_max);
        slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL + i].input.velocity = wheel_speeds[i];
        slab->motors[MOTOR_ID_BACK_LEFT_WHEEL + i].input.velocity = wheel_speeds[i];
        slab->motors[i].input.position = slab->input.leg_positions[i];
    }
}

void slab_update(Slab* slab) {
    assert(slab);
    slab_gamepad_input_update(slab);
    slab_ground_controller_update(slab);
}

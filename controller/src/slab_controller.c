#include "slab_controller.h"

void slab_update(Slab* slab) {
    const float r = slab->config.wheel_diameter / 2;
    const float R = slab->config.wheel_distance / 2;
    const float v_max = slab->config.max_wheel_speed;
    const float v_lin = slab->input.linear_velocity;
    const float v_ang = slab->input.angular_velocity;
    float wheel_speeds[2] = {(v_lin - v_ang * R) / r, (v_lin + v_ang * R) / r};
    for (int i = 0; i < 2; ++i) {
        wheel_speeds[i] = fmin(v_max, fmax(-v_max, wheel_speeds[i]));
        slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL + i].input.velocity = wheel_speeds[i];
        slab->motors[MOTOR_ID_BACK_LEFT_WHEEL + i].input.velocity = wheel_speeds[i];
        slab->motors[i].input.position = slab->input.leg_positions[i];
    }
}

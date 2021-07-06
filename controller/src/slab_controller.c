#include "slab_controller.h"

void slab_update(Slab* slab) {
    // SlabContext_to_json(NULL, NULL);

    const float r = slab->config.wheel_diameter / 2;
    const float R = slab->config.wheel_distance / 2;
    const float v_max = slab->config.max_wheel_speed;
    float v_lin = slab->input.linear_velocity;
    float v_ang = slab->input.angular_velocity;
    float left_speed = (v_lin - v_ang * R) / r;
    float right_speed = (v_lin + v_ang * R) / r;
    left_speed = fmin(v_max, fmax(-v_max, left_speed));
    right_speed = fmin(v_max, fmax(-v_max, right_speed));
    printf("%f %f \n", left_speed, right_speed);
    slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL].input.velocity = left_speed;
    slab->motors[MOTOR_ID_FRONT_RIGHT_WHEEL].input.velocity = right_speed;
    slab->motors[MOTOR_ID_BACK_LEFT_WHEEL].input.velocity = left_speed;
    slab->motors[MOTOR_ID_BACK_RIGHT_WHEEL].input.velocity = right_speed;

    if (slab->tick > 100) {
        slab->motors[MOTOR_ID_FRONT_LEGS].input.position = -M_PI;
        slab->motors[MOTOR_ID_BACK_LEGS].input.position = -M_PI;
    } else {
        slab->motors[MOTOR_ID_FRONT_LEGS].input.position = -M_PI;
        slab->motors[MOTOR_ID_BACK_LEGS].input.position = -M_PI;
    }

    static char text_buf[1024];
    Slab_to_json(slab, text_buf);
    // puts(text_buf);
    // puts("");
}

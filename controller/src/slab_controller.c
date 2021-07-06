#include "slab_controller.h"

void slab_update(Slab* slab) {
    // SlabContext_to_json(NULL, NULL);

    slab->motors[MOTOR_ID_FRONT_LEFT_WHEEL].input.velocity = 1;
    slab->motors[MOTOR_ID_FRONT_RIGHT_WHEEL].input.velocity = 1;
    slab->motors[MOTOR_ID_BACK_LEFT_WHEEL].input.velocity = 1;
    slab->motors[MOTOR_ID_BACK_RIGHT_WHEEL].input.velocity = 1;

    if (slab->tick > 100) {
        slab->motors[MOTOR_ID_FRONT_LEGS].input.position = 0;
        slab->motors[MOTOR_ID_BACK_LEGS].input.position = -M_PI;
    } else {
        slab->motors[MOTOR_ID_FRONT_LEGS].input.position = -M_PI;
        slab->motors[MOTOR_ID_BACK_LEGS].input.position = -M_PI;
    }

    static char text_buf[1024];
    Slab_to_json(slab, text_buf);
    puts(text_buf);
    puts("");
}

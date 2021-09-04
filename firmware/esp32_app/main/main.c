#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "esp_bt_device.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/twai.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "odrive_can.h"
#include "ps3.h"
#include "slab.h"

//------------------------------------------------------------------------------
// constants

#define PIN_LED 33

#if 1
#define PIN_SDA 13
#define PIN_CLK 16

#define PIN_CAN_TX 12
#define PIN_CAN_RX 4
#else
#define PIN_SDA 22
#define PIN_CLK 23

#define PIN_CAN_TX 32
#define PIN_CAN_RX 25
#endif

#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_1MBITS

#define LED_HEARTBEAT_DOT_TICKS 20
#define MOTOR_HOMING_TIMEOUT_TICKS 50
#define MOTOR_COMMS_TIMEOUT_TICKS 50
#define IMU_COMMS_TIMEOUT_TICKS 50
#define GAMEPAD_COMMS_TIMEOUT_TICKS 50

#define N_MOTORS 6
static const float MOTOR_GEAR_RATIOS[N_MOTORS] = {-26, 26, -1, 1, -1, 1};

static const SlabConfig SLAB_CONFIG = {
        .max_wheel_speed = 50.0f,          // rads/s
        .wheel_diameter = 0.165f,          // m
        .wheel_distance = 0.425f,          // m
        .body_length = 0.380f,             // m
        .leg_length = 0.294f,              // m
        .max_leg_position = M_PI * 3 / 4,  // rad
        .min_leg_position = -M_PI,         // rad
        .imu_axis_remap = {AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z},
        .joystick_threshold = 10,       // 0 - 255
        .ground_rise_threshold = 0.08,  // m
        .ground_fall_threshold = 0.04,  // m
        .incline_p_gain = 40.0f,
        //.speed_p_gain = 0.6f,
        //.speed_i_gain = 0.003f,
};

static const wifi_ap_config_t WIFI_AP_CONFIG = {
        .ssid = "slab_robot",
        .ssid_len = 10,
        .channel = 1,
        .password = "",
        .max_connection = 3,
        .authmode = WIFI_AUTH_OPEN,
};

//------------------------------------------------------------------------------
// typedefs

typedef struct {
    bool homing_required_front : 1;
    bool homing_required_back : 1;
    bool estop : 1;
    bool can_error : 1;
    bool motor_error : 1;
    bool motor_comms_timeout : 1;
    bool imu_comms_timeout : 1;
    bool gamepad_comms_timeout : 1;
} AppErrorFlags;

#define TYPEDEF_AppError(X, _) \
    X(uint32_t, code, )        \
    _(AppErrorFlags, flags, )
MXGEN(union, AppError)

#define TYPEDEF_ErrorCounter(X, _) \
    _(int32_t, running, )          \
    X(int32_t, total, )
MXGEN(struct, ErrorCounter)

// keep the same layout as twai_status_info_t
// rx_overrun_count exist only in espidf > 4.3
#define TYPEDEF_CanStatus(X, _)      \
    X(uint32_t, state, )             \
    X(uint32_t, msgs_to_tx, )        \
    X(uint32_t, msgs_to_rx, )        \
    X(uint32_t, tx_error_counter, )  \
    X(uint32_t, rx_error_counter, )  \
    X(uint32_t, tx_failed_counter, ) \
    X(uint32_t, rx_missed_count, )   \
    X(uint32_t, rx_overrun_count, )  \
    X(uint32_t, arb_loss_count, )    \
    X(uint32_t, bus_error_count, )
MXGEN(struct, CanStatus)

// keep the same layout as TaskStatus_t
#define TYPEDEF_TaskStatus(X, _)        \
    _(void*, xHandle, )                 \
    _(char*, pcTaskName, )              \
    X(uint32_t, xTaskNumber, )          \
    X(uint32_t, eCurrentState, )        \
    X(uint32_t, uxCurrentPriority, )    \
    _(uint32_t, uxBasePriority, )       \
    _(uint32_t, ulRunTimeCounter, )     \
    _(void*, pxStackBase, )             \
    X(uint32_t, usStackHighWaterMark, ) \
    X(uint32_t, xCoreID, )
MXGEN(struct, TaskStatus)

// keep the same layout as ODriveAxis
#define TYPEDEF_ODriveStatus(X, _)     \
    X(uint64_t, motor_error, )         \
    X(uint32_t, encoder_error, )       \
    _(uint32_t, sensorless_error, )    \
    X(uint32_t, axis_error, )          \
    X(uint32_t, axis_state, )          \
    _(int32_t, encoder_shadow_count, ) \
    _(int32_t, encoder_count_cpr, )    \
    X(float, encoder_position, )       \
    X(float, encoder_velocity, )
MXGEN(struct, ODriveStatus)

#define TYPEDEF_MotorStatus(X, _) \
    X(ODriveStatus, status, )     \
    _(ODriveAxis, odrive, )
MXGEN(union, MotorStatus)

#define TYPEDEF_AppStatus(X, _)                     \
    X(uint32_t, tick, )                             \
    X(CanStatus, can, )                             \
    _(TaskStatus, tasks, [8])                       \
    X(MotorStatus, motors, [N_MOTORS])              \
    X(ErrorCounter, motor_comms_missed, [N_MOTORS]) \
    X(ErrorCounter, imu_comms_missed, )             \
    X(uint32_t, gamepad_timestamp, )                \
    X(AppError, error, )
MXGEN(struct, AppStatus)

typedef struct Imu Imu;

#define TYPEDEF_App(X, _)               \
    _(uint64_t, led_pattern, )          \
    _(int32_t, dir_idx, )               \
    _(TaskHandle_t, control_task, )     \
    _(TaskHandle_t, monitor_task, )     \
    _(TimerHandle_t, heartbeat_timer, ) \
    _(TimerHandle_t, homing_timer, )    \
    _(Imu*, imu, )                      \
    X(Slab, slab, )                     \
    X(AppStatus, status, )
MXGEN(struct, App)

//------------------------------------------------------------------------------
// helper functions

#define RTOS_ERROR_CHECK(ERR) ESP_ERROR_CHECK((ERR) -pdPASS)

#define CAN_ERROR_CHECK(APP, ERR) \
    (app)->status.error.flags.can_error |= ESP_ERROR_CHECK_WITHOUT_ABORT(ERR)

static int error_counter_update(ErrorCounter* counter, int error_count) {
    counter->total += error_count;
    return counter->running = error_count ? counter->running + error_count : 0;
}

static FRESULT f_write_line(FIL* file, char* text_buf, int len) {
    text_buf[len] = '\n';
    text_buf[++len] = '\0';
    uint32_t bytes_written = 0;
    return f_write(file, text_buf, len, &bytes_written);
}

static FIL* file_create(const char* file_name) {
    FIL* file = malloc(sizeof(FIL));
    assert(file);
    assert(file_name);
    FRESULT fresult = f_open(file, file_name, FA_WRITE | FA_CREATE_NEW);
    if (fresult == FR_OK) {
        ESP_LOGI(pcTaskGetName(NULL), "f_open %s success", file_name);
        return file;
    } else {
        free(file);
        ESP_LOGW(pcTaskGetName(NULL), "failed to f_open %s error %d", file_name, fresult);
        return NULL;
    }
}

static int dir_create(const char* dir_name) {
    FRESULT fresult;
    char text_buf[16];
    int dir_idx = 0;
    do {
        sprintf(text_buf, "%s%d", dir_name, dir_idx);
        fresult = f_mkdir(text_buf);
    } while (FR_EXIST == fresult && ++dir_idx);
    if (fresult != FR_OK) {
        ESP_LOGW(pcTaskGetName(NULL), "f_mkdir sdcard/%s error %d", text_buf, fresult);
        return -1;
    }
    ESP_LOGI(pcTaskGetName(NULL), "f_mkdir sdcard/%s success", text_buf);
    return dir_idx;
}

//------------------------------------------------------------------------------
// input and feedback functions

extern Imu* imu_create();
extern bool imu_read(Imu* imu, ImuMsg* imu_msg);

static void ps3_gamepad_led_update(AppError app_error) {
    ps3_cmd_t cmd = {0};
    cmd.led1 = true;
    cmd.led2 = app_error.flags.homing_required_front || app_error.flags.homing_required_back;
    cmd.led3 = app_error.flags.estop;
    cmd.led4 = app_error.code > 7;
    if (app_error.code > 3) {
        cmd.rumble_left_intensity = 255;
        cmd.rumble_right_intensity = 255;
        cmd.rumble_left_duration = 30;
        cmd.rumble_right_duration = 30;
    }
    ps3Cmd(cmd);
}

static void motors_homing_request(App* app) {
    for (int i = 0; i < 2; ++i) {
        // only send request if homing required and not already homing
        static const uint32_t state = ODRIVE_AXIS_STATE_HOMING;
        if (app->status.error.code & (1 << i) &&
                app->status.motors[i].status.axis_state != state) {
            CAN_ERROR_CHECK(app,
                    odrive_send_command(i, ODRIVE_CMD_SET_REQUESTED_STATE, &state, sizeof(state)));
        }
    }
    // start homing timer to detect timeout
    RTOS_ERROR_CHECK(xTimerStart(app->homing_timer, 0));
}

static void motors_homing_complete_callback(
        uint8_t axis_id, ODriveAxisState new_state, ODriveAxisState old_state, void* context) {
    assert(axis_id < 2);
    App* app = (App*) context;
    // reset homing required error when state transitions from HOMING to IDLE
    if (old_state == ODRIVE_AXIS_STATE_HOMING && new_state == ODRIVE_AXIS_STATE_IDLE) {
        app->status.error.code &= ~(1 << axis_id);
        app->slab.input.leg_positions[axis_id] = -M_PI;
        app->slab.motors[axis_id].input = (MotorInput){-M_PI, 0, 0, MOTOR_CONTROL_MODE_POSITION};
        app->slab.motors[axis_id].estimate = (MotorEstimate){-M_PI, 0};
        ps3_gamepad_led_update(app->status.error);
    }
}

static void motors_homing_timeout_callback(TimerHandle_t timer) {
    App* app = (App*) pvTimerGetTimerID(timer);
    for (int i = 0; i < 2; ++i) {
        // assume that homing is complete if there are no errors after a timeout
        // this is required because homing complete callback won't trigger if motors start homed
        const ODriveStatus* odrive = &app->status.motors[i].status;
        if (app->status.error.code & (1 << i) && odrive->axis_state == ODRIVE_AXIS_STATE_IDLE &&
                !(odrive->axis_error & ~ODRIVE_AXIS_ERROR_MIN_ENDSTOP_PRESSED) &&
                !odrive->motor_error && !odrive->encoder_error) {
            motors_homing_complete_callback(
                    i, ODRIVE_AXIS_STATE_IDLE, ODRIVE_AXIS_STATE_HOMING, app);
        }
    }
}

static void motors_input_update(App* app) {
    for (int i = 0; i < N_MOTORS; ++i) {
        const ODriveStatus* odrive = &app->status.motors[i].status;
        const MotorMsg* motor = &app->slab.motors[i];
        // clear endstop errors if the command is in the centering direction
        if ((odrive->axis_error == ODRIVE_AXIS_ERROR_MIN_ENDSTOP_PRESSED &&
                    motor->input.position >= motor->estimate.position &&
                    motor->input.velocity >= 0 && motor->input.torque >= 0) ||
                (odrive->axis_error == ODRIVE_AXIS_ERROR_MAX_ENDSTOP_PRESSED &&
                        motor->input.position <= motor->estimate.position &&
                        motor->input.velocity <= 0 && motor->input.torque <= 0)) {
            CAN_ERROR_CHECK(app, odrive_send_command(i, ODRIVE_CMD_CLEAR_ERRORS, NULL, 0));
        }
        // request closed loop control state if not already
        static const uint32_t state = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
        if (odrive->axis_state != state) {
            CAN_ERROR_CHECK(app,
                    odrive_send_command(i, ODRIVE_CMD_SET_REQUESTED_STATE, &state, sizeof(state)));
        }
        // send different motor commands depending on desired control mode
        // assume that the odrive is preconfigured to the correct mode
        // scale by gear ratio then change radians to cycles
        switch (motor->input.control_mode) {
            case MOTOR_CONTROL_MODE_POSITION: {
                const ODriveInputPosition input_pos = {
                        (motor->input.position + M_PI) * MOTOR_GEAR_RATIOS[i] / (2 * M_PI),
                        1000 * motor->input.velocity * MOTOR_GEAR_RATIOS[i] / (2 * M_PI),
                        1000 * motor->input.torque / MOTOR_GEAR_RATIOS[i]};
                CAN_ERROR_CHECK(app, odrive_send_command(i, ODRIVE_CMD_SET_INPUT_POS, &input_pos,
                                             sizeof(input_pos)));
                break;
            }
            case MOTOR_CONTROL_MODE_VELOCITY: {
                const ODriveInputVelocity input_vel = {
                        motor->input.velocity * MOTOR_GEAR_RATIOS[i] / (2 * M_PI),
                        motor->input.torque / MOTOR_GEAR_RATIOS[i]};
                CAN_ERROR_CHECK(app, odrive_send_command(i, ODRIVE_CMD_SET_INPUT_VEL, &input_vel,
                                             sizeof(input_vel)));
                break;
            }
            default:
                assert(0 && "motor control mode not implemented");
        }
    }
}

static esp_err_t motors_feedback_update(App* app) {
    ODriveAxis* odrives = &app->status.motors->odrive;
    // clear updates flags
    for (int i = 0; i < N_MOTORS; ++i) {
        odrives[i].updates = (ODriveUpdates){0};
    }
    // fetch updates
    esp_err_t rx_error = odrive_receive_updates(odrives, N_MOTORS);
    app->status.error.flags.motor_comms_timeout = false;
    for (int i = 0; i < N_MOTORS; ++i) {
        // check for motor comms timeout
        const uint8_t* updated = (uint8_t*) &odrives[i].updates;
        if (error_counter_update(&app->status.motor_comms_missed[i], *updated == 0) >
                MOTOR_COMMS_TIMEOUT_TICKS) {
            app->status.error.flags.motor_comms_timeout = true;
            // reset velocity and current estimates on timeout
            odrives[i].encoder_estimates.velocity = 0;
            odrives[i].sensorless_estimates.velocity = 0;
            odrives[i].iq.measured = 0;
        }
        // check for motor errors (excluding endstop reached)
        if ((odrives[i].updates.heartbeat &&
                    (odrives[i].heartbeat.axis_error & ~ODRIVE_AXIS_ERROR_MIN_ENDSTOP_PRESSED &
                            ~ODRIVE_AXIS_ERROR_MAX_ENDSTOP_PRESSED)) ||
                (odrives[i].updates.motor_error && odrives[i].motor_error) ||
                (odrives[i].updates.encoder_error && odrives[i].encoder_error)) {
            app->status.error.flags.motor_error = true;
        }
        // convert odrive interface to motor interface
        // change cycles to radians then scale by gear ratio
        if (odrives[i].updates.encoder_estimates) {
            app->slab.motors[i].estimate.position =
                    2 * M_PI * odrives[i].encoder_estimates.position / MOTOR_GEAR_RATIOS[i] - M_PI;
            app->slab.motors[i].estimate.velocity =
                    2 * M_PI * odrives[i].encoder_estimates.velocity / MOTOR_GEAR_RATIOS[i];
        }
    }
    return rx_error;
}

static esp_err_t motors_error_request(uint32_t tick) {
    // send a different error request command every tick
    uint8_t axis_id = (tick / 2) % N_MOTORS;
    uint8_t cmd_id = tick & 1 ? ODRIVE_CMD_GET_MOTOR_ERROR : ODRIVE_CMD_GET_ENCODER_ERROR;
    return odrive_send_get_command(axis_id, cmd_id);
}

static void motors_clear_errors(App* app) {
    app->status.error.flags.motor_error = false;
    app->status.error.flags.can_error = false;
    for (int i = 0; i < N_MOTORS; ++i) {
        CAN_ERROR_CHECK(app, odrive_send_command(i, ODRIVE_CMD_CLEAR_ERRORS, NULL, 0));
    }
}

static void motors_init(App* app) {
    motors_clear_errors(app);
    // wait for errors to clear
    vTaskDelay(1);
    // homing is required for legs only
    for (int i = 0; i < 2; ++i) {
        app->status.error.code |= 1 << i;
        app->status.motors[i].odrive.state_transition_callback = motors_homing_complete_callback;
        app->status.motors[i].odrive.state_transition_context = app;
    }
    // create motor homing timer
    app->homing_timer = xTimerCreate("homing_timer", MOTOR_HOMING_TIMEOUT_TICKS, false, app,
            motors_homing_timeout_callback);
    assert(app->homing_timer);
}

//------------------------------------------------------------------------------
// task functions

static void led_heartbeat_callback(TimerHandle_t timer) {
    App* app = (App*) pvTimerGetTimerID(timer);
    if (!app->led_pattern) {
        int idx = 4;
        for (int i = 0; i < 8; ++i) {
            if (app->status.error.code & (1 << i)) {
                app->led_pattern |= 0x7ULL << idx;
                idx += 4;
            } else {
                app->led_pattern |= 1ULL << idx;
                idx += 2;
            }
        }
    }
    ESP_ERROR_CHECK(gpio_set_level(PIN_LED, !(app->led_pattern & 1)));
    app->led_pattern >>= 1;
}

static void gamepad_callback(void* pvParameters, ps3_t ps3_state, ps3_event_t ps3_event) {
    App* app = (App*) pvParameters;
    // parse gamepad message
    GamepadMsg* gamepad = &app->slab.gamepad;
    memcpy(&gamepad->buttons, &ps3_state.button, 2);
    gamepad->left_stick.x = ps3_state.analog.stick.lx;
    gamepad->left_stick.y = ps3_state.analog.stick.ly;
    gamepad->right_stick.x = ps3_state.analog.stick.rx;
    gamepad->right_stick.y = ps3_state.analog.stick.ry;
    gamepad->left_trigger = ps3_state.analog.button.l2;
    gamepad->right_trigger = ps3_state.analog.button.r2;
    // update timestamp
    app->status.gamepad_timestamp = xTaskGetTickCount();
    // press PS home button to send homing request
    if (ps3_state.button.ps) {
        motors_homing_request(app);
    }
    // flip balance trigger
    gamepad->buttons ^= GAMEPAD_BUTTON_L1;
}

static void control_loop(void* pvParameters) {
    App* app = (App*) pvParameters;
    // match IMU DMP output rate of 100Hz
    // ODrive encoder updates also configured to 100Hz
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
        uint32_t prev_error_code = app->status.error.code;
        // fetch motor updates
        CAN_ERROR_CHECK(app, motors_error_request(tick));
        CAN_ERROR_CHECK(app, motors_feedback_update(app));
        // fetch imu updates
        app->status.error.flags.imu_comms_timeout =
                error_counter_update(&app->status.imu_comms_missed,
                        !imu_read(app->imu, &app->slab.imu)) > IMU_COMMS_TIMEOUT_TICKS;
        // reset accelerometer and gyro readings on timeout
        if (app->status.error.flags.imu_comms_timeout) {
            app->slab.imu.linear_acceleration = (Vector3F){0};
            app->slab.imu.angular_velocity = (Vector3F){0};
        }
        // detect gamepad communication timeout
        app->status.error.flags.gamepad_comms_timeout =
                tick > app->status.gamepad_timestamp + GAMEPAD_COMMS_TIMEOUT_TICKS;
        // reset gamepad input on timeout
        if (app->status.error.flags.gamepad_comms_timeout) {
            app->slab.gamepad = (GamepadMsg){0};
        }
        // else if gamepad is connected
        else {
            // press select button to trigger estop
            if (app->slab.gamepad.buttons & GAMEPAD_BUTTON_SELECT) {
                app->status.error.flags.estop = true;
            }
            // press start button to clear errors
            if (app->slab.gamepad.buttons & GAMEPAD_BUTTON_START) {
                app->status.error.flags.estop = false;
                motors_clear_errors(app);
            }
            // set controller LED when error state changes
            if (app->status.error.code != prev_error_code) {
                ps3_gamepad_led_update(app->status.error);
            }
        }
        // request idle state if there is an error
        if (app->status.error.code) {
            for (int i = 0; i < N_MOTORS; ++i) {
                if (app->status.motors[i].status.axis_state ==
                        ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
                    static const uint32_t state = ODRIVE_AXIS_STATE_IDLE;
                    CAN_ERROR_CHECK(app, odrive_send_command(i, ODRIVE_CMD_SET_REQUESTED_STATE,
                                                 &state, sizeof(state)));
                }
            }
        }
        // run control logic only when error-free
        else {
            app->slab.tick = tick;
            slab_update(&app->slab);
            motors_input_update(app);
        }
    }
}

static void monitor_loop(void* pvParameters) {
    App* app = (App*) pvParameters;
    static char text_buf[4096];
    // check memory layout alignment of reinterpreted structs
    TRY_STATIC_ASSERT(sizeof(CanStatus) == sizeof(twai_status_info_t), "memory layout mismatch");
    TRY_STATIC_ASSERT(sizeof(TaskStatus) == sizeof(TaskStatus_t), "memory layout mismatch");
    TRY_STATIC_ASSERT(sizeof(MotorStatus) == sizeof(ODriveAxis), "memory layout mismatch");
    // create new csv log file on sdcard if directory exists
    FIL* csv_log = NULL;
    if (app->dir_idx > -1) {
        sprintf(text_buf, "slab%d/%s", app->dir_idx, "data.csv");
        // write csv header row if file was successfully created
        if ((csv_log = file_create(text_buf))) {
            int len = App_to_csv_header(0, text_buf);
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_write_line(csv_log, text_buf, len));
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_sync(csv_log));
        }
    }
    // monitor loops at a lower frequency
    for (app->status.tick = xTaskGetTickCount();; vTaskDelayUntil(&app->status.tick, 5)) {
#if 0
        // print task status
        puts("Task Name       State   Pri     Stack   Num     CoreId");
        vTaskList(text_buf);
        puts(text_buf);

        puts("Task Name       Abs Time        % Time");
        vTaskGetRunTimeStats(text_buf);
        puts(text_buf);

        // update task status
        uxTaskGetSystemState((TaskStatus_t*) app->status.tasks,
                sizeof(app->status.tasks) / sizeof(app->status.tasks[0]), NULL);
#endif
        // update can status
        ESP_ERROR_CHECK(twai_get_status_info((twai_status_info_t*) &app->status.can));
        // print app data to uart
        App_to_json(app, text_buf);
        puts(text_buf);
        // log app status as csv entry
        if (csv_log) {
            int len = App_to_csv_entry(app, text_buf);
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_write_line(csv_log, text_buf, len));
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_sync(csv_log));
        }
    }
}

//------------------------------------------------------------------------------
// init functions

static void can_init() {
    twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.intr_flags |= ESP_INTR_FLAG_IRAM;
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;
    twai_timing_config_t t_config = CAN_BAUD_RATE();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

static void i2c_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t) PIN_SDA;
    conf.scl_io_num = (gpio_num_t) PIN_CLK;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

static void led_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ull << PIN_LED;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {.ap = WIFI_AP_CONFIG};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(pcTaskGetName(NULL), "WIFI SSID:%s", (char*) &wifi_config.ap.ssid);
}

static void ps3_init() {
    ps3Init();
    const uint8_t* mac = esp_bt_dev_get_address();
    assert(mac);
    ESP_LOGI(pcTaskGetName(NULL), "Bluetooth MAC %hhx:%hhx:%hhx:%hhx:%hhx:%hhx", mac[0], mac[1],
            mac[2], mac[3], mac[4], mac[5]);
}

static void sdcard_init() {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    slot_config.width = 1;  // configure 1-line mode
    sdmmc_card_t* card;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024};
    ESP_ERROR_CHECK_WITHOUT_ABORT(
            esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card));
}

void app_main(void) {
    // init peripherals
    can_init();
    i2c_init();
    led_init();
    wifi_init();
    ps3_init();
    sdcard_init();
    // init app
    static App app = {0};
    app.slab.config = SLAB_CONFIG;
    app.imu = imu_create();
    motors_init(&app);
    ps3SetEventObjectCallback(&app, gamepad_callback);
    esp_log_level_set("PS3_L2CAP", ESP_LOG_WARN);
    app.dir_idx = dir_create("slab");
    // init tasks
    RTOS_ERROR_CHECK(xTaskCreatePinnedToCore(
            monitor_loop, "monitor_loop", 8 * 2048, &app, 1, &app.monitor_task, 0));
    RTOS_ERROR_CHECK(xTaskCreatePinnedToCore(
            control_loop, "control_loop", 8 * 2048, &app, 9, &app.control_task, 1));
    app.heartbeat_timer = xTimerCreate(
            "heartbeat_timer", LED_HEARTBEAT_DOT_TICKS, true, &app, led_heartbeat_callback);
    assert(app.heartbeat_timer);
    RTOS_ERROR_CHECK(xTimerStart(app.heartbeat_timer, 0));
}

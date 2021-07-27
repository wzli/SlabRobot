#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"

#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/twai.h"
#include "sdkconfig.h"

#include "ps3.h"
#include "msg_defs.h"

#if 1
#define PIN_SDA 13
#define PIN_CLK 16
#define PIN_CTX 12
#define PIN_CRX 4
#else
#define PIN_SDA 22
#define PIN_CLK 23
#define PIN_CTX 32
#define PIN_CRX 25
#endif

#define MOTOR_COMMS_TIMEOUT_TICKS 50
#define IMU_COMMS_TIMEOUT_TICKS 50
#define GAMEPAD_COMMS_TIMEOUT_TICKS 50

typedef struct {
    bool homing_required : 1;
    bool motor_error : 1;
    bool motor_comms_timeout : 1;
    bool imu_comms_timeout : 1;
    bool gamepad_comms_timeout : 1;
} AppError;

// rx_overrun_count exist in espidf > 4.3
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

#define TYPEDEF_ErrorCounter(X, _) \
    X(int32_t, running, )          \
    X(int32_t, total, )
MXGEN(struct, ErrorCounter)

#define TYPEDEF_AppStatus(X, _)           \
    X(uint32_t, tick, )                   \
    _(TaskStatus, tasks, [8])             \
    X(CanStatus, can, )                   \
    X(ErrorCounter, motor_comms_missed, ) \
    X(ErrorCounter, imu_comms_missed, )   \
    X(uint32_t, gamepad_timestamp, )      \
    X(uint32_t, error, )
MXGEN(struct, AppStatus)

esp_err_t odrive_receive_updates(MotorMsg* motors, uint8_t len);

typedef struct Imu Imu;
Imu* imu_create();
bool imu_read(Imu* imu, ImuMsg* imu_msg);

typedef struct {
    Imu* imu;
    Slab slab;
    AppStatus status;
    TaskHandle_t control_task;
    TaskHandle_t monitor_task;
    int dir_idx;
} App;

static int update_error_counter(ErrorCounter* counter, int error_count) {
    if (error_count) {
        counter->running += error_count;
        counter->total += error_count;
    } else {
        counter->running = 0;
    }
    return counter->running;
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

static void can_init() {
    twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NORMAL);
    g_config.intr_flags |= ESP_INTR_FLAG_IRAM;
    g_config.rx_queue_len = 32;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
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

static void ps3_init() {
    static const uint8_t mac[6] = {0x54, 0x54, 0x54, 0x54, 0x54, 0x54};
    ESP_ERROR_CHECK(nvs_flash_init());
    ps3SetBluetoothMacAddress(mac);
    ps3Init();
    ESP_LOGI(pcTaskGetName(NULL), "Bluetooth MAC %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1],
            mac[2], mac[3], mac[4], mac[5]);
}

static void ps3_gamepad_callback(void* pvParameters, ps3_t ps3_state, ps3_event_t ps3_event) {
    App* app = (App*) pvParameters;
    GamepadMsg* gamepad = &app->slab.gamepad;
    memcpy(&gamepad->buttons, &ps3_state.button, 2);
    gamepad->left_stick.x = ps3_state.analog.stick.lx;
    gamepad->left_stick.y = ps3_state.analog.stick.ly;
    gamepad->right_stick.x = ps3_state.analog.stick.rx;
    gamepad->right_stick.y = ps3_state.analog.stick.ry;
    gamepad->left_trigger = ps3_state.analog.button.l2;
    gamepad->right_trigger = ps3_state.analog.button.r2;
    app->status.gamepad_timestamp = xTaskGetTickCount();
}

static void control_loop(void* pvParameters) {
    App* app = (App*) pvParameters;
    AppError* app_error = (AppError*) &app->status.error;
    static char text_buf[512];
    // Match IMU DMP output rate of 100Hz
    // ODrive encoder updates also configured to 100Hz
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
        // printf("t%d\n", tick);

        // fetch motor updates
        app_error->motor_comms_timeout =
                MOTOR_COMMS_TIMEOUT_TICKS <
                update_error_counter(&app->status.motor_comms_missed,
                        ESP_OK != odrive_receive_updates(app->slab.motors, 2));
        // fetch imu updates
        app_error->imu_comms_timeout =
                IMU_COMMS_TIMEOUT_TICKS < update_error_counter(&app->status.imu_comms_missed,
                                                  !imu_read(app->imu, &app->slab.imu));
        // detect gamepad timeout and zero input
        if ((app_error->gamepad_comms_timeout =
                            tick > GAMEPAD_COMMS_TIMEOUT_TICKS + app->status.gamepad_timestamp)) {
            app->slab.gamepad = (GamepadMsg){0};
        }
        // ImuMsg_to_json(&app->imu_msg, app->json);
        // Vector3F_to_json(&app->imu_msg.linear_acceleration, app->json);
        // puts(app->json);
        if (app->slab.gamepad.buttons) {
            GamepadMsg_to_json(&app->slab.gamepad, text_buf);
            puts(text_buf);
        }
        // disable control loop during error state
        if (app->status.error) {
            continue;
        }
        // control code below here
    }
}

static FRESULT f_write_line(FIL* file, char* text_buf, int len) {
    assert(file);
    assert(text_buf);
    text_buf[len] = '\n';
    text_buf[++len] = '\0';
    uint32_t bytes_written = 0;
    return f_write(file, text_buf, len, &bytes_written);
}

static void monitor_loop(void* pvParameters) {
    TRY_STATIC_ASSERT(sizeof(CanStatus) == sizeof(twai_status_info_t), "type mismatch");
    static char text_buf[2048];

    assert(pvParameters);
    App* app = (App*) pvParameters;
    const char* TAG = pcTaskGetName(NULL);
    FIL* csv_log = NULL;
#if 1
    // create new csv log
    if (app->dir_idx > -1) {
        csv_log = malloc(sizeof(FIL));
        assert(csv_log);
        sprintf(text_buf, "%d/%s", app->dir_idx, "test.csv");
        FRESULT fresult = f_open(csv_log, text_buf, FA_WRITE | FA_CREATE_NEW);
        if (fresult == FR_OK) {
            ESP_LOGI(TAG, "f_open %s success", text_buf);
            int len = AppStatus_to_csv_header(0, text_buf);
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_write_line(csv_log, text_buf, len));
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_sync(csv_log));
        } else {
            ESP_LOGW(TAG, "failed to f_open %s error %d", text_buf, fresult);
            free(csv_log);
            csv_log = NULL;
        }
    }
#endif

    for (app->status.tick = xTaskGetTickCount();; vTaskDelayUntil(&app->status.tick, 300)) {
#if 0
        puts("Task Name       State   Pri     Stack   Num     CoreId");
        vTaskList(text_buf);
        puts(text_buf);

        puts("Task Name       Abs Time        % Time");
        vTaskGetRunTimeStats(text_buf);
        puts(text_buf);
#endif
        ESP_ERROR_CHECK(twai_get_status_info((twai_status_info_t*) &app->status.can));
        uxTaskGetSystemState((TaskStatus_t*) app->status.tasks,
                sizeof(app->status.tasks) / sizeof(app->status.tasks[0]), NULL);

        AppStatus_to_json(&app->status, text_buf);
        puts(text_buf);
        puts("");

        if (csv_log) {
            int len = AppStatus_to_csv_entry(&app->status, text_buf);
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_write_line(csv_log, text_buf, len));
            ESP_ERROR_CHECK_WITHOUT_ABORT(f_sync(csv_log));
        }
    }
}

static int dir_create() {
    FRESULT fresult;
    char text_buf[11];
    int dir_idx = 0;
    do {
        sprintf(text_buf, "%d", dir_idx);
        fresult = f_mkdir(text_buf);
    } while (FR_EXIST == fresult && ++dir_idx);
    if (fresult != FR_OK) {
        ESP_LOGW(pcTaskGetName(NULL), "f_mkdir sdcard/%d error %d", dir_idx, fresult);
        return -1;
    }
    ESP_LOGI(pcTaskGetName(NULL), "f_mkdir sdcard/%d success", dir_idx);
    return dir_idx;
}

void app_main(void) {
    sdcard_init();
    can_init();
    i2c_init();
    ps3_init();

    static App app;
    app.dir_idx = dir_create();
    app.imu = imu_create();
    ps3SetEventObjectCallback(&app, ps3_gamepad_callback);

    xTaskCreatePinnedToCore(monitor_loop, "monitor_loop", 8 * 2048, &app, 1, &app.monitor_task, 0);
    xTaskCreatePinnedToCore(control_loop, "control_loop", 8 * 2048, &app, 9, &app.control_task, 1);
}

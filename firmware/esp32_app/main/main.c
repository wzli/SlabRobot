#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/twai.h"
#include "sdkconfig.h"

#include "msg_defs.h"

#define PIN_SDA 13
#define PIN_CLK 16
// #define PIN_SDA 13
// #define PIN_CLK 12

#define PIN_CTX 12
#define PIN_CRX 4

// #define PIN_CTX 32
// #define PIN_CRX 25

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

#define TYPEDEF_ErrorCounters(X, _) \
    X(uint32_t, imu_fetch, )        \
    X(uint32_t, odrive_fetch, )
MXGEN(struct, ErrorCounters)

#define TYPEDEF_AppStatus(X, _) \
    _(TaskStatus, tasks, [8])   \
    X(CanStatus, can, )         \
    X(ErrorCounters, error_counters, )
MXGEN(struct, AppStatus)

esp_err_t odrive_receive_updates(MotorMsg* motors, uint8_t len);

typedef struct Imu Imu;
Imu* imu_create();
bool imu_read(Imu* imu, ImuMsg* imu_msg);

typedef struct {
    Imu* imu;
    SlabContext ctx;
    AppStatus status;
    TaskHandle_t control_task;
    TaskHandle_t monitor_task;
} App;

static void can_init() {
    twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NORMAL);
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

static void control_loop(void* pvParameters) {
    App* app = (App*) pvParameters;
    // static char text_buf[512];
    // Match IMU DMP output rate of 100Hz
    // ODrive encoder updates also configured to 100Hz
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
        printf("t%d\n", tick);
        app->status.error_counters.imu_fetch += !imu_read(app->imu, &app->ctx.imu);
        app->status.error_counters.odrive_fetch += 1 & odrive_receive_updates(app->ctx.motors, 2);
        // ImuMsg_to_json(&app->imu_msg, app->json);
        // Vector3F_to_json(&app->imu_msg.linear_acceleration, app->json);
        // puts(app->json);
    }
}

static void monitor_loop(void* pvParameters) {
    TRY_STATIC_ASSERT(sizeof(CanStatus) == sizeof(twai_status_info_t), "type mismatch");
    App* app = (App*) pvParameters;
    static char text_buf[2048];
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1000)) {
#if 0
        puts("Task Name       State   Pri     Stack   Num     CoreId");
        vTaskList(text_buf);
        puts(text_buf);

        puts("Task Name       Abs Time        % Time");
        vTaskGetRunTimeStats(text_buf);
        puts(text_buf);
#endif

        ESP_ERROR_CHECK(twai_get_status_info((twai_status_info_t*) &app->status.can));
        uxTaskGetSystemState((TaskStatus_t*) app->status.tasks, 8, NULL);
        AppStatus_to_json(&app->status, text_buf);
        puts(text_buf);
        // asprintf(text_buf);
    }
}

void app_main(void) {
    static App app;
    can_init();
    i2c_init();
    app.imu = imu_create();
    xTaskCreatePinnedToCore(control_loop, "control_loop", 2048, &app, 9, &app.control_task, 1);
    xTaskCreatePinnedToCore(monitor_loop, "monitor_loop", 2048, &app, 1, &app.monitor_task, 0);
}

#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/twai.h"
#include "sdkconfig.h"

#include "odrive_can.h"
#include "msg_defs.h"

#define PIN_SDA 22
#define PIN_CLK 23

//#define PIN_CTX 12
//#define PIN_CRX 13
#define PIN_CTX 32
#define PIN_CRX 25

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

typedef struct Imu Imu;

typedef struct {
    Imu* imu;
    ImuMsg imu_msg;
    TaskHandle_t control_task;
    TaskHandle_t can_task;
    TaskHandle_t monitor_task;
} App;

extern Imu* imu_create();
extern bool imu_read(Imu* imu, ImuMsg* imu_msg);

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

static void can_init() {
    twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

static void control_loop(void* pvParameters) {
    App* app = (App*) pvParameters;
    static char json[256];
    // Match IMU DMP output rate of 100Hz
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
        imu_read(app->imu, &app->imu_msg);
        // ImuMsg_to_json(&app->imu_msg, app->json);
        // Vector3F_to_json(&app->imu_msg.linear_acceleration, app->json);
        // puts(app->json);
    }
}
static void can_loop(void* pvParameters) {
    twai_message_t can_msg = {};
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
        printf("tick %d\n", tick);
        while (twai_receive(&can_msg, 0) == ESP_OK) {
            printf("  ID %d sz %d\n", can_msg.identifier, can_msg.data_length_code);
        }
    }
}

static void monitor_loop(void* pvParameters) {
    assert(sizeof(CanStatus) == sizeof(twai_status_info_t));
    twai_status_info_t can_status;
    static char text_buf[1024];
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 100)) {
        puts("Task Name       State   Pri     Stack   Num     CoreId");
        vTaskList(text_buf);
        puts(text_buf);

        puts("Task Name       Abs Time        % Time");
        vTaskGetRunTimeStats(text_buf);
        puts(text_buf);

        ESP_ERROR_CHECK(twai_get_status_info(&can_status));
        CanStatus_to_json((CanStatus*) &can_status, text_buf);
        puts(text_buf);
    }
}

void app_main(void) {
    static App app;
    i2c_init();
    can_init();
    app.imu = imu_create();
    xTaskCreatePinnedToCore(control_loop, "control_loop", 2048, &app, 9, &app.control_task, 1);
    xTaskCreatePinnedToCore(can_loop, "can_loop", 2048, &app, 9, &app.can_task, 1);
    xTaskCreatePinnedToCore(monitor_loop, "monitor_loop", 2048, &app, 1, &app.monitor_task, 1);
}

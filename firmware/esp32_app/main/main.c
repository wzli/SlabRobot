#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "sdkconfig.h"

#include "msg_defs.h"

#define PIN_SDA 22
#define PIN_CLK 23

typedef struct Imu Imu;

typedef struct {
    Imu* imu;
    ImuMsg imu_msg;
    char json[256];
    TaskHandle_t control_task;
} App;

extern Imu* imu_create();
extern bool imu_read(Imu* imu, ImuMsg* imu_msg);

static void i2c_init() {
    i2c_config_t conf;
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
    TickType_t prev_wake_time = xTaskGetTickCount();
    while (1) {
        imu_read(app->imu, &app->imu_msg);
        // ImuMsg_to_json(&app->imu_msg, app->json);
        Vector3F_to_json(&app->imu_msg.linear_acceleration, app->json);
        puts(app->json);
        // Match IMU DMP output rate of 100Hz
        vTaskDelayUntil(&prev_wake_time, 1);
    }
}

void app_main(void) {
    static App app;
    i2c_init();
    app.imu = imu_create();
    xTaskCreatePinnedToCore(control_loop, "control_loop", 2048, &app, 9, &app.control_task, 1);
}

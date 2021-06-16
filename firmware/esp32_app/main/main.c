#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/twai.h"
#include "sdkconfig.h"

#include "msg_defs.h"

#define PIN_SDA 22
#define PIN_CLK 23

#define PIN_CTX GPIO_NUM_13
#define PIN_CRX GPIO_NUM_12

typedef struct Imu Imu;

typedef struct {
    Imu* imu;
    ImuMsg imu_msg;
    char json[256];
    TaskHandle_t control_task;
    TaskHandle_t can_task;
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
    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX,
    // TWAI_MODE_NORMAL);
    twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

static void control_loop(void* pvParameters) {
    App* app = (App*) pvParameters;
    // Match IMU DMP output rate of 100Hz
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
        imu_read(app->imu, &app->imu_msg);
        // ImuMsg_to_json(&app->imu_msg, app->json);
        Vector3F_to_json(&app->imu_msg.linear_acceleration, app->json);
        // puts(app->json);
    }
}
static void can_loop(void* pvParameters) {
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)) {
#if 0
        //Configure message to transmit
        uint8_t axis_id = 0;
        uint8_t cmd_id = 3;

        twai_message_t message;
        message.identifier = (axis_id << 5) | cmd_id;
        message.extd = 0;
        message.rtr = 1;
        message.data_length_code = 0;
        // for (int i = 0; i < 4; i++) {
        //    message.data[i] = 0;
        //}

        //Queue message for transmission
        if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
            printf("Message queued for transmission\n");
        } else {
            printf("Failed to queue message for transmission\n");
        }
#else
        twai_message_t tx_msg = {.data_length_code = 1, .identifier = 0x555, .self = 1};
        ESP_ERROR_CHECK(twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)));
#endif

        {
            // Wait for message to be received
            twai_message_t message;
            if (twai_receive(&message, pdMS_TO_TICKS(100000)) == ESP_OK) {
                printf("Message received\n");
            } else {
                printf("Failed to receive message\n");
                return;
            }

            // Process received message
            if (message.extd) {
                printf("Message is in Extended Format\n");
            } else {
                printf("Message is in Standard Format\n");
            }
            printf("ID is %d\n", message.identifier);
            if (!(message.rtr)) {
                for (int i = 0; i < message.data_length_code; i++) {
                    printf("Data byte %d = %d\n", i, message.data[i]);
                }
            }
        }
    }
}

void app_main(void) {
    static App app;
    i2c_init();
    can_init();
    app.imu = imu_create();
    xTaskCreatePinnedToCore(control_loop, "control_loop", 2048, &app, 9, &app.control_task, 1);
    xTaskCreatePinnedToCore(can_loop, "can_loop", 2048, &app, 1, &app.can_task, 1);

    char text_buf[1024];
    puts("Task Name       State   Pri     Stack   Num     CoreId");
    vTaskList(text_buf);
    puts(text_buf);
    puts("Task Name       Abs Time        % Time");
    vTaskGetRunTimeStats(text_buf);
    puts(text_buf);
}

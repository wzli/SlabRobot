#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "sdkconfig.h"

#include "MPU6050.h"

#include "msg_defs.h"

#define PIN_SDA 22
#define PIN_CLK 23

Quaternion q;              // [w, x, y, z]         quaternion container
VectorFloat gravity;       // [x, y, z]            gravity vector
float ypr[3];              // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t fifoCount;        // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];    // FIFO storage buffer
uint8_t mpuIntStatus;      // holds actual interrupt status byte from MPU

void task_initI2C() {
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

void task_display(void*) {
    MPU6050 mpu = MPU6050();
    mpu.initialize();
    mpu.dmpInitialize();
    //mpu.setXAccelOffset(1150);
    //mpu.setYAccelOffset(-50);
    //mpu.setZAccelOffset(1060);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    TickType_t prev_wake_time = xTaskGetTickCount();

    char json[256];
    while(1) {
        ImuMsg imu_msg;
        for(int i = mpu.getFIFOCount() / mpu.dmpPacketSize; i > 0; --i) {
            mpu.getFIFOBytes((uint8_t*)&imu_msg, sizeof(imu_msg));
        }
        // TODO: handle when fifo is empty
        mpu.dmpProcessFIFOPacket((uint8_t*)&imu_msg);
        ImuMsg_to_json(&imu_msg, json);
        puts(json);

        // Best result is to match with DMP refresh rate 100Hz
        vTaskDelayUntil(&prev_wake_time, 1);
    }

}

extern "C" {

void app_main(void) {
    task_initI2C();
    task_display(0);
}

}

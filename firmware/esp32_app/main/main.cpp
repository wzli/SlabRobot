/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "sdkconfig.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>
#include "MPU6050.h"
static inline bool writeCustomDmpConfig(MPU6050* ctx, const uint8_t* config, uint16_t len) {
    uint8_t custom_config[len];
    memcpy(custom_config, config, len);
    // Last byte determines DMP FIFO output frequency calculated as (200Hz / (1 + value)).
    // Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    custom_config[len - 1] = 1; // 100 Hz
    return ctx->writeProgDMPConfigurationSet(custom_config, len);
}
#define writeProgDMPConfigurationSet(config, len) writeCustomDmpConfig(this, config, len)
#include "MPU6050_6Axis_MotionApps20.h"



#define PIN_SDA 22
#define PIN_CLK 23

Quaternion q;              // [w, x, y, z]         quaternion container
VectorFloat gravity;       // [x, y, z]            gravity vector
float ypr[3];              // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;  // expected DMP packet size (default is 42 bytes)
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
    //uint8_t* mod = (uint8_t*)dmpConfig + MPU6050_DMP_CONFIG_SIZE - 1;
    //*mod = 1;
    mpu.dmpInitialize();

    // This need to be setup individually
    // mpu.setXGyroOffset(0);
    // mpu.setYGyroOffset(0);
    // mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1500);

    mpu.setDMPEnabled(true);

    while (1) {
        mpuIntStatus = mpu.getIntStatus();
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();

            // otherwise, check for DMP data ready interrupt frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize)
                fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //printf("YAW: %3.1f, ", ypr[0] * 180 / M_PI);
            //printf("PITCH: %3.1f, ", ypr[1] * 180 / M_PI);
            //printf("ROLL: %3.1f \n", ypr[2] * 180 / M_PI);
            //printf("%f, %f, %f\n", ypr[0] * 180 / M_PI, ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);
            //printf("%f, %f, %f\n", gravity.x, gravity.y, gravity.z);

            int16_t accel[3];
            mpu.dmpGetAccel(accel, fifoBuffer);
            printf("%d, %d, %d\n", accel[0], accel[1], accel[2]);
        }

        // Best result is to match with DMP refresh rate
        // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
        // Now its 0x13, which means DMP is refreshed with 10Hz rate
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

extern "C" {

void app_main(void) {
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET, chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    
    task_initI2C();
    task_display(0);
}
}

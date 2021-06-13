#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "sdkconfig.h"

#include "MPU6050.h"

#include "msg_defs.h"


static inline void swap_uint8(uint8_t* a, uint8_t* b) {
    *a ^= *b;
    *b ^= *a;
    *a ^= *b;
}

void dmp_packet_reverse_endian(uint8_t* packet) {
    for(int i = 0; i < 4; ++i) {
        swap_uint8(packet, packet + 3);
        swap_uint8(packet + 1, packet + 2);
        packet += 4;
    }
    for(int i = 0; i < 6; ++i) {
        swap_uint8(packet, packet + 1);
        packet += 2;
    }
}

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
    //uint8_t* mod = (uint8_t*)dmpConfig + MPU6050_DMP_CONFIG_SIZE - 1;
    //*mod = 1;
    assert(!mpu.dmpInitialize());

    // This need to be setup individually
    // mpu.setXGyroOffset(0);
    // mpu.setYGyroOffset(0);
    // mpu.setZGyroOffset(0);
    // mpu.setZAccelOffset(1500);

/*
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);
    */

    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    //while(1) {
    //    mpu.GetCurrentFIFOPacket(fifoBuffer, packetSize)
    //}

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
            while (fifoCount < mpu.dmpPacketSize)
                fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO

            mpu.getFIFOBytes(fifoBuffer, mpu.dmpPacketSize);
            dmp_packet_reverse_endian(fifoBuffer);
            char json[256];
            ImuMsg_to_json((const ImuMsg*)fifoBuffer, json);
            puts(json);
        }

        // Best result is to match with DMP refresh rate
        // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
        // Now its 0x13, which means DMP is refreshed with 10Hz rate
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

extern "C" {

void app_main(void) {
    task_initI2C();
    task_display(0);
}

}

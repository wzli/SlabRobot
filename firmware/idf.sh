#/bin/bash
version=release-v4.2
project=esp32_app
docker run -it --rm --privileged -w /project -v $PWD/$project:/project \
-v $PWD/../submodules/i2cdevlib/ESP32_ESP-IDF/components/I2Cdev:/project/components/I2Cdev \
-v $PWD/../submodules/i2cdevlib/ESP32_ESP-IDF/components/MPU6050:/project/components/MPU6050 \
espressif/idf:$version idf.py $@


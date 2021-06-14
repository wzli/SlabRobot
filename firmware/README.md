# Firmware Setup

## Hardware
### ESP32-CAM
- ***First check if VDD is connected to 5V or 3.3V***
- remove resistors to flash LED, its the one right below the voltage selectors

- can module already comes with 120ohm

# ESP32
Follow the getting started guide in
[Official ESP-IDF Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/) to setup firmware development enviorment. Alternatively
[ESP-IDF Docker](https://hub.docker.com/r/espressif/idf) provides everything preconfigured.

```bash
docker pull espressif/idf:release-v4.3
```

## Configure
`idf.sh menuconfig`

### Options
- Compiler Options / Optimization Level
  - Optimize for performance (-O2)
- Component Config / Driver Configurations / TWAI configuration
  - Place TWAI ISR function into IRAM
- Component Config / ESP32-Specific / CPU frequency
  - 240 MHz
- Component Config / ESP32-Specific / Support for external, SPI-connected RAM
  - True (ESP32-CAM has 4MB PSRAM)


## Build
`idf.sh build`

## Flash
- connect usb serial to UART pins
- ground IO0 pin
- `idf.sh -p PORT [-b BAUD] flash`

## Serial Monitor
`idf.py monitor`

# MPU6050
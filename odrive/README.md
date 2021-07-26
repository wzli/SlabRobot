# ODrive Setup Guide

Tips on setting up Odrive (v3.6) on FW v5.2 complementary to the [Official Docs](https://docs.odriverobotics.com/).

# USB connection
See [Downloading and Installing Tools](https://docs.odriverobotics.com/#downloading-and-installing-tools).

- For linux, need to setup udev rules.
- For windows, need to use [zadig](https://zadig.akeo.ie) to setup *libusb-win32* driver.

# Firmware
Should first update to latest firmware by running `odrivetool dfu`.
See [docs](https://docs.odriverobotics.com/#firmware) for details.
If that runs into issues though, don't bother troubleshooting *dfu* it's way more simple and reliable to just build firmware and flash through STlink.

## Firmware Development
Refer to [docs](https://docs.odriverobotics.com/developer-guide), in summary:

- Install dependencies `gcc-arm-embedded, openocd, tup, etc`.
- `git clone https://github.com/odriverobotics/ODrive` and pull the latest release.
- Edit `Firmware/tup.config` and call `make` in Firmware directory.
- Connect STlink pins `GND, SWD, and SWC` and call `make flash`.
- DIP switch needs to be flipped to RUN instead of DFU inorder to flash.
- For clone boards, the call to `check_board_version()` in `Firmware/Board/v3/board.cpp` has to be disabed, otherwise it will intentionally spinlock during init from OTP mismatch.

# CAN Bus Configuration
See [CAN Protocol Docs](https://docs.odriverobotics.com/can-protocol).

- There is a built-in DIP switch that toggles the 120Ω termination resistor.
- Each axis is an independent endpoint, with their unique node ID configured by `odrv0.axis0.config.can.node_id = [0 - 0x3F]`.
- Set baudrate to 1Mbps `odrv0.can.config.baud_rate = 1000000`.
- Set encoder rate (for both axis) `odrv0.axis0.config.can.encoder_rate_ms = 10`.
- Set heartbeat rate (for both axis) `odrv0.axis0.config.can.heartbeat_rate_ms = 100`.
- Save and reboot `odrv0.save_configuration()`.

# End Stops
Documentation is provided at [Endstops guide](https://docs.odriverobotics.com/endstops). Refer to [ODrive pinout](https://docs.odriverobotics.com/pinout) to select which GPIOs to use.

```
// Open drain example setup
odrv0.config.gpio1_mode = GPIO_MODE_DIGITAL_PULL_UP
odrv0.axis0.min_endstop.config.enabled = True
odrv0.axis0.min_endstop.config.gpio_num = 1
odrv0.axis0.min_endstop.config.is_active_high = False
odrv0.save_configuration()

// homing example
odrv0.axis0.controller.config.homing_speed = 5 // -ve to change direction
odrv0.axis0.requested_state = AXIS_STATE_HOMING
```

# Setup
First follow [wiring guide](https://docs.odriverobotics.com/#wiring-up-the-odrive) to hook things up.
Then refer to [params guide](https://docs.odriverobotics.com/#configure-m0) to configure each axis.

- **Double check wiring things might burn if hooked up incorrectly!** 
- **Always recalibrate motor and encoders after things have been plugged and unplugged!**

## A. System Configuration
1. Set max regen current according to battery `odrv0.config.max_regen_current = 5`.
1. `odrv0.config.dc_max_negative_current = -10`.
2. Enable brake resistor `odrv0.config.enable_brake_resistor = True`.
3. Set `odrv0.config.brake_resistance = 2` (value of power resistor that comes with kit).

## B. Calibrate Motor 

1. Set `odrv0.axis0.motor.config.pole_pairs = X`.
Refer to params guide to determing number of pole pairs for the motor.
The hover board motors have 15 pole pairs, and BDUAV6384 motors has 7 pole pairs.
2. Set `odrv0.axis0.motor.config.torque_constant = 8.27/(motor KV)`
3. Call `odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION` to perform motor calibration.
Check `odrv0.axis0.motor.error == 0` to confirm that calibration succeeded.
4. To save motor calibration set `odrv0.axis0.motor.config.pre_calibrated = True` and call `odrv0.save_configuration()`

## C. Calibrate Hall Sensor
1. Before starting refer to [pinout](https://docs.odriverobotics.com/encoders.html#hall-effect-encoders) to correctly hook up hall sensor wires.
2. Set the following configs:

 ```
// axis0
odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = odrv0.axis0.motor.config.pole_pairs * 6
odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.encoder.config.calib_scan_distance = 150
odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL
// axis 1
odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis1.encoder.config.cpr = odrv0.axis0.motor.config.pole_pairs * 6
odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.encoder.config.calib_scan_distance = 150
odrv0.config.gpio12_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio13_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio14_mode = GPIO_MODE_DIGITAL
```
3. Run polarity calibration by calling `odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION`. Check `odrv0.axis0.encoder.error == 0` to confirm that calibration succeeded.
4. Run calibration by calling `odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION`. Check `odrv0.axis0.encoder.error == 0` to confirm that calibration succeeded.
5. To save encoder calibration set `odrv0.axis0.encoder.config.pre_calibrated = True` and call `odrv0.save_configuration()`.

## D. Tune Controller

Follow steps in  [tuning guide](https://docs.odriverobotics.com/control).

- Set `odrv0.axis0.controller.config.vel_limit` to below the max speed motor can spin given supply voltage.
Units are revolutions/second. This can be calculated approximaty `0.7 * supply voltage * KV / 60`. Eg for the 6384 motors its 33 rev/s @ 24V.
- Set `odrv0.axis0.motor.config.current_lim` depending on motor (to not burn it). The ODrive can  handle around 40A continous phase current without active cooling so recommend limiting to 50A max.
- Tune current control loop by adjusting `odrv0.axis0.motor.config.current_control_bandwidth` until the current response is stable. Can verify by sending speed or velocity setpoints and looking live plot of the measured phase current.
- Tune velocity control loop by sending velocity setpoint an viewing live plot of estimate.
  1. Set `odrv0.axis0.controller.config.vel_integrator_gain = 0`
  2. Bump up `odrv0.axis0.controller.config.vel_gain` to the point where there is ossilation the back down a bit.
  3. Bump up `odrv0.axis0.controller.config.vel_integrator_gain` until there is overshoot then back down a bit.
- Tune position control loop by sending position setpoints and live ploting the feedback.
  - Bump up `odrv0.axis0.controller.config.pos_gain` until there is overshoot then back down a bit.
  - Make sure that the setpoint is not too far away from current `odrv0.axis0.encoder.pos_estimate`, otherwise the error compensation will be really large and the controller will send unrealistic velocities.


## Reference Configurations
### BDUAV6384

- odrv0.axis0.controller.config.vel_limit = 50
- odrv0.axis0.motor.config.current_lim = 60
- odrv0.axis0.motor.config.current_lim_margin = 20
- odrv0.axis0.motor.config.current_control_bandwidth = 100
- odrv0.axis0.controller.config.pos_gain = 3
- odrv0.axis0.controller.config.vel_gain = 0.05
- odrv0.axis0.controller.config.vel_integrator_gain = 0.03
- odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL



# Troubleshoot
If error occured, the error codes stored in the following fields:

```
odrv0.axis.error
odrv0.axis.motor.error
odrv0.axis.encoder.error
odrv0.axis.controller.error
```
Refer to [troubleshoot guide](https://docs.odriverobotics.com/troubleshooting#error-codes) to decipher the error string.
Once addressed, the error fields can be set to 0 to clear, and operation can be resumed by re-entering closed loop control
`odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`. Also `odrv0.reboot()` will clear the errors as well.
## Error codes:
### ERROR_CURRENT_LIMIT_VIOLATION
This is a problem with the current control loop where it overshoots the current limit set.
Fix by decreasing `odrv0.axis0.motor.config.current_control_bandwidth` to make current control less aggressive.
Can also increase `odrv0.axis0.motor.config.current_lim_margin` to allow the overshoot without erroring out, but note overshoot shouldn't be more than 30% of target in this case the configured current limit.

### ERROR_DC_BUS_OVER_REGEN_CURRENT
This happens when the motor is requested to spin down too quickly. By default the back EMF is absorbed by the brake resistor, but if its too large the resistor will get saturated. To fix either reduce jerkyness of the motor, or hook up a battery as power source to absorb the regen current.
Usually it is safe to charge lipo batteries at 1-2C. Once the allowable regen current is known, configure with:

```
odrv0.config.max_regen_current = 5
odrv0.config.dc_max_negative_current = -5 # note this is negative the above
``` 

### ERROR_PHASE_RESISTANCE_OUT_OF_RANGE
Need to bump up `odrv0.axis0.motor.config.resistance_calib_max_voltage` if the phase resistance of the motor is large. This is the case with hoverboard motors.

```
resistance_calib_max_voltage > calibration_current * phase_resistance
resistance_calib_max_voltage < 0.5 * vbus_voltage
```
### ILLEGAL_SENSOR_STATE
It could be noise on the encoder lines that can be fixed by adding 47nF caps see [here](https://discourse.odriverobotics.com/t/encoder-error-error-illegal-hall-state/1047/7?u=madcowswe).
Also it could be the hall states are not what the firmware expects because older versions have hall states hardcoded. This is the case with the BDUAV6384 motors. Update to `>= fw-v0.5.2` to pull in the hall state detection routine to fix.

# Common Commands

## Print Errors
```
dump_errors(odrv0)
```
## Idle
```
odrv0.axis0.requested_state = AXIS_STATE_IDLE
```
## Velocity Test
```
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 0
```
## Position Test
```
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_pos = 0
```
## Live Plotter
```
start_liveplotter(lambda: [odrv0.axis0.motor.current_control.Iq_measured])
start_liveplotter(lambda: [odrv0.axis0.controller.input_vel, odrv0.axis0.encoder.vel_estimate])
start_liveplotter(lambda: [odrv0.axis0.controller.input_pos, odrv0.axis0.encoder.pos_estimate])
```
## Backup and Restore
```
odrivetool backup-config myconfig.json
odrivetool restore-config myconfig.json
```
## Save Configuration
`odrv0.save_configuration()`
## Factory Reset
`odrv0.erase_configuration()`
## Reboot
`odrv0.reboot()`


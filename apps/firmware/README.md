## BoatController - firmware

This is the firmware that implements all features to control the boat.

It takes input from two radio channels, mpu6050 accelerometer and gyrometer data to control two motors
and two servos (one for each rudder). Input/Output are saved periodically on a SD card. All I/O uses servo protocol as describer [here](https://en.wikipedia.org/wiki/Servo_control).

SD card and mpu6050 are optional. The controller will still perform as best as it can without.

### SD card

SD and SDHC cards are supported, but the code has only been tested with 2GB SD cards. The SD card must be contain at least one valid FAT16 partition in the master boot record.

The controller used to create a random directory and log everything in three different files: `RADIO.TXT`, `OUTPUT.TXT` and `MPU6050.TXT`.
However, it appeared that the sdcard cache could not cope well with reading and writing to many different blocks. So the controller now creates a single file in the root directory. Its name is random.

This file contains three types of data (indicated by second element):
```
ticks, 0, radio_dir, radio_speed
ticks, 1, left_rudder, right_rudder, left_motor, right_motor
ticks, 2, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
```

`ticks` is a unsigned 32-bit integer when indicates when the frame was created. It has a resolution of 1 ms.

`radio_dir, radio_speed` are unsigned 16-bit integers.

`left_rudder, right_rudder, left_motor, right_motor` are unsigned 16-bit integers.

`accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z` are signed 16-bit integers.

The controller flushes all blocks to the SD card every 2 seconds such that not much data is lost when the board is powered off.

### MPU6050 accel/gyro

The PIC24FJ128GB202 communicates to the MPU6050 via I2C at 100kHz. Fast speed (400 kHz) is not used since the signal is bad due to the logic level converter. Pull-up resistors R2 and R3 are not fit.

The controller will attempt to load calibration data from the SD card by reading ```/CALIB.TXT```. This feature
will only be used if both MPU6050 and SD card are detected at initialisation.

### LED pattern

  - **fast blinking**: Controller is initialising
  - **one flash every 5 seconds**: Initialisation is complete
  - **one flash every 2 seconds**: Initialisation is complete, MPU6050 could not be initialised
  - **two flash every 2 seconds**: Initialisation is complete, sdcard could not be initialised
  - **three flash every 2 seconds**: Initialisation is complete, sdcard and MPU6050 could not be initialised

### Timer usage

|**timer index**|**module/driver**|**description**|**frequency (Hz)**|
|:--------------:|:--------------:|:-------------------:|:--------:|
| 1 | none | mcu_delay | 1000 |
| 2 | status | blink LED | 10 |
| 3 | output | Iterate over all states | 250 |
| 4 | output | Set pin low | 500-1000 |
| 5 | MPU6050 fifo driver | Read sample from MPU6050 | 50 |

## BoatController - firmware

This is the firmware that implements all features to control the boat.

It takes input from two radio channels, mpu6050 accelerometer and gyrometer data to control two motors
and two servos (one for each rudder). Input/Output are saved periodically on a SD card. All I/O uses servo protocol as describer [here](https://en.wikipedia.org/wiki/Servo_control).

SD card and mpu6050 are optional. The controller will still perform as best as it can without.

### SD card

SD and SDHC cards are supported. The SD card must be contain at least one valid FAT16 partition in the master boot record.
The controller will create a directory, whose name is random. In this directory, two files are created: ```IO.TXT``` and ```MPU6050.TXT```.

In ```IO.TXT```, the controller saves all radio frames and output frames using the following format:
```
radio_dir, radio_speed, left_rudder, right_rudder, left_motor, right_motor
```

In ```MPU6050.TXT```, the controller saves all samples from the MPU6050 using the following format:
```
accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
```

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

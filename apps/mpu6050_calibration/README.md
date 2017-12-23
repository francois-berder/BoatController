## BoatController - mpu6050 calibration

This application implements the calibration software for the MPU6050 accelerometer.

It asks the user to place the MPU6050 in three different positions (X, Y or Z axis pointing towards earth).
In each position, the software takes 128 samples and computes the average acceleration and gyrometer vectors.
Then, it computes a linear regression using the three accelerometer vectors (one for each position).

```
x = coeff_x * raw_x + offset_x
y = coeff_y * raw_y + offset_y
z = coeff_z * raw_z + offset_z
```

This gives 6 parameters in total, 2 for each component. Offset for gyroscope are also computed.

All numbers are not floating point numbers. They are manipulated as 12.4 fixed point integers.

Finally, they are saved on a SD card in the file ```/CALIB.TXT``` in this format:

```
accel_coeff_x, 0, 0, accel_offset_x
0, accel_coeff_y, 0, accel_offset_y
0, 0, accel_coeff_z, accel_offset_z
gyro_offset_x, gyro_offset_y, gyro_offset_z
```

Once the calibration is finished, the new calibration data is loaded and the firmware prints accelerometer data 10 times a second.

### LED status

The LED is turned on during initialisation. Once initialisation is finished and during calibration, the LED is blinking. If the LED is not blinking, something went wrong during the initialisation. After calibration, the LED stops blinking.

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

This gives 6 parameters in total, 2 for each component. They are not floating point numbers. They are manipulated as 12.4 fixed point integers.

Finally, they are saved on a SD card in the file ```/CALIB.TXT``` in this format:

```
coeff_x, 0, 0, offset_x
0, coeff_y, 0, offset_y
0, 0, coeff_z, offset_z
```
### LED status

The LED is turned on during initialisation. Once initialisation is finished and during calibration, the LED is blinking. If the LED is not blinking, something went wrong during the initialisation. After calibration, the LED stops blinking.

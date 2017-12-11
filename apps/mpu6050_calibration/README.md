## BoatController - mpu6050 calibration

This application implements the calibration software for the MPU6050 accelerometer.

This is still the first version of this application: it only computes the average of 128 samples while
assuming that the MPU6050 lies on a flat surface, immobile. These averages (6 in total) are saved on
a SD card using CSV format in the file ```/CALIB.TXT```.

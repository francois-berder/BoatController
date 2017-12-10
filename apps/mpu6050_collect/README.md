## BoatController - mpu6050 sample collection

This firmware is used to collect samples from the MPU6050. These samples are written on the sdcard in
the file ```/MPU6050.TXT```.

Each line of the file represents a sample in the following format:
```
accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
```

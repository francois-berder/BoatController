# BoatController


The BoatController project consists of different applications for a board that is in charge of controlling a boat, hence the name...It is a fork of [pic24-framework](https://github.com/francois-berder/pic24-framework).


The board consists of a PIC24FJ128GB202, a micro SD card slot, a I2C port, 6 servo slots. The PIC24 is in charge of interpreting commands from two radio channels to control two rudders and two motors. Additional information is provided by a MPU6050 accelerometer/gyroscope via I2C. Data can be logged on a micro SD card.

Three applications have been developped:
   - **mpu6050_collect**: perform data collection from the MPU6050 and save it to the micro SD card.
   - **mpu6050_calibration**: perform calibration of the MPU6050 and save calibration data on the micro SD card.
   - **firmware**: main application that controls motors and rudders depending on radio commands. Data are saved on a micro SD card, if present.

For more information about these applications, have a look at the ```README.md``` present in each application directory.

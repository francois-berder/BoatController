/*
 * Copyright (C) 2017  Francois Berder <fberder@outlook.fr>
 *
 * This file is part of pic24-framework.
 *
 * pic24-framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * pic24-framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pic24-framework.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>

struct mpu6050_sample_t {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } accel;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } gyro;
};

/**
 * @brief Initialise MPU6050
 *
 * I2C must have been initialised and GPIO pins configured as digital I/O.
 *
 * @param[in] i2c_num
 */
int mpu6050_init(unsigned int i2c_num);

/**
 * @brief Get accelerometer and gyroscope readings from the MPU6050
 *
 * mpu6050_init must have been called before.
 *
 * @param[in] i2c_num
 * @param[out] sample
 */
void mpu6050_get_acc_gyro(unsigned int i2c_num, struct mpu6050_sample_t *sample);

/**
 * @brief Get accelerometer readings from the MPU6050
 *
 * mpu6050_init must have been called before.
 *
 * @param[in] i2c_num
 * @param[out] sample Only accelerometer data will be valid
 */
void mpu6050_get_acc(unsigned int i2c_num, struct mpu6050_sample_t *sample);

/**
 * @brief Get gyroscope readings from the MPU6050
 *
 * mpu6050_init must have been called before.
 *
 * @param[in] i2c_num
 * @param[out] sample Only gyroscope data will be valid
 */
void mpu6050_get_gyro(unsigned int i2c_num, struct mpu6050_sample_t *sample);

#endif

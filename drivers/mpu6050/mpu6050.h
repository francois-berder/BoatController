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
    uint32_t t;
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

struct mpu6050_calibration_data_t {
    struct {
        struct {
            int16_t x;
            int16_t y;
            int16_t z;
        } offset;

        struct {
            int16_t x;
            int16_t y;
            int16_t z;
        } coeff;
    } accel;

    struct {
        struct {
            int16_t x;
            int16_t y;
            int16_t z;
        } offset;
    } gyro;
};

struct mpu6050_dev_t {
    unsigned int i2c_num;
    struct mpu6050_calibration_data_t cdata;
};

/**
 * @brief Initialise MPU6050
 *
 * I2C must have been initialised and GPIO pins configured as digital I/O.
 *
 * @param[in] dev
 * @param[in] enable_acc 0 to disable accelerometer, 1 to enable it
 * @param[in] enable_gyro 0 to disable gyroscope, 1 to enable it
 * @param[in] 0 if successful, -1 otherwise
 */
int mpu6050_init(struct mpu6050_dev_t *dev, unsigned int enable_acc, unsigned int enable_gyro);

/**
 * @brief Get accelerometer and gyroscope readings from the MPU6050
 *
 * mpu6050_init must have been called before.
 *
 * @param[in] dev
 * @param[out] sample
 * @param[in] 0 if successful, -1 otherwise
 */
int mpu6050_get_acc_gyro(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample);

/**
 * @brief Get accelerometer readings from the MPU6050
 *
 * mpu6050_init must have been called before.
 *
 * @param[in] dev
 * @param[out] sample Only accelerometer data will be valid
 * @param[in] 0 if successful, -1 otherwise
 */
int mpu6050_get_acc(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample);

/**
 * @brief Get gyroscope readings from the MPU6050
 *
 * mpu6050_init must have been called before.
 *
 * @param[in] dev
 * @param[out] sample Only gyroscope data will be valid
 * @param[in] 0 if successful, -1 otherwise
 */
int mpu6050_get_gyro(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample);

/**
 * @brief Power up MPU6050
 *
 * @param[in] dev
 */
void mpu6050_power_up(struct mpu6050_dev_t *dev);

/**
 * @brief Power down MPU6050
 *
 * @param[in] dev
 */
void mpu6050_power_down(struct mpu6050_dev_t *dev);

/**
 * @brief Create default calibration data
 *
 * This default calibration data does not rectify anything and leave data
 * unchanged.
 */
struct mpu6050_calibration_data_t mpu6050_create_default_calibration_data(void);

#endif

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

struct mpu6050_calibration_data_t {
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
};

/**
 * @brief Initialise MPU6050
 */
int mpu6050_init(void);

/**
 * @brief Set calibration data for MPU6050
 *
 * @param[in] cdata
 */
void mpu6050_set_calibration_data(struct mpu6050_calibration_data_t cdata);

/**
 * @return Number of samples in the FIFO
 */
unsigned int mpu6050_get_sample_count(void);

/**
 * @brief Reads a sample from the FIFO
 *
 * If no samples are in the FIFO, this function does nothing.
 *
 * @param[out] sample
 */
void mpu6050_get_sample(struct mpu6050_sample_t *sample);

#endif

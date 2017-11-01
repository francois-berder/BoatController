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
 */
int mpu6050_init(void);

/**
 * @brief Retrieves the number of samples available in the FIFO of the device.
 *
 * @return Number of samples in the FIFO.
 */
uint16_t mpu6050_get_sample_cnt(void);

/**
 * @brief Reads an array of samples from the sensor
 *
 * @param[out] samples Array to store samples from device.
 * @param[in] sample_cnt Number of samples to read from the device.
 */
void mpu6050_read_fifo(struct mpu6050_sample_t *samples, uint16_t samples_cnt);

/**
 * @brief Clears the FIFO on the device.
 *
 * It has been found that reading the FIFO does not decrease the number of
 * samples until the FIFO gets full and discards new samples. Hence, it is
 * advised to periodically read every available samples from the device and
 * then clear the FIFO.
 */
void mpu6050_clear_fifo(void);

#endif

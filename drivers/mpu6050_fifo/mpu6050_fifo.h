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

#ifndef __MPU6050_FIFO_H__
#define __MPU6050_FIFO_H__

#include "mpu6050/mpu6050.h"

/**
 * @brief Initialise FIFO and timer
 *
 * mpu6050_init must have been called before.
 * The last two parameters control what will be retrieved from
 * the MPU6050. Hence, if use_gyro is set to 0, samples will not
 * hold valid data.
 *
 * @param[in] dev
 * @param[in] use_acc A boolean to read or not accelerometer data
 * @param[in] use_gyro A boolean to read or not gyroscope data
 */
void mpu6050_fifo_init(struct mpu6050_dev_t dev, unsigned int use_acc, unsigned int use_gyro);

/**
 * @brief Start collecting samples
 */
void mpu6050_fifo_start(void);

/**
 * @brief Stop collecting samples
 */
void mpu6050_fifo_stop(void);

/**
 * @brief Check whether driver is running or not
 *
 * @retval 0 not running
 * @retval 1 running
 */
int mpu6050_fifo_is_running(void);

/**
 * @brief Clear all samples in the FIFO
 */
void mpu6050_fifo_clear_samples(void);

/**
 * @return Number of samples available
 */
unsigned int mpu6050_fifo_get_sample_count(void);

/**
 * @brief Get a sample from the FIFO
 *
 * @retval 1 A sample was retrieved from the FIFO
 * @retval 0 No samples available
 * @retval -1 An error occured
 */
int mpu6050_fifo_get_sample(struct mpu6050_sample_t *sample);

#endif

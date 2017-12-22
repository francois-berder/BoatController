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

#ifndef __IMU_H__
#define __IMU_H__

#include "mpu6050/mpu6050.h"

/**
 * @brief Initialise FIFO and timer
 */
void imu_init(struct mpu6050_dev_t dev);

/**
 * @brief Start collecting samples
 */
void imu_start(void);

/**
 * @brief Stop collecting samples
 */
void imu_stop(void);

/**
 * @brief Clear all samples in the FIFO
 */
void imu_clear_samples(void);

/**
 * @return Number of samples available
 */
unsigned int imu_get_sample_count(void);

/**
 * @brief Get a sample from the FIFO
 *
 * @retval 1 A sample was retrieved from the FIFO
 * @retval 0 No samples available
 * @retval -1 An error occured
 */
int imu_get_sample(struct mpu6050_sample_t *sample);

#endif

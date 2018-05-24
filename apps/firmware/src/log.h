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

#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>
#include "mpu6050/mpu6050.h"
#include "output.h"
#include "radio.h"

/**
 * @brief Initialise log module
 *
 * Create a file in the root directory. The name of the
 * file is random.
 */
void log_init(void);

/**
 * @brief Log a radio frame
 *
 * @param[in] rf
 */
void log_radio_frame(struct radio_frame_t rf);

/**
 * @brief Log an output frame
 */
void log_output_frame(uint32_t t, struct output_frame_t of);

/**
 * @brief Log a MPU6050 frame
 */
void log_mpu6050_frame(struct mpu6050_sample_t s);

/**
 * @brief Flush sdcard cache
 */
void log_flush(void);

/**
 * @brief Close log file (if opened)
 *
 * log_radio_frame, log_output_frame and log_mpu6050_frame
 * will not anything after this call.
 */
void log_stop(void);

/**
 * @retval 1 Log is running
 * @retval 0 Log is not running
 */
int log_is_running(void);

#endif

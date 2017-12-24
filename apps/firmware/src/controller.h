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

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "config.h"
#include "mpu6050/mpu6050.h"

/**
 * @brief Initialise controller
 *
 * @param[in] config
 * @param[in] mpu6050_dev
 */
void controller_init(struct board_config_t config, struct mpu6050_dev_t mpu6050_dev);

/**
 * @brief Run controller forever
 *
 * Note that this function does not return.
 */
void controller_run(void);

#endif

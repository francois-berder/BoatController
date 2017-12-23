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

#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdint.h>

struct radio_frame_t {
    uint16_t dir;
    uint16_t speed;
};

/**
 * @brief Configure radio module
 *
 * Notice that this function does not enable this module.
 */
void radio_configure(void);

/**
 * @brief Enable radio module
 */
void radio_enable(void);

/**
 * @brief Disable radio module
 */
void radio_disable(void);

/**
 * @retval 0 No frame is avaialble
 * @retval 1 A frame is available
 */
uint8_t radio_has_frame(void);

/**
 * @brief Retrieve a frame from radio module FIFO
 *
 * Do **not** call this function if no frame is avaiable.
 * Check frame availability with radio_has_frame.
 *
 * @return a radio frame
 */
struct radio_frame_t radio_get_frame(void);

#endif

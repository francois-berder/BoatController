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

#ifndef __SDCARD_H__
#define __SDCARD_H__

#include <stdint.h>

/**
 * @brief Initialise SD card
 *
 * @return 0 if successful, -1 otherwise
 */
int sdcard_init(void);

/**
 * @return Block length
 */
uint32_t sdcard_get_block_length(void);

/**
 * @brief Read block from sector
 *
 * @param[in] block Array of bytes at least greater or equal than block length
 * @param[in] sector
 * @return 0 if successful, -1 otherwise
 */
int sdcard_read_block(void *block, uint32_t sector);

/**
 * @brief Write block to sector
 *
 * @param[in] block Array of bytes at least greater or equal than block length
 * @param[in] sector
 * @return 0 if successful, -1 otherwise
 */
int sdcard_write_block(const void *block, uint32_t sector);

#endif

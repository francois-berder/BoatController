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

#ifndef __BLOCK_STORAGE_H__
#define __BLOCK_STORAGE_H__

#include <stdint.h>

/**
 * @brief Initialise block storage
 *
 * This function does not configure the SD card. It is assumed
 * that it has already been done before.
 * The current address is set to 0.
 */
void block_storage_init(void);

/**
 * @brief Read bytes from block storage at current address
 *
 * @param[out] buffer
 * @param[in] length Number of bytes to read
 */
void block_storage_read(void *buffer, uint32_t length);

/**
 * @brief Read one byte from block storage at current address
 *
 * @param[in] data
 */
void block_storage_read_byte(void *data);

/**
 * @brief Write bytes to block storage at current address
 *
 * @param[in] buffer
 * @param[in] length Number of bytes to write
 */
void block_storage_write(const void *buffer, uint32_t length);

/**
 * @brief Set the current address
 *
 * @param[in] address
 */
void block_storage_seek(uint32_t address);

/**
 * @brief Flush all dirty blocks to the SD card
 *
 * Reset all counters.
 * This function should be called from time to time to
 * ensure that:
 *   - data is written back to the SD card
 *   - cache entry eviction works in an optimal fashion
 */
void block_storage_flush(void);

#endif
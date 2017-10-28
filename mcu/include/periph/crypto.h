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

#ifndef __MCU_CRYPTO_H__
#define __MCU_CRYPTO_H__

#include <stdint.h>

/**
 * @defgroup crypto Crypto peripheral
 * @{
 */

/**
 * @brief Fill a buffer with random bytes
 *
 * @param[out] dst
 * @param[in] length Number of random bytes to generate
 */
void crypto_get_random(void *dst, uint32_t length);

/**
 * @brief Enable crypto module
 */
void crypto_enable(void);

/**
 * @brief Disable crypto module
 */
void crypto_disable(void);

/**
 * @brief Power up crypto module
 */
void crypto_power_up(void);

/**
 * @brief Power down crypto module
 */
void crypto_power_down(void);

/** @} */

#endif

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

#ifndef __MCU_GPIO_H__
#define __MCU_GPIO_H__

#include <stdint.h>

/**
 * @defgroup gpio GPIO peripheral
 * @{
 */

/** Define a gpio pin from its port and index */
#define GPIO_PIN(PORT, INDEX)   (((PORT) << 4) | ((INDEX) & 0xF))

/**
 * @brief Initialise a pin as an output
 *
 * @param[in] pin
 * @param[in] val
 */
void gpio_init_out(uint8_t pin, uint8_t val);

/**
 * @brief Initialise a pin as an input
 *
 * @param[in] pin
 */
void gpio_init_in(uint8_t pin);

/**
 * @brief Set the state of a pin.
 *
 * If the pin is invalid or the pin is not configured
 * as an output, then this function does nothing.
 *
 * @param[in] pin
 * @param[in] val
 */
void gpio_write(uint8_t pin, uint8_t val);

/**
 * @brief Read the state of a pin
 *
 * Notice that the pin does not have to be configured
 * as an input.
 *
 * @param[in] pin
 *
 * @retval 0 if the pin is low
 * @retval 1 if the pin is high
 */
int gpio_read(uint8_t pin);

/**
 * @brief Toggle the state of a pin
 *
 * If the pin is invalid or the pin is not configured
 * as an output, then this function does nothing.
 *
 * @param[in] pin
 */
void gpio_toggle(uint8_t pin);

/** @} */

#endif

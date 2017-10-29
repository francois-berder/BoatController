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

#ifndef __MCU_TIMER1_H__
#define __MCU_TIMER1_H__

#include <stdint.h>

/**
 * @defgroup timer1 Timer 1 peripheral
 * @{
 */

/**
 * @brief Timer 1 callback
 *
 * This function can be redefined by the user.
 * The default implementation does nothing.
 */
void timer1_callback(void);

/**
 * @brief Configure timer1
 *
 * The timer1 must be powered up before calling this function.
 * This function does not enable the timer 1.
 *
 * @param[in] prescaler
 * @param[in] period
 * @param[in] enable_interrupt
 */
void timer1_configure(uint8_t prescaler, uint16_t period, uint8_t enable_interrupt);

/**
 * @brief Start timer 1
 */
void timer1_start(void);

/**
 * @brief Stop timer 1
 */
void timer1_stop(void);

/**
 * @brief Power up timer 1
 */
void timer1_power_up(void);

/**
 * @brief Power down timer 1
 *
 * The module must be disabled before calling this function.
 * Once timer 1 is powered down, do not call any functions
 * except timer1_power_up.
 */
void timer1_power_down(void);

/** @} */

#endif

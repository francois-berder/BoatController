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


#ifndef __MCU_TIMER_H__
#define __MCU_TIMER_H__

#include <stdint.h>

/**
 * @defgroup timer Timer peripheral
 * @{
 */

/**
 * Timer callbacks that can be redefined by the user.
 * The default implementation does not do anything.
 */
void timer2_callback(void);
void timer3_callback(void);
void timer4_callback(void);
void timer5_callback(void);

/**
 * @brief Configure a timer
 *
 * The timer module must be powered up before calling this function.
 * This function does not enable the timer.
 *
 * @param[in] timer_num
 * @param[in] prescaler
 * @param[in] period
 * @param[in] enable_interrupt
 */
void timer_configure(unsigned int timer_num, uint8_t prescaler, uint16_t period, uint8_t enable_interrupt);

/**
 * @brief Start a timer
 *
 * @param[in] timer_num
 */
void timer_start(unsigned int timer_num);

/**
 * @brief Stop a timer
 *
 * @param[in] timer_num
 */
void timer_stop(unsigned int timer_num);

/**
 * @brief Check whether a timer is running or not
 *
 * @retval 0 timer is not running
 * @retval 1 timer is running
 */
int timer_is_running(unsigned int timer_num);

/**
 * @brief Get period in nanoseconds
 *
 * @param[in] timer_num
 * @return timer period
 */
uint32_t timer_get_period(unsigned int timer_num);

/**
 * @brief Power up a timer
 *
 * @param[in] timer_num
 */
void timer_power_up(unsigned int timer_num);

/**
 * @brief Power down a timer
 *
 * The module must be disabled before calling this function.
 * Do not call any other functions except timer_power_up once
 * the timer module is powered down.
 *
 * @param[in] timer_num
 */
void timer_power_down(unsigned int timer_num);

/** @} */

#endif

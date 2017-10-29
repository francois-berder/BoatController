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

#ifndef __MCU_PWM_H__
#define __MCU_PWM_H__

#include <stdint.h>

/**
 * @defgroup pwm Pulse Width Modulation peripheral
 * @{
 */

/**
 * @brief Initialise a PWM module
 *
 * The PWM module must be powered up and must not be running before calling this
 * function.
 * This function does not enable the PWM module.
 * Since the PWM module uses one of timer 2-5 as a source,
 * it is necessary to initialise and start the timer before initialising the
 * PWM.
 * If you change the timer period, you need to call this function again.
 *
 * @param[in] pwm_num Index of the PWM module
 * @param[in] timer_num Index of the timer module to use as source
 */
void pwm_configure(unsigned int pwm_num, unsigned int timer_num);

/**
 * @brief Enable a PWM module
 *
 * @param[in] pwm_num Index of the PWM module
 */
void pwm_enable(unsigned int pwm_num);

/**
 * @brief Disable a PWM module
 *
 * @param[in] pwm_num Index of the PWM module
 */
void pwm_disable(unsigned int pwm_num);

/**
 * @param[in] pwm_num Index of the PWM module
 * @return Period of the pwm in nanoseconds
 */
uint32_t pwm_get_period(unsigned int pwm_num);

/**
 * @param[in] pwm_num Index of the PWM module
 * @return Duty cycle of the pwm in nanoseconds
 */
uint32_t pwm_get_duty_cycle(unsigned int pwm_num);

/**
 * @brief Change the duty cycle
 *
 * If the duty cycle is greater than the period of the timer, it will be capped
 * to the period.
 *
 * @param[in] pwm_num Index of the PWM module
 * @param[in] duty_cycle in nanoseconds
 */
void pwm_set_duty_cycle(unsigned int pwm_num, uint32_t duty_cycle);

/**
 * @brief Power up a PWM module
 *
 * @param[in] pwm_num Index of the PWM module
 */
void pwm_power_up(unsigned int pwm_num);

/**
 * @brief Power down a PWM module
 *
 * @param[in] pwm_num Index of the PWM module
 */
void pwm_power_down(unsigned int pwm_num);

/** @} */

#endif

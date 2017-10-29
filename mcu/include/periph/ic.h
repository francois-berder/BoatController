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

#ifndef __MCU_IC_H__
#define __MCU_IC_H__

#include <stdint.h>

/**
 * @defgroup ic Input Capture peripheral
 * @{
 */

/**
 * @brief Input Capture mode
 */
enum IC_MODE {
    IC_EDGE_DETECT = 0b001,             /**< Capture occurs at every edge */
    IC_FALLING_EDGE = 0b010,            /**< Capture occurs at every falling edge */
    IC_RISING_EDGE = 0b011,             /**< Capture occurs at every rising edge */
    IC_FOURTH_RISING_EDGE = 0b100,      /**< Capture occurs every 4th rising edge */
    IC_SIXTEENTH_RISING_EDGE = 0b101    /**< Capture occurs every 16th rising edge */
};

/**
 * @brief Input Capture 1 callback
 *
 * This function is called only if the input capture was
 * enabled and configured with interrupts enabled.
 *
 * This function can be redefined by the user.
 * The default implementation does not do anything.
 */
void ic1_callback(void);

/**
 * @brief Input Capture 2 callback
 *
 * This function is called only if the input capture was
 * enabled and configured with interrupts enabled.
 *
 * This function can be redefined by the user.
 * The default implementation does not do anything.
 */
void ic2_callback(void);

/**
 * @brief Input Capture 3 callback
 *
 * This function is called only if the input capture was
 * enabled and configured with interrupts enabled.
 *
 * This function can be redefined by the user.
 * The default implementation does not do anything.
 */
void ic3_callback(void);

/**
 * @brief Input Capture 4 callback
 *
 * This function is called only if the input capture was
 * enabled and configured with interrupts enabled.
 *
 * This function can be redefined by the user.
 * The default implementation does not do anything.
 */
void ic4_callback(void);

/**
 * @brief Input Capture 5 callback
 *
 * This function is called only if the input capture was
 * enabled and configured with interrupts enabled.
 *
 * This function can be redefined by the user.
 * The default implementation does not do anything.
 */
void ic5_callback(void);

/**
 * @brief Input Capture 6 callback
 *
 * This function is called only if the input capture was
 * enabled and configured with interrupts enabled.
 *
 * This function can be redefined by the user.
 * The default implementation does not do anything.
 */
void ic6_callback(void);

/**
 * @brief Configure an input capture module
 *
 * This function does not enable the input capture module.
 * The module must be powered up before calling this function (this is
 * the case after any reset).
 *
 * @param[in] ic_num
 * @param[in] mode
 * @param[in] enable_interrupt
 */
void ic_configure(unsigned int ic_num, enum IC_MODE mode, uint8_t enable_interrupt);

/**
 * @brief Enable an input capture module
 *
 * @param[in] ic_num
 */
void ic_enable(unsigned int ic_num);

/**
 * @brief Disable an input capture module
 *
 * @param[in] ic_num
 */
void ic_disable(unsigned int ic_num);

/**
 * @brief Check if Input Capture buffer is not empty
 *
 * @param[in] ic_num
 * @return 0 if no data is available, 1 otherwise
 */
uint8_t ic_has_data(unsigned int ic_num);

/**
 * @param[in] ic_num
 * @return input capture buffer
 */
uint16_t ic_get_data(unsigned int ic_num);

/**
 * @brief Power up input capture module
 *
 * @param[in] ic_num
 */
void ic_power_up(unsigned int ic_num);

/**
 * @brief Power down input capture module
 *
 * The module must be disabled before calling this function.
 *
 * Once the module is powered down, do not call any
 * other functions except ic_power_up.
 *
 * @param[in] ic_num
 */
void ic_power_down(unsigned int ic_num);

/** @} */

#endif

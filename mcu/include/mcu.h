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

#ifndef __MCU_H__
#define __MCU_H__

#include <stdint.h>

/**
 * @defgroup mcu MCU
 * @{
 */

/** Reset cause of the MCU */
enum reset_cause {
    POR_RESET,          /**< Power on reset */
    BOR_RESET,          /**< Brown out reset */
    SOFT_RESET,         /**< Software reset */
    WATCHDOG_RESET,     /**< Watchdog reset */
    EXT_RESET           /**< Pin reset */
};

/**
 * @brief Set the MCU clock.
 *
 * Beware this function does NOT change the system clock.
 * It is assumed the clock has already been changed before calling this
 * function.
 * Any subsequent calls of mcu_get_system_clock will return @p system_clock
 * You might want to reconfigure all peripherals that depend on the
 * system clock.
 *
 * @param[in] system_clock in Hz
 */
void mcu_set_system_clock(uint32_t system_clock);

/**
 * @return Current system clock in Hz
 */
uint32_t mcu_get_system_clock(void);

/**
 * @brief Enable interrupts
 */
void mcu_enable_interrupts(void);

/**
 * @brief Disable interrupts
 */
void mcu_disable_interrupts(void);

/**
 * @brief Disable interrupt and return interrupt context
 */
unsigned int mcu_save_context(void);

/**
 * @brief Restore interrupt context
 */
void mcu_restore_context(unsigned int ctx);

/**
 * @return ID of the MCU
 */
uint16_t mcu_get_id(void);

/**
 * @brief Short busy loop
 *
 * Do not use it for precise timing.
 * This function relies on the core timer.
 * Hence, configure the relevant timer before
 * calling this function or the mcu will get stuck !
 *
 * @param[in] ticks Number of ticks to wait
 */
void mcu_delay(uint32_t ticks);

/**
 * @brief Short busy loop
 *
 * Note: Do not use it for precise timing
 *
 * @param[in] secs Number of seconds to wait
 */
void mcu_delay_sec(uint32_t secs);

/**
 * @brief Perform a soft reset of the MCU.
 */
void mcu_reset(void);

/**
 * @return The cause of the last reset.
 */
enum reset_cause mcu_get_reset_cause(void);

/**
 * @brief Set the MCU in idle mode
 */
void mcu_idle(void);

/**
 * @brief Set the MCU in sleep mode
 */
void mcu_sleep(void);

/** @} */

#endif

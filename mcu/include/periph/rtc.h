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

#ifndef __MCU_RTC_H__
#define __MCU_RTC_H__

#include <stdint.h>

/**
 * @defgroup rtc Real Time Clock peripheral
 * @{
 */

/**
 * @brief Represents a date
 */
struct rtc_date_t {
    uint8_t year;           /**< year in range 0..99 */
    uint8_t month;          /**< month in range 0..11 */
    uint8_t day;            /**< day in range 1..31 */
    uint8_t wday;           /**< wday in range 0..6 */
    uint8_t hour;           /**< hour in range 0..23 */
    uint8_t min;            /**< min in range 0..59 */
    uint8_t sec;            /**< sec in range 0..59 */
};

/**
 * @brief Configure RTC module
 *
 * @param[in] use_internal_clock 1: use LPRC 0: use SOSC
 */
void rtc_configure(uint8_t use_internal_clock);

/**
 * @brief Enable RTC module
 */
void rtc_enable(void);

/**
 * @brief Disable RTC module
 */
void rtc_disable(void);

/**
 * @brief Set date
 *
 * Make sure that date is valid as there is no checking done.
 *
 * @param[in] date
 */
void rtc_set_date(struct rtc_date_t date);

/**
 * @return Current date
 */
struct rtc_date_t rtc_get_date(void);

/** @} */

#endif

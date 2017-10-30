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

#include <xc.h>
#include "periph/rtc.h"

static uint8_t bcd_to_binary(uint8_t tens, uint8_t ones)
{
    return tens * 10 + ones;
}

static uint8_t binary_to_bcd(uint8_t a)
{
    uint32_t tens = a / 10;
    uint32_t ones = a - tens * 10;

    return (tens << 4) | ones;
}

static inline void unlock_rcfgcal(void)
{
    NVMKEY = 0x55;
    NVMKEY = 0xAA;
    RCFGCAL |= _RCFGCAL_RTCWREN_MASK;
    NVMKEY = 0;
}

static inline void lock_rcfgcal(void)
{
    RCFGCAL &= ~_RCFGCAL_RTCWREN_MASK;
}

void rtc_configure(uint8_t use_internal_clock)
{
    unlock_rcfgcal();
    RCFGCAL = 0;        /* Disable RTC */
    ALCFGRPT = 0;       /* Disable alarm */
    RTCPWC = 0;         /* Disable power control */

    if (use_internal_clock)
        RTCPWC |= 0b01 << _RTCPWC_RTCLK_POSITION;

    lock_rcfgcal();
}

void rtc_enable(void)
{
    unlock_rcfgcal();
    RCFGCAL |= _RCFGCAL_RTCEN_MASK;
    lock_rcfgcal();
}

void rtc_disable(void)
{
    unlock_rcfgcal();
    RCFGCAL &= ~_RCFGCAL_RTCEN_MASK;
    lock_rcfgcal();
}

void rtc_set_date(struct rtc_date_t date)
{
    uint16_t val[4];
    const uint16_t rcfgcal = RCFGCAL;

    /* Always disable RTC to avoid roll-over when writing to RTCVAL */
    rtc_disable();

    /*          | year */
    val[0] = binary_to_bcd(date.year);

    /* month    | day */
    val[1] = binary_to_bcd(date.month) << 8;
    val[1] |= binary_to_bcd(date.day);

    /* weekdays | hour */
    val[2] = binary_to_bcd(date.wday) << 8;
    val[2] |= binary_to_bcd(date.hour);

    /* minutes  | seconds */
    val[3] = (unsigned int)binary_to_bcd(date.min) << 8;
    val[3] |= binary_to_bcd(date.sec);

    RCFGCAL |= 0b11 << _RCFGCAL_RTCPTR_POSITION;

    unlock_rcfgcal();
    RTCVAL = val[0];
    RTCVAL = val[1];
    RTCVAL = val[2];
    RTCVAL = val[3];
    lock_rcfgcal();

    /* Re-enable RTC if it was enabled before setting date */
    if (rcfgcal & _RCFGCAL_RTCEN_MASK)
        rtc_enable();
}

struct rtc_date_t rtc_get_date(void)
{
    struct rtc_date_t date;
    uint16_t val[4];

    /* Wait until it is safe to read RTCVAL */
    while (RCFGCAL & _RCFGCAL_RTCSYNC_MASK)
        ;

    RCFGCAL |= 0b11 << _RCFGCAL_RTCPTR_POSITION;

    val[0] = RTCVAL;    /*          | year */
    val[1] = RTCVAL;    /* month    | day */
    val[2] = RTCVAL;    /* weekdays | hour */
    val[3] = RTCVAL;    /* minutes  | seconds */

    date.year = bcd_to_binary(val[0] >> 4, val[0] & 0xF);
    date.month = bcd_to_binary(val[1] >> 12, (val[1] >> 8) & 0xF);
    date.day = bcd_to_binary((val[1] >> 4) & 0xF, val[1] & 0xF);
    date.wday = bcd_to_binary(val[2] >> 12, (val[2] >> 8) & 0xF);
    date.hour = bcd_to_binary((val[2] >> 4) & 0xF, val[2] & 0xF);
    date.min = bcd_to_binary(val[3] >> 12, (val[3] >> 8) & 0xF);
    date.sec = bcd_to_binary((val[3] >> 4) & 0xF, val[3] & 0xF);

    return date;
}

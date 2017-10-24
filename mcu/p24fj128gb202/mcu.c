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
#include "mcu.h"

static uint32_t system_clock;

void mcu_set_system_clock(uint32_t _system_clock)
{
    system_clock = _system_clock;
}

uint32_t mcu_get_system_clock(void)
{
    return system_clock;
}

uint16_t mcu_get_id(void)
{
    volatile uint16_t *devid = (volatile uint16_t *)0xFF0000;

    return *devid;
}

void mcu_delay(uint32_t ns)
{
    (void)ns;

    /* @todo Not yet implemented */
}

void mcu_reset(void)
{
    __asm__ __volatile__ ("reset");
}

enum reset_cause mcu_get_reset_cause(void)
{
    static enum reset_cause cause;

    if (cause)
        return cause;

    if (RCON & _RCON_WDTO_MASK)
        cause = WATCHDOG_RESET;
    else if (RCON & _RCON_SWR_MASK)
        cause = SOFT_RESET;
    else if (RCON & _RCON_EXTR_MASK)
        cause = EXT_RESET;
    else if (RCON & _RCON_POR_MASK)
        cause = POR_RESET;
    else if (RCON & _RCON_BOR_MASK)
        cause = BOR_RESET;

    RCON &= ~(_RCON_EXTR_MASK | _RCON_SWR_MASK | _RCON_WDTO_MASK |
              _RCON_BOR_MASK | _RCON_POR_MASK);

    return cause;
}

void mcu_idle(void)
{
    __asm__ __volatile__ ("pwrsav #1");
}

void mcu_sleep(void)
{
    __asm__ __volatile__ ("pwrsav #0");
}

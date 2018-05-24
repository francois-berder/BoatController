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
#include "core_timer.h"
#include "mcu.h"

static uint32_t system_clock;

void __attribute__((interrupt, no_auto_psv, noreturn)) _OscillatorFail(void)
{
    INTCON1 &= ~_INTCON1_OSCFAIL_MASK;
    while (1)
        ;
}

void __attribute__((interrupt, no_auto_psv, noreturn)) _AddressError(void)
{
    INTCON1 &= ~_INTCON1_ADDRERR_MASK;
    while (1)
        ;
}

void __attribute__((interrupt, no_auto_psv, noreturn)) _StackError(void)
{
    INTCON1 &= ~_INTCON1_STKERR_MASK;
    while (1)
        ;
}

void __attribute__((interrupt, no_auto_psv, noreturn)) _MathError(void)
{
    INTCON1 &= ~_INTCON1_MATHERR_MASK;
    while (1)
        ;
}

void mcu_set_system_clock(uint32_t _system_clock)
{
    system_clock = _system_clock;
}

uint32_t mcu_get_system_clock(void)
{
    return system_clock;
}

void mcu_enable_interrupts(void)
{
    SET_CPU_IPL(0);
}

void mcu_disable_interrupts(void)
{
    SET_CPU_IPL(7);
}

unsigned int mcu_save_context(void)
{
    unsigned int ctx;
    SET_AND_SAVE_CPU_IPL(ctx, 7);
    return ctx;
}

void mcu_restore_context(unsigned int ctx)
{
    RESTORE_CPU_IPL(ctx);
}

uint16_t mcu_get_id(void)
{
    volatile uint16_t *devid = (volatile uint16_t *)0xFF0000;

    return *devid;
}

void mcu_delay(uint32_t ticks)
{
    uint32_t now = core_timer_get_ticks();
    while (core_timer_get_ticks() - now < ticks)
        ;
}

void mcu_delay_sec(uint32_t secs)
{
    uint32_t i;

    for (i = 0; i < secs; ++i)
        mcu_delay(TICKS_PER_SEC);
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

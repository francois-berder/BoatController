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
#include "periph/timer1.h"
#include "periph_conf.h"

volatile uint32_t ticks;

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
    ++ticks;
    IFS0 &= ~_IFS0_T1IF_MASK;
}

void timer1_configure(uint8_t prescaler, uint16_t period)
{
    T1CON = (prescaler << _T1CON_TCKPS_POSITION) & _T1CON_TCKPS_MASK;
    PR1 = period;
    TMR1 = 0;
    ticks = 0;
}

void timer1_start(void)
{
    IFS0 &= ~_IFS0_T1IF_MASK;
    IEC0 |= _IEC0_T1IE_MASK;
    T1CON |= _T1CON_TON_MASK;
}

void timer1_stop(void)
{
    T1CON &= ~_T1CON_TON_MASK;
    IEC0 &= ~_IEC0_T1IE_MASK;
}

void timer1_power_up(void)
{
    PMD1 &= ~_PMD1_T1MD_MASK;
}

void timer1_power_down(void)
{
    PMD1 |= _PMD1_T1MD_MASK;
}

uint32_t timer1_get_tick_count(void)
{
    return ticks;
}

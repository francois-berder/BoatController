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

void __attribute__((weak)) timer1_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine timer1_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
    timer1_callback();
    IFS0 &= ~_IFS0_T1IF_MASK;
}

void timer1_configure(uint8_t prescaler, uint16_t period, uint8_t enable_interrupt)
{
    T1CON = (prescaler << _T1CON_TCKPS_POSITION) & _T1CON_TCKPS_MASK;
    PR1 = period;
    TMR1 = 0;

    IFS0 &= ~_IFS0_T1IF_MASK;
    if (enable_interrupt)
        IEC0 |= _IEC0_T1IE_MASK;
    else
        IEC0 &= ~_IEC0_T1IE_MASK;
}

void timer1_start(void)
{
    T1CON |= _T1CON_TON_MASK;
}

void timer1_stop(void)
{
    T1CON &= ~_T1CON_TON_MASK;
}

void timer1_power_up(void)
{
    PMD1 &= ~_PMD1_T1MD_MASK;
}

void timer1_power_down(void)
{
    PMD1 |= _PMD1_T1MD_MASK;
}

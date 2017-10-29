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
#include "periph/timer.h"
#include "periph_conf.h"

void __attribute__ ((weak)) timer2_callback(void)
{

}

void __attribute__ ((weak)) timer3_callback(void)
{

}

void __attribute__ ((weak)) timer4_callback(void)
{

}

void __attribute__ ((weak)) timer5_callback(void)
{

}

void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    timer2_callback();
    IFS0 &= ~ _IFS0_T2IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _T3Interrupt(void)
{
    timer3_callback();
    IFS0 &= ~ _IFS0_T3IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void)
{
    timer4_callback();
    IFS1 &= ~ _IFS1_T4IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _T5Interrupt(void)
{
    timer5_callback();
    IFS1 &= ~ _IFS1_T5IF_MASK;
}

void timer_configure(unsigned int timer_num,
                     uint8_t prescaler, uint16_t period,
                     uint8_t enable_interrupt)
{
    switch (timer_num) {
    case TIMER_2:
        T2CON = (prescaler << _T2CON_TCKPS_POSITION) & _T2CON_TCKPS_MASK;
        PR2 = period;
        TMR2 = 0;
        IFS0 &= ~ _IFS0_T2IF_MASK;
        break;
    case TIMER_3:
        T3CON = (prescaler << _T3CON_TCKPS_POSITION) & _T3CON_TCKPS_MASK;
        PR3 = period;
        TMR3 = 0;
        IFS0 &= ~ _IFS0_T3IF_MASK;
        break;
    case TIMER_4:
        T4CON = (prescaler << _T4CON_TCKPS_POSITION) & _T4CON_TCKPS_MASK;
        PR4 = period;
        TMR4 = 0;
        IFS1 &= ~ _IFS1_T4IF_MASK;
        break;
    case TIMER_5:
        T5CON = (prescaler << _T5CON_TCKPS_POSITION) & _T5CON_TCKPS_MASK;
        PR5 = period;
        TMR5 = 0;
        IFS1 &= ~ _IFS1_T5IF_MASK;
        break;
    }

    if (enable_interrupt) {
        switch (timer_num) {
        case TIMER_2:
            IEC0 |= _IEC0_T2IE_MASK;
            break;
        case TIMER_3:
            IEC0 |= _IEC0_T3IE_MASK;
            break;
        case TIMER_4:
            IEC1 |= _IEC1_T4IE_MASK;
            break;
        case TIMER_5:
            IEC1 |= _IEC1_T5IE_MASK;
            break;
        }
    } else {
        switch (timer_num) {
        case TIMER_2:
            IEC0 &= ~_IEC0_T2IE_MASK;
            break;
        case TIMER_3:
            IEC0 &= ~_IEC0_T3IE_MASK;
            break;
        case TIMER_4:
            IEC1 &= ~_IEC1_T4IE_MASK;
            break;
        case TIMER_5:
            IEC1 &= ~_IEC1_T5IE_MASK;
            break;
        }
    }
}

void timer_start(unsigned int timer_num)
{
    switch (timer_num) {
    case TIMER_2:
        T2CON |= _T2CON_TON_MASK;
        break;
    case TIMER_3:
        T3CON |= _T3CON_TON_MASK;
        break;
    case TIMER_4:
        T4CON |= _T4CON_TON_MASK;
        break;
    case TIMER_5:
        T5CON |= _T5CON_TON_MASK;
        break;
    }
}

void timer_stop(unsigned int timer_num)
{
    switch (timer_num) {
    case TIMER_2:
        T2CON &= ~_T2CON_TON_MASK;
        break;
    case TIMER_3:
        T3CON &= ~_T3CON_TON_MASK;
        break;
    case TIMER_4:
        T4CON &= ~_T4CON_TON_MASK;
        break;
    case TIMER_5:
        T5CON &= ~_T5CON_TON_MASK;
        break;
    }
}

uint32_t timer_get_period(unsigned int timer_num)
{
    uint64_t period = 0;
    unsigned int prescaler = 0;

    switch (timer_num) {
    case TIMER_2:
        prescaler = (T2CON & _T2CON_TCKPS_MASK) >> _T2CON_TCKPS_POSITION;
        period = PR2;
        break;
    case TIMER_3:
        prescaler = (T3CON & _T3CON_TCKPS_MASK) >> _T3CON_TCKPS_POSITION;
        period = PR3;
        break;
    case TIMER_4:
        prescaler = (T4CON & _T4CON_TCKPS_MASK) >> _T4CON_TCKPS_POSITION;
        period = PR4;
        break;
    case TIMER_5:
        prescaler = (T5CON & _T5CON_TCKPS_MASK) >> _T5CON_TCKPS_POSITION;
        period = PR5;
        break;
    }

    switch (prescaler) {
    case TIMER2_PRESCALER_1:
        break;
    case TIMER2_PRESCALER_8:
        period <<= 3;
        break;
    case TIMER2_PRESCALER_64:
        period <<= 6;
        break;
    case TIMER2_PRESCALER_256:
        period <<= 8;
        break;
    }

    /* Convert period to nanoseconds */
    period *= 1000;
    period *= 1000;
    period *= 1000;

    period /= (mcu_get_system_clock() >> 1);

    return period;
}

void timer_power_up(unsigned int timer_num)
{
    switch (timer_num) {
    case TIMER_2:
        PMD1 &= ~_PMD1_T2MD_MASK;
        break;
    case TIMER_3:
        PMD1 &= ~_PMD1_T3MD_MASK;
        break;
    case TIMER_4:
        PMD1 &= ~_PMD1_T4MD_MASK;
        break;
    case TIMER_5:
        PMD1 &= ~_PMD1_T5MD_MASK;
        break;
    }
}

void timer_power_down(unsigned int timer_num)
{
    switch (timer_num) {
    case TIMER_2:
        PMD1 |= _PMD1_T2MD_MASK;
        break;
    case TIMER_3:
        PMD1 |= _PMD1_T3MD_MASK;
        break;
    case TIMER_4:
        PMD1 |= _PMD1_T4MD_MASK;
        break;
    case TIMER_5:
        PMD1 |= _PMD1_T5MD_MASK;
        break;
    }
}

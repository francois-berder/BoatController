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
#include "periph/timer.h"
#include "periph_conf.h"

void timer_configure(unsigned int timer_num, uint8_t prescaler, uint16_t period)
{
    switch (timer_num) {
    case TIMER_2:
        T2CON = (prescaler << _T2CON_TCKPS_POSITION) & _T2CON_TCKPS_MASK;
        PR2 = period;
        TMR2 = 0;
        break;
    case TIMER_3:
        T3CON = (prescaler << _T3CON_TCKPS_POSITION) & _T3CON_TCKPS_MASK;
        PR3 = period;
        TMR3 = 0;
        break;
    case TIMER_4:
        T4CON = (prescaler << _T4CON_TCKPS_POSITION) & _T4CON_TCKPS_MASK;
        PR4 = period;
        TMR4 = 0;
        break;
    case TIMER_5:
        T5CON = (prescaler << _T5CON_TCKPS_POSITION) & _T5CON_TCKPS_MASK;
        PR5 = period;
        TMR5 = 0;
        break;
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

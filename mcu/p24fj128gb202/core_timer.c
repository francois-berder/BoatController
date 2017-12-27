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

#include "core_timer.h"

/* By default, let's use timer 1 */
#ifndef CORE_TIMER_NUM
#define CORE_TIMER_NUM  (1)
#endif

static uint32_t ticks;

#if CORE_TIMER_NUM == 1
void timer1_callback(void)
#elif CORE_TIMER_NUM == 2
void timer2_callback(void)
#elif CORE_TIMER_NUM == 3
void timer3_callback(void)
#elif CORE_TIMER_NUM == 4
void timer4_callback(void)
#elif CORE_TIMER_NUM == 5
void timer5_callback(void)
#endif
{
    ++ticks;
}

uint32_t core_timer_get_ticks(void)
{
    return ticks;
}

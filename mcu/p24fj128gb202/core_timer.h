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

#ifndef __MCU_CORE_TIMER_H__
#define __MCU_CORE_TIMER_H__

#include <stdint.h>

#ifndef TICKS_PER_SEC
#define TICKS_PER_SEC 1000
#endif

/**
 * @brief Return the number of ticks
 *
 * Since the core timer is dependent on timer 1,
 * it only works if timer 1 has been configured and
 * enabled.
 * Tick counter is incremented at each timer 1 interrupt
 *
 * @return Ticks
 */
uint32_t core_timer_get_ticks(void);

#endif

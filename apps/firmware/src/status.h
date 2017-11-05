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

#ifndef __STATUS_H__
#define __STATUS_H__

enum STATUS_MODE {
    STATUS_OFF,             /**< Always OFF */
    STATUS_ON,              /**< Always ON */
    STATUS_FLASH,           /**< ON: 100ms, OFF: 4.9s */
    STATUS_FAST_BLINK,      /**< ON: 100ms, OFF: 100ms */
    STATUS_ONE_PER_2SEC,    /**< 1 blink every 2 seconds */
    STATUS_TWO_PER_2SEC,    /**< 2 blinks every 2 seconds */
    STATUS_THREE_PER_2SEC,  /**< 3 blinks every 2 seconds */
};

/**
 * @brief Configure the status LED
 */
void status_configure(void);

/**
 * @brief Set the mode of the status LED
 *
 * @param[in] mode
 */
void status_set_mode(enum STATUS_MODE mode);

#endif

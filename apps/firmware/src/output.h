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

#ifndef __OUTPUT_H__
#define __OUTPUT_H__

#include <stdint.h>

struct output_frame_t {
    uint16_t left_rudder;
    uint16_t right_rudder;
    uint16_t left_motor;
    uint16_t right_motor;
};

/**
 * @brief Configure output module
 */
void output_configure(void);

/**
 * @brief Set output frame
 */
void output_set_frame(struct output_frame_t frame);

#endif

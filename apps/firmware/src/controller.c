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

#include "controller.h"
#include "mcu.h"
#include "output.h"
#include "radio.h"

void controller_run(struct board_config_t config)
{
    (void)config;

    while (1) {
        if (radio_has_frame()) {
            struct radio_frame_t radio_frame;
            struct output_frame_t output_frame;

            radio_frame = radio_get_frame();
            output_frame.left_rudder = radio_frame.dir;
            output_frame.right_rudder = radio_frame.dir;
            output_frame.left_motor = radio_frame.speed;
            output_frame.right_motor = radio_frame.speed;

            output_set_frame(output_frame);
        }

        /* Wait 5ms */
        mcu_delay(5);
    }
}

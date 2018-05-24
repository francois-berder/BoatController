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

#include <stdbool.h>
#include <stdint.h>
#include "radio_filter.h"

#define NEUTRAL_POS                 (6000)
#define LOST_CONTACT_THRESHOLD      (6)

static uint16_t invalid_radio_frame_bitmap;
static struct radio_frame_t last_valid_frame = {NEUTRAL_POS, NEUTRAL_POS, 0};
static bool radio_contact_lost = false;

void __attribute__((weak)) radio_contact_status_callback(bool lost)
{
    (void)lost;
}

/**
 * @brief Check if a radio frame is valid (= usable)
 *
 * @param[in] in
 * @retval 0 Radio frame is invalid
 * @retval 1 Radio frame is valid
 */
static int check_valid_radio_frame(struct radio_frame_t *in)
{
    static int16_t last_dir = NEUTRAL_POS;
    int valid = 1;
    int delta = last_dir;
    delta -= in->dir;

    if (delta < -1000 || delta > 1000
    ||  in->dir < 3800 || in->dir > 8200
    ||  in->speed < 3800 || in->speed > 8200)
        valid = 0;

    last_dir = in->dir;

    return valid;
}

/**
 * @brief Check if we lost radio contact with emitter
 *
 * This function counts the number of 1s in invalid_radio_frame_bitmap.
 * If there are more than LOST_CONTACT_THRESHOLD invalid frames,
 * we consider that we lost contact with the emitter.
 *
 * The algorithm comes from:
 * https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
 *
 * @retval 0 Radio contact is still good
 * @retval 1 Radio contact is lost
 */
static int is_radio_contact_lost(void)
{
    unsigned int v = invalid_radio_frame_bitmap;
    unsigned int c;

    for (c = 0; v; c++)
        v &= v - 1;

    return c >= LOST_CONTACT_THRESHOLD;
}

int radio_filter(struct radio_frame_t *out, struct radio_frame_t *in)
{
    int valid = check_valid_radio_frame(in);

    invalid_radio_frame_bitmap <<= 1;
    if (!valid)
        invalid_radio_frame_bitmap |= 0x1;

    if (radio_contact_lost != is_radio_contact_lost()) {
        radio_contact_lost = !radio_contact_lost;
        radio_contact_status_callback(radio_contact_lost);
    }

    if (valid) {
        /* IIR filtering */
        out->dir = (last_valid_frame.dir + in->dir) >> 1;
        out->speed = (last_valid_frame.speed + in->speed) >> 1;

        last_valid_frame = *in;
    }

    return valid;
}

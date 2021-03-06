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
#include "core_timer.h"
#include "mcu.h"
#include "periph/ic.h"
#include "periph_conf.h"
#include "radio.h"

#define DIR_RECEIVED_FLAG       (1)
#define SPEED_RECEIVED_FLAG     (2)
#define ALL_RECEIVED_FLAG       (DIR_RECEIVED_FLAG | SPEED_RECEIVED_FLAG)

#define RADIO_FIFO_LEN          (4) /* must be a power of 2 */

static volatile struct radio_frame_t frames[RADIO_FIFO_LEN];
static volatile unsigned int available_frame;
static volatile unsigned int current_frame;
static volatile uint8_t flags;

static volatile uint16_t dir_buf_prev;
static volatile uint16_t speed_buf_prev;

static void push_frame(void)
{
    unsigned int tmp = (current_frame + 1) & (RADIO_FIFO_LEN - 1);

    /* If we have a buffer overflow, drop this frame */
    if (tmp != available_frame) {
        frames[current_frame].t = core_timer_get_ticks();
        current_frame = tmp;
    }

    flags = 0;
}

void ic1_callback(void)
{
    unsigned int dir_buf = ic_get_data(IC_1);
    unsigned int dir = dir_buf - dir_buf_prev;
    dir_buf_prev = dir_buf;

    if (dir < 20000) {
        flags |= DIR_RECEIVED_FLAG;
        frames[current_frame].dir = dir;
    }

    if (flags == ALL_RECEIVED_FLAG)
        push_frame();
}

void ic2_callback(void)
{
    unsigned int speed_buf = ic_get_data(IC_2);
    unsigned int speed = speed_buf - speed_buf_prev;
    speed_buf_prev = speed_buf;

    if (speed < 20000) {
        flags |= SPEED_RECEIVED_FLAG;
        frames[current_frame].speed = speed;
    }

    if (flags == ALL_RECEIVED_FLAG)
        push_frame();
}

void radio_configure(void)
{
    ic_power_up(IC_1);
    ic_power_up(IC_2);
    ic_configure(IC_1, IC_EDGE_DETECT, 1);
    ic_configure(IC_2, IC_EDGE_DETECT, 1);

    current_frame = 0;
    available_frame = 0;
    flags = 0;
    dir_buf_prev = 0;
    speed_buf_prev = 0;

}

void radio_enable(void)
{
    ic_enable(IC_1);
    ic_enable(IC_2);
}

void radio_disable(void)
{
    ic_disable(IC_1);
    ic_disable(IC_2);
}

uint8_t radio_has_frame(void)
{
    unsigned int tmp;
    unsigned int ctx;

    ctx = mcu_save_context();
    tmp = current_frame;
    mcu_restore_context(ctx);

    return tmp != available_frame;
}

int radio_get_frame(struct radio_frame_t *frame)
{
    unsigned int index = available_frame;
    int ret = 0;
    unsigned int ctx;

    ctx = mcu_save_context();
    if (available_frame != current_frame) {
        *frame = frames[index];
        available_frame = (available_frame + 1) & (RADIO_FIFO_LEN - 1);
        ret = 1;
    }
    mcu_restore_context(ctx);

    return ret;
}

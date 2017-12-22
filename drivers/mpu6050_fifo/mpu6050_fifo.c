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

#include "mcu.h"
#include "mpu6050_fifo/mpu6050_fifo.h"
#include "periph/timer.h"
#include "periph_conf.h"

#ifndef IMU_FIFO_DEPTH
#define IMU_FIFO_DEPTH          (32)
#endif

static struct mpu6050_dev_t dev;
static volatile struct mpu6050_sample_t samples[IMU_FIFO_DEPTH];
static volatile unsigned int sample_count;
static volatile unsigned int fifo_start_index;

void mpu6050_fifo_init(struct mpu6050_dev_t _dev)
{
    dev = _dev;
    sample_count = 0;
    fifo_start_index = 0;
}

void mpu6050_fifo_start(void)
{
    timer_power_up(TIMER_5);
    timer_configure(TIMER_5, TIMER5_PRESCALER_64, 1250, 1);
    timer_start(TIMER_5);
}

void mpu6050_fifo_stop(void)
{
    timer_stop(TIMER_5);
    timer_power_down(TIMER_5);
}

void mpu6050_fifo_clear_samples(void)
{
    mcu_disable_interrupts();
    fifo_start_index = 0;
    sample_count = 0;
    mcu_enable_interrupts();
}

unsigned int mpu6050_fifo_get_sample_count(void)
{
    unsigned int tmp;

    mcu_disable_interrupts();
    tmp = sample_count;
    mcu_enable_interrupts();

    return tmp;
}

int mpu6050_fifo_get_sample(struct mpu6050_sample_t *sample)
{
    unsigned int index;
    unsigned int ret = 0;

    mcu_disable_interrupts();
    if (sample_count > 0) {
        index = (fifo_start_index + sample_count) & (IMU_FIFO_DEPTH - 1);
        *sample = samples[index];
        ++fifo_start_index;
        fifo_start_index &= (IMU_FIFO_DEPTH - 1);
        --sample_count;
        ret = 1;
    }
    mcu_enable_interrupts();

    return ret;
}

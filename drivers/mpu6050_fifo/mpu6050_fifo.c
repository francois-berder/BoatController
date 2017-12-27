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

#ifndef MPU6050_FIFO_DEPTH
#define MPU6050_FIFO_DEPTH          (32)
#endif

/* By default, use timer 5 */
#ifndef MPU6050_FIFO_TIMER_NUM
#define MPU6050_FIFO_TIMER_NUM      (5)
#endif

#define MPU6050_FIFO_ACC_ENABLED    (0x1)
#define MPU6050_FIFO_GYRO_ENABLED   (0x2)

static struct mpu6050_dev_t dev;
static volatile unsigned int features;
static struct mpu6050_sample_t samples[MPU6050_FIFO_DEPTH];
static volatile unsigned int sample_count;
static volatile unsigned int fifo_start_index;

#if MPU6050_FIFO_TIMER_NUM == 2
void timer2_callback(void)
#elif MPU6050_FIFO_TIMER_NUM == 3
void timer3_callback(void)
#elif MPU6050_FIFO_TIMER_NUM == 4
void timer4_callback(void)
#elif MPU6050_FIFO_TIMER_NUM == 5
void timer5_callback(void)
#endif
{
    unsigned int index;
    int ret;

    if (sample_count >= MPU6050_FIFO_DEPTH)
        return;

    index = (fifo_start_index + sample_count) & (MPU6050_FIFO_DEPTH - 1);
    if (features & (MPU6050_FIFO_ACC_ENABLED | MPU6050_FIFO_GYRO_ENABLED))
        ret = mpu6050_get_acc_gyro(&dev, &samples[index]);
    else if (features & MPU6050_FIFO_ACC_ENABLED)
        ret = mpu6050_get_acc(&dev, &samples[index]);
    else if (features & MPU6050_FIFO_GYRO_ENABLED)
        ret = mpu6050_get_gyro(&dev, &samples[index]);
    else
        return;

    if (ret == 0)
        ++sample_count;
    else                    /* Stop driver if something went wrong */
        mpu6050_fifo_stop();
}

void mpu6050_fifo_init(struct mpu6050_dev_t _dev, unsigned int use_acc, unsigned int use_gyro)
{
    dev = _dev;
    sample_count = 0;
    fifo_start_index = 0;
    features = 0;

    if (use_acc)
        features |= MPU6050_FIFO_ACC_ENABLED;
    if (use_gyro)
        features |= MPU6050_FIFO_GYRO_ENABLED;
}

void mpu6050_fifo_start(void)
{
    timer_power_up(MPU6050_FIFO_TIMER_NUM);
    timer_configure(MPU6050_FIFO_TIMER_NUM, TIMER5_PRESCALER_64, 1250, 1);
    timer_start(MPU6050_FIFO_TIMER_NUM);
}

void mpu6050_fifo_stop(void)
{
    timer_stop(MPU6050_FIFO_TIMER_NUM);
    timer_power_down(MPU6050_FIFO_TIMER_NUM);
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
        index = (fifo_start_index + sample_count) & (MPU6050_FIFO_DEPTH - 1);
        *sample = samples[index];
        ++fifo_start_index;
        fifo_start_index &= (MPU6050_FIFO_DEPTH - 1);
        --sample_count;
        ret = 1;
    }
    mcu_enable_interrupts();

    return ret;
}

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

#include <stdio.h>
#include "controller.h"
#include "fat16/fat16.h"
#include "mpu6050_fifo/mpu6050_fifo.h"
#include "mcu.h"
#include "output.h"
#include "periph/crypto.h"
#include "radio.h"
#include "sdcard_cache/sdcard_cache.h"

#ifndef CONTROLLER_PERIOD_MS
#define CONTROLLER_PERIOD_MS    (4)
#endif

static int io_fd = -1;
static int imu_fd = -1;

static void open_log_files(void)
{
    char dirname[9];
    char filepath[32];
    unsigned int i;

    /* Create directory name: 8 random letters */
    crypto_power_up();
    crypto_enable();
    crypto_get_random(dirname, 8);
    crypto_disable();
    crypto_power_down();
    for (i = 0; i < 8; ++i)
            dirname[i] = 'A' + (dirname[i] & 0xF);
    dirname[8] = '\0';

    /* Create directory */
    if (fat16_mkdir(dirname) < 0) {
        printf("Failed to create directory %s\n", dirname);
        return;
    }

    /* Open file IO.TXT */
    sprintf(filepath, "%s/%s", dirname, "IO.TXT");
    io_fd = fat16_open(filepath, 'w');
    if (io_fd < 0) {
        printf("Cannot log I/O to file %s\n", filepath);
    } else {
        printf("Logging I/O to file %s\n", filepath);
    }

    /* Open file MPU6050.TXT */
    sprintf(filepath, "%s/%s", dirname, "MPU6050.TXT");
    imu_fd = fat16_open(filepath, 'w');
    if (imu_fd < 0) {
        printf("Cannot log IMU data to file %s\n", filepath);
    } else {
        printf("Logging IMU data to file %s\n", filepath);
    }

    /*
     * Flush cache now to ensure that file will exist on the SD card
     * even if no radio frames are found later.
     */
    sdcard_cache_flush();
}

static void log_io(struct radio_frame_t rf, struct output_frame_t of)
{
    char buffer[128];
    int ret = sprintf(buffer, "%u, %u, %u, %u, %u, %u\n",
                      rf.dir, rf.speed,
                      of.left_rudder, of.right_rudder, of.left_motor, of.right_motor);
    if (ret >= 0)
        fat16_write(io_fd, buffer, ret);
}

static void log_imu(struct mpu6050_sample_t s)
{
    char buffer[128];
    int ret = sprintf(buffer, "%d, %d, %d, %d, %d, %d\n",
                      s.accel.x, s.accel.y, s.accel.z,
                      s.gyro.x, s.gyro.y, s.gyro.z);
    if (ret >= 0)
        fat16_write(imu_fd, buffer, ret);
}

void controller_run(struct board_config_t config, struct mpu6050_dev_t mpu6050_dev)
{
    unsigned int counter = 0;

    if (config.sdcard_enabled) {
        open_log_files();
    }

    if (config.mpu6050_enabled) {
        mpu6050_fifo_init(mpu6050_dev);
        mpu6050_fifo_start();
    }

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

           if (io_fd >= 0)
                log_io(radio_frame, output_frame);
        }

        /* Record all samples available from MPU6050 FIFO */
        {
            struct mpu6050_sample_t s;
            while (mpu6050_fifo_get_sample(&s) == 1) {
                if (imu_fd >= 0)
                    log_imu(s);
            }
        }

        mcu_delay(CONTROLLER_PERIOD_MS);

        /* Ensure that log file is periodically saved to SD card */
        if (io_fd >= 0 || imu_fd >= 0) {
            counter += 5;
            if (counter >= 2000) {
                counter = 0;
                sdcard_cache_flush();
            }
        }
    }
}

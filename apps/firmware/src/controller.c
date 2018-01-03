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
#include "periph/spi.h"
#include "periph_conf.h"
#include "radio.h"
#include "sdcard_cache/sdcard_cache.h"

#ifndef CONTROLLER_PERIOD_MS
#define CONTROLLER_PERIOD_MS    (4)
#endif

#define NEUTRAL_POS     (6000)

struct board_config_t config;

/* File descriptors */
static int radio_fd = -1;
static int output_fd = -1;
static int mpu6050_fd = -1;

static int16_t angular_speed_target = NEUTRAL_POS;
static int16_t speed_target = NEUTRAL_POS;

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

    /* Open file RADIO.TXT */
    sprintf(filepath, "/%s/%s", dirname, "RADIO.TXT");
    radio_fd = fat16_open(filepath, 'w');
    if (radio_fd < 0)
        printf("Cannot log I/O to file %s\n", filepath);
    else
        printf("Logging I/O to file %s\n", filepath);

    /* Open file OUTPUT.TXT */
    sprintf(filepath, "/%s/%s", dirname, "OUTPUT.TXT");
    output_fd = fat16_open(filepath, 'w');
    if (output_fd < 0)
        printf("Cannot log I/O to file %s\n", filepath);
    else
        printf("Logging I/O to file %s\n", filepath);

    /* Open file MPU6050.TXT */
    sprintf(filepath, "/%s/%s", dirname, "MPU6050.TXT");
    mpu6050_fd = fat16_open(filepath, 'w');
    if (mpu6050_fd < 0)
        printf("Cannot log IMU data to file %s\n", filepath);
    else
        printf("Logging IMU data to file %s\n", filepath);

    /*
     * Flush cache now to ensure that file will exist on the SD card
     * even if no radio frames are found later.
     */
    sdcard_cache_flush();
}

static void log_radio_frame(struct radio_frame_t rf)
{
    char buffer[64];
    int ret = sprintf(buffer, "%u, %u\n",
                      rf.dir, rf.speed);
    if (ret >= 0)
        fat16_write(radio_fd, buffer, ret);
}

static void log_output_frame(struct output_frame_t of)
{
    char buffer[128];
    int ret = sprintf(buffer, "%u, %u, %u, %u\n",
                      of.left_rudder, of.right_rudder, of.left_motor, of.right_motor);
    if (ret >= 0)
        fat16_write(output_fd, buffer, ret);
}

static void log_mpu6050_frame(struct mpu6050_sample_t s)
{
    char buffer[128];
    int ret = sprintf(buffer, "%d, %d, %d, %d, %d, %d\n",
                      s.accel.x, s.accel.y, s.accel.z,
                      s.gyro.x, s.gyro.y, s.gyro.z);
    if (ret >= 0)
        fat16_write(mpu6050_fd, buffer, ret);
}

void controller_init(struct board_config_t _config)
{
    config = _config;

    output_configure();
    output_enable();

    radio_configure();
    radio_enable();

    if (config.sdcard_enabled)
        open_log_files();

    if (config.mpu6050_enabled) {
        mpu6050_fifo_init(config.mpu6050_dev, 1, 1);
        mpu6050_fifo_start();
    }
}

void controller_run(void)
{
    unsigned int counter = 0;

    while (1) {
        unsigned int update_output_frame = 0;

        /* Check if mpu6050 FIFO stopped running */
        if (config.mpu6050_enabled && !mpu6050_fifo_is_running()) {
            config.mpu6050_enabled = 0;
            printf("MPU6050 FIFO stopped running.\n");
        }

        /* Check if the SD card is working correctly */
        if (config.sdcard_enabled) {
            struct sdcard_cache_stats_t stats = sdcard_cache_get_stats();
            if (stats.write_error || stats.read_error) {
                fat16_close(radio_fd);
                fat16_close(mpu6050_fd);
                fat16_close(output_fd);

                radio_fd = -1;
                mpu6050_fd = -1;
                output_fd = -1;

                /* Disable SPI module since we won't use it anymore */
                spi_disable(config.sdcard_dev.spi_num);
                spi_power_down(config.sdcard_dev.spi_num);

                config.sdcard_enabled = 0;
                printf("Errors occured while using SD card.\nStop using SD card.\n");
            }
        }

        /* Process all frames available from the radio */
        {
            struct radio_frame_t frame;
            while (radio_get_frame(&frame)) {
                angular_speed_target = frame.dir;
                speed_target = frame.speed;
                angular_speed_target -= NEUTRAL_POS;
                speed_target -= NEUTRAL_POS;

                update_output_frame = 1;

                if (radio_fd >= 0)
                    log_radio_frame(frame);
            }
        }

        /* Process all samples available from MPU6050 FIFO */
        if (config.mpu6050_enabled) {
            struct mpu6050_sample_t s;
            while (mpu6050_fifo_get_sample(&s) == 1) {
                if (mpu6050_fd >= 0)
                    log_mpu6050_frame(s);
            }
        }

        if (update_output_frame) {
            struct output_frame_t output_frame;
            output_frame.left_rudder = angular_speed_target + NEUTRAL_POS;
            output_frame.right_rudder = angular_speed_target + NEUTRAL_POS;
            output_frame.left_motor = speed_target + NEUTRAL_POS;
            output_frame.right_motor = speed_target + NEUTRAL_POS;
            output_set_frame(output_frame);

            if (output_fd >= 0)
                log_output_frame(output_frame);
        }

        /* Ensure that logs are periodically saved to SD card */
        if (radio_fd >= 0 || output_fd >= 0 || mpu6050_fd >= 0) {
            if (counter == 0)
                sdcard_cache_flush();
        }

        mcu_delay(CONTROLLER_PERIOD_MS);

        counter += CONTROLLER_PERIOD_MS;
        if (counter >= 2000)
            counter = 0;
    }
}

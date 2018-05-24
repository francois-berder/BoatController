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
#include "core_timer.h"
#include "log.h"
#include "mpu6050_fifo/mpu6050_fifo.h"
#include "mcu.h"
#include "output.h"
#include "periph/spi.h"
#include "periph_conf.h"
#include "radio.h"
#include "sdcard_cache/sdcard_cache.h"

#ifndef CONTROLLER_PERIOD_MS
#define CONTROLLER_PERIOD_MS    (4)
#endif

#define NEUTRAL_POS     (6000)

static struct board_config_t config;

/*
 * Compute delta between radio direction frames
 * It is used to check if radio contact is good
 */
static int16_t delta_radio_buffer[16];
static int16_t last_dir = NEUTRAL_POS;

static int16_t angular_speed_target = NEUTRAL_POS;
static int16_t speed_target = NEUTRAL_POS;


void controller_init(struct board_config_t _config)
{
    config = _config;

    output_configure();
    output_enable();

    radio_configure();
    radio_enable();

    if (config.sdcard_enabled)
        log_init();

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
        if (config.sdcard_enabled && log_is_running()) {
            struct sdcard_cache_stats_t stats = sdcard_cache_get_stats();
            if (stats.write_error || stats.read_error) {

                log_stop();

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
                unsigned int i;
                unsigned int bad_delta_count = 0;

                update_output_frame = 1;


                /* Ignore the 2 least significant bits */
                frame.dir &= ~0x3;
                frame.speed &= ~0x3;

                /* Add to buffer */
                for (i = 15; i > 0; --i)
                    delta_radio_buffer[i] = delta_radio_buffer[i - 1];
                delta_radio_buffer[0] = ((int16_t)frame.dir) - last_dir;
                last_dir = frame.dir;

                /* Find out if radio contact is loss */
                for (i = 0; i < 16; ++i) {
                    if (delta_radio_buffer[i] > 1000 || delta_radio_buffer[i] < -1000)
                        ++bad_delta_count;
                }

                /* If we lost radio contact, let's stop the boat */
                if (bad_delta_count >= 6) {
                    angular_speed_target = 0;
                    speed_target = 0;
                    continue;
                }

                /* IIR filter */
                if (frame.dir >= 3800 && frame.dir <= 8200) {
                    angular_speed_target += NEUTRAL_POS;
                    angular_speed_target = (angular_speed_target + frame.dir) >> 1;
                    angular_speed_target -= NEUTRAL_POS;
                }

                if (frame.speed >= 3800 && frame.speed <= 8200) {
                    speed_target += NEUTRAL_POS;
                    speed_target = (speed_target + frame.speed) >> 1;
                    speed_target -= NEUTRAL_POS;
                }

                log_radio_frame(frame);
            }
        }

        /* Process all samples available from MPU6050 FIFO */
        if (config.mpu6050_enabled) {
            struct mpu6050_sample_t s;
            while (mpu6050_fifo_get_sample(&s) == 1) {
                log_mpu6050_frame(s);
            }
        }

        if (update_output_frame) {
            struct output_frame_t output_frame;
            uint32_t t = core_timer_get_ticks();
            output_frame.left_rudder = angular_speed_target + NEUTRAL_POS;
            output_frame.right_rudder = angular_speed_target + NEUTRAL_POS;
            output_frame.left_motor = speed_target + NEUTRAL_POS;
            output_frame.right_motor = speed_target + NEUTRAL_POS;
            output_set_frame(output_frame);

            log_output_frame(t, output_frame);
        }

        /* Ensure that logs are periodically saved to SD card */
        if (counter == 0)
            log_flush();

        mcu_delay(CONTROLLER_PERIOD_MS);

        counter += CONTROLLER_PERIOD_MS;
        if (counter >= 2000)
            counter = 0;
    }
}

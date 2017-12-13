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
#include "block_storage.h"
#include "controller.h"
#include "fat16/fat16.h"
#include "mcu.h"
#include "output.h"
#include "periph/crypto.h"
#include "radio.h"

static int fd = -1;

static void open_log_file(void)
{
    char filename[13];
    unsigned int i;

    crypto_power_up();
    crypto_enable();
    crypto_get_random(filename, 8);
    crypto_disable();
    crypto_power_down();

    for (i = 0; i < 8; ++i)
        filename[i] = 'A' + (filename[i] & 0xF);

    filename[8] = '.';
    filename[9] = 'T';
    filename[10] = 'X';
    filename[11] = 'T';
    filename[12] = '\0';

    fd = fat16_open(filename, 'w');
    if (fd < 0) {
        printf("Cannot log to file %s\n", filename);
    } else {
        printf("Starting logging to file %s\n", filename);
    }

    /*
        * Flush cache now to ensure that file will exist on the SD card
        * even if no radio frames are found later.
        */
    block_storage_flush();
}

static void log_frame(struct radio_frame_t rf, struct output_frame_t of)
{
    char buffer[128];
    int ret = sprintf(buffer, "%u, %u, %u, %u, %u, %u\n",
                      rf.dir, rf.speed,
                      of.left_rudder, of.right_rudder, of.left_motor, of.right_motor);
    if (ret >= 0)
        fat16_write(fd, buffer, ret);
}

void controller_run(struct board_config_t config)
{
    unsigned int counter = 0;

    if (config.sdcard_enabled) {
        open_log_file();
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

           if (fd >= 0)
                log_frame(radio_frame, output_frame);
        }

        /* Wait 5ms */
        mcu_delay(5);

        /* Ensure that log file is periodically saved to SD card */
        if (fd >= 0) {
            counter += 5;
            if (counter >= 2000) {
                counter = 0;
                block_storage_flush();
            }
        }
    }
}

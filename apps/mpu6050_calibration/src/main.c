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

/* CONFIG1 */
#pragma config WDTPS = PS1024
#pragma config FWPSA = PR32
#pragma config WINDIS = OFF
#pragma config FWDTEN = SWON
#pragma config ICS = PGx2
#pragma config LPCFG = OFF
#pragma config GWRP = OFF
#pragma config GCP = OFF
#pragma config JTAGEN = OFF

/* CONFIG2 */
#pragma config POSCMD = NONE
#pragma config WDTCLK = LPRC
#pragma config OSCIOFCN = ON
#pragma config FCKSM = CSDCMD
#pragma config FNOSC = FRC
#pragma config ALTRB6 = RETAIN
#pragma config ALTCMPI = CxINC_RB
#pragma config WDTCMX = WDTCLK
#pragma config IESO = OFF

/* CONFIG3 */
#pragma config WPFP = WPFP127
#pragma config SOSCSEL = OFF
#pragma config WDTWIN = PS25_0
#pragma config PLLSS = PLL_FRC
#pragma config BOREN = ON
#pragma config WPDIS = WPDIS
#pragma config WPCFG = WPCFGDIS
#pragma config WPEND = WPENDMEM

/* CONFIG4 */
#pragma config DSWDTPS = DSWDTPS1F
#pragma config DSWDTOSC = LPRC
#pragma config DSBOREN = ON
#pragma config DSWDTEN = ON
#pragma config DSSWEN = ON
#pragma config PLLDIV = PLL4X
#pragma config I2C1SEL = DISABLE
#pragma config IOL1WAY = OFF

#include <stdio.h>
#include <string.h>
#include <xc.h>
#include "block_storage.h"
#include "fat16/fat16.h"
#include "mbr.h"
#include "mcu.h"
#include "mpu6050.h"
#include "periph/gpio.h"
#include "periph/timer1.h"
#include "periph/uart.h"
#include "periph_conf.h"
#include "sdcard.h"

#define UART_TX_PIN     (GPIO_PIN(PORT_B, 15))
#define UART_RX_PIN     (GPIO_PIN(PORT_B, 14))

#ifndef SAMPLE_COUNT
#define SAMPLE_COUNT    (128U)
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static const char *welcome_msg = "Boat Controller firmware " FIRMWARE_VERSION
                                 " - " __DATE__ " " __TIME__
                                 "\n";

static struct storage_dev_t dev = {
    block_storage_read,
    block_storage_read_byte,
    block_storage_write,
    block_storage_seek
};

static void stop(const char *reason)
{
    printf("\n%s\n", reason);
    while (1)
        ;
}

int main(void)
{
    unsigned int i;
    struct partition_info_t p;

    mcu_set_system_clock(8000000LU);

    /* Configure unused I/O as output low */
    gpio_init_out(GPIO_PIN(PORT_A, 0), 0);
    gpio_init_out(GPIO_PIN(PORT_A, 1), 0);
    gpio_init_out(GPIO_PIN(PORT_A, 4), 0);

    /* Power down all peripherals */
    PMD1 = 0xFFFF;
    PMD2 = 0xFFFF;
    PMD3 = 0xFFFF;
    PMD4 = 0xFFFF;
    PMD5 = 0xFFFF;
    PMD6 = 0xFFFF;
    PMD7 = 0xFFFF;
    PMD8 = 0xFFFF;

    /* Configure timer 1 to enable mcu_delay, 1 tick = 1ms */
    timer1_power_up();
    timer1_configure(TIMER1_PRESCALER_1, 4000, 1);
    timer1_start();

    /* Configure uart */
    gpio_init_out(UART_TX_PIN, 1);
    gpio_init_in(UART_RX_PIN);
    RPOR7bits.RP15R = 0x03;
    RPINR18bits.U1RXR = 0x0E;
    uart_power_up(UART_1);
    uart_configure(UART_1, UART_BD_9600);
    uart_enable(UART_1);

    printf(welcome_msg);

    /* Configure MPU6050 device */
    printf("Configuring MPU6050 device...");
    if (!mpu6050_init())
        printf("done\n");
    else
        stop("Failed to initialise MPU6050");

    /* Configure SD card */
    printf("Configuring SD card...");
    if (!sdcard_init())
        printf("done\n");
    else
        stop("Failed to configure SD card");

    printf("Configuring block storage...");
    block_storage_init();
    printf("done\n");

    printf("Reading MBR...");
    mbr_read_partition_table();
    printf("done\n");

    printf("Looking for a FAT16 partition...\n");
    for (i = 0; i < PARTITION_ENTRY_COUNT; ++i) {
        p = mbr_get_partition_info(i);
        if ((p.status == BOOTABLE_PARTITION || p.status == INACTIVE_PARTITION)
        &&  p.type == FAT16_PARTITION_TYPE) {
            uint32_t size_100kB = p.size / 100000; /* size in 100kB unit */
            printf("Found FAT16 partition at entry %u\n", i);
            printf("\tstart_sector: %lu\n", p.start_sector);
            printf("\tsize: %lu bytes (%lu.%lu MB)\n", p.size, size_100kB / 10, size_100kB % 10);
            break;
        }
    }
    if (i == PARTITION_ENTRY_COUNT)
        stop("Failed to found a FAT16 partition");

    {
        uint32_t partition_offset = p.start_sector;
        partition_offset <<= 9;         /* sector size is 512 bytes */
        fat16_init(dev, partition_offset);
    }

    printf("Initialisation finished\n");
    printf("Ensure that the MPU6050 is immobile, on a flat surface\n");

    while (1) {
        struct mpu6050_sample_t samples[SAMPLE_COUNT];
        unsigned int j;
        uint64_t avg[6] = {0, 0, 0, 0, 0, 0};
        int ax, ay, az, gx, gy, gz;

        printf("Retrieving %u samples", SAMPLE_COUNT);
        mpu6050_clear_samples();
        for (j = 0; j < SAMPLE_COUNT; ++j) {
            printf(".");
            while (mpu6050_get_sample_count() == 0)
                ;
            mpu6050_get_sample(&samples[j]);
        }
        printf("\n");

        /* Compute average for each channel */
        for (j = 0; j < SAMPLE_COUNT; ++j) {
            avg[0] += samples[j].accel.x;
            avg[1] += samples[j].accel.y;
            avg[2] += samples[j].accel.z;
            avg[3] += samples[j].gyro.x;
            avg[4] += samples[j].gyro.y;
            avg[5] += samples[j].gyro.z;
        }
        avg[0] >>= 7;
        avg[1] >>= 7;
        avg[2] >>= 7;
        avg[3] >>= 7;
        avg[4] >>= 7;
        avg[5] >>= 7;

        ax = avg[0];
        ay = avg[1];
        az = avg[2];
        gx = avg[3];
        gy = avg[4];
        gz = avg[5];
        printf("avg accel (%d, %d, %d)\n", ax, ay, az);
        printf("avg gyro (%d, %d, %d)\n", gx, gy, gz);

        /* Saving to sd card */
        {
            char buffer[128];
            unsigned int len;
            int ret;

            printf("Saving to SD card...");
            int fd = fat16_open("/CALIB.TXT", 'w');
            if (fd < 0) {
                printf("failed\n");
                continue;
            }

            sprintf(buffer, "%d, %d, %d, %d, %d, %d\n", ax, ay, az, gx, gy, gz);
            len = strlen(buffer);
            ret = fat16_write(fd, buffer, len);
            if (ret < 0 || (unsigned int)ret != len) {
                printf("failed\n");
                continue;
            }

            fat16_close(fd);
            block_storage_flush();
            printf("done\n");
        }

        mcu_delay(1000);
    }

    return 0;
}

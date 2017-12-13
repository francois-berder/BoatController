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

struct calibration_data {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } offset;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } coeff;
};
static struct calibration_data cdata;

static void stop(const char *reason)
{
    printf("\n%s\n", reason);
    while (1)
        ;
}

static void wait_for_user(void)
{
    char c;
    do {
        char buffer[128];
        struct mpu6050_sample_t raw_sample;

        mpu6050_clear_samples();
        while (mpu6050_get_sample_count() == 0)
            ;
        mpu6050_get_sample(&raw_sample);
        sprintf(buffer,
                "\rPress enter to continue, (%d, %d, %d)\n",
                raw_sample.accel.x, raw_sample.accel.y, raw_sample.accel.z);
        printf("%s", buffer);
        uart_read(UART_1, &c, 1);
    } while (c != '\n');
}

static void compute_avg(int16_t *ax, int16_t *ay, int16_t *az)
{
    struct mpu6050_sample_t samples[SAMPLE_COUNT];
    unsigned int i;
    uint64_t avg[3] = {0, 0, 0};

    printf("Retrieving %u samples", SAMPLE_COUNT);
    mpu6050_clear_samples();
    for (i = 0; i < SAMPLE_COUNT; ++i) {
        printf(".");
        while (mpu6050_get_sample_count() == 0)
            ;
        mpu6050_get_sample(&samples[i]);
    }
    printf("\n");

    /* Compute average for each component */
    for (i = 0; i < SAMPLE_COUNT; ++i) {
        avg[0] += samples[i].accel.x;
        avg[1] += samples[i].accel.y;
        avg[2] += samples[i].accel.z;
    }
    avg[0] >>= 7;
    avg[1] >>= 7;
    avg[2] >>= 7;

    *ax = avg[0];
    *ay = avg[1];
    *az = avg[2];
    printf("avg accel (%d, %d, %d)\n", *ax, *ay, *az);
}

static void compute_calibration_data(int16_t *raw_accel)
{
    int32_t x, x2;
    int32_t y, y2;
    int32_t z, z2;

    x = raw_accel[0];
    x2 = raw_accel[6];
    cdata.coeff.x = 131072L / (x2 - x);
    cdata.offset.x = - cdata.coeff.x * x;

    y = raw_accel[1];
    y2 = raw_accel[4];
    cdata.coeff.y = 131072L / (y2 - y);
    cdata.offset.y = - cdata.coeff.y * y;

    z = raw_accel[8];
    z2 = raw_accel[2];
    cdata.coeff.z = 131072L / (z2 - z);
    cdata.offset.z = - cdata.coeff.z * z;

    printf("Calibration data:\n");
    printf("coeff x = %d, offset x = %d\n", cdata.coeff.x, cdata.offset.x);
    printf("coeff y = %d, offset y = %d\n", cdata.coeff.y, cdata.offset.y);
    printf("coeff z = %d, offset z = %d\n", cdata.coeff.z, cdata.offset.z);
}

static void save_calibration_data(void)
{
    int fd;

    printf("Saving to SD card...");
    fd = fat16_open("/CALIB.TXT", 'w');
    if (fd < 0) {
        printf("failed\n");
        return;
    }

    {
        char buffer[64];
        unsigned int len;
        int ret;

        sprintf(buffer, "%d, 0, 0, %d\n", cdata.coeff.x, cdata.offset.x);
        len = strlen(buffer);
        ret = fat16_write(fd, buffer, len);
        if (ret < 0 || (unsigned int)ret != len) {
            fat16_close(fd);
            printf("failed\n");
            return;
        }
    }

    {
        char buffer[64];
        unsigned int len;
        int ret;

        sprintf(buffer, "0, %d, 0, %d\n", cdata.coeff.y, cdata.offset.y);
        len = strlen(buffer);
        ret = fat16_write(fd, buffer, len);
        if (ret < 0 || (unsigned int)ret != len) {
            fat16_close(fd);
            printf("failed\n");
            return;
        }
    }

    {
        char buffer[64];
        unsigned int len;
        int ret;

        sprintf(buffer, "0, 0, %d, %d\n", cdata.coeff.z, cdata.offset.z);
        len = strlen(buffer);
        ret = fat16_write(fd, buffer, len);
        if (ret < 0 || (unsigned int)ret != len) {
            fat16_close(fd);
            printf("failed\n");
            return;
        }
    }

    fat16_close(fd);
    block_storage_flush();
    printf("done\n");
}


int main(void)
{
    unsigned int i;
    struct partition_info_t p;
    int16_t raw_accel[9];

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

    printf("Place the MPU6050 on a flat surface such that Z axis is pointing to earth\n");
    wait_for_user();
    compute_avg(&raw_accel[0], &raw_accel[1], &raw_accel[2]);

    printf("Place the MPU6050 on a flat surface such that Y axis is pointing to earth\n");
    wait_for_user();
    compute_avg(&raw_accel[3], &raw_accel[4], &raw_accel[5]);

    printf("Place the MPU6050 on a flat surface such that X axis is pointing to earth\n");
    wait_for_user();
    compute_avg(&raw_accel[6], &raw_accel[7], &raw_accel[8]);

    compute_calibration_data(raw_accel);
    save_calibration_data();

    printf("\n");
    while (1) {
        char buffer[128];
        struct mpu6050_sample_t raw_sample;
        int16_t ax, ay, az;

        mpu6050_clear_samples();
        while (mpu6050_get_sample_count() == 0)
            ;
        mpu6050_get_sample(&raw_sample);

        {
            int32_t x;
            x = raw_sample.accel.x;
            x <<= 4;
            x *= cdata.coeff.x;
            x >>= 4;
            x += cdata.offset.x;

            int32_t y;
            y = raw_sample.accel.y;
            y <<= 4;
            y *= cdata.coeff.y;
            y >>= 4;
            y += cdata.offset.y;

            int32_t z;
            z = raw_sample.accel.z;
            z <<= 4;
            z *= cdata.coeff.z;
            z >>= 4;
            z += cdata.offset.z;

            ax = x >> 4;
            ay = y >> 4;
            az = z >> 4;
        }

        sprintf(buffer,
                "\rraw=(%d, %d, %d), calibrated=(%d, %d, %d)       ",
                raw_sample.accel.x, raw_sample.accel.y, raw_sample.accel.z,
                ax, ay, az);
        printf("%s", buffer);

        mcu_delay(100);
    }

    return 0;
}

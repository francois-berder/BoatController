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
#include <stdlib.h>
#include <xc.h>
#include "config.h"
#include "controller.h"
#include "fat16/fat16.h"
#include "mbr/mbr.h"
#include "mcu.h"
#include "mpu6050/mpu6050.h"
#include "output.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph/spi.h"
#include "periph/timer1.h"
#include "periph/uart.h"
#include "periph_conf.h"
#include "radio.h"
#include "sdcard/sdcard.h"
#include "sdcard_cache/sdcard_cache.h"
#include "status.h"

/* Pins for UART interface */
#define UART_TX_PIN     (GPIO_PIN(PORT_B, 15))
#define UART_RX_PIN     (GPIO_PIN(PORT_B, 14))

/* Pins for MPU6050 */
#define I2C_SCL_PIN     (GPIO_PIN(PORT_B, 8))
#define I2C_SDA_PIN     (GPIO_PIN(PORT_B, 9))

/* Pins for SD card */
#define MOSI_PIN        (GPIO_PIN(PORT_B, 13))
#define MISO_PIN        (GPIO_PIN(PORT_B, 4))
#define SCK_PIN         (GPIO_PIN(PORT_B, 6))
#define CS_PIN          (GPIO_PIN(PORT_B, 7))

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static const char *welcome_msg = "Boat Controller firmware " FIRMWARE_VERSION
                                 " - " __DATE__ " " __TIME__
                                 "\n";

static struct storage_dev_t dev = {
    sdcard_cache_read,
    sdcard_cache_read_byte,
    sdcard_cache_write,
    sdcard_cache_seek
};

/**
 * @brief Try to find a FAT16 partition on the SD card
 *
 * @return First sector of the FAT16 partition, 0 if no
 *         partition was found.
 */
static uint32_t find_fat16_partition(struct sdcard_spi_dev_t *dev)
{
    unsigned int i;
    uint32_t first_sector = 0;

    printf("Reading MBR...");
    mbr_read_partition_table(dev);
    printf("done\n");

    printf("Looking for a FAT16 partition...\n");
    for (i = 0; i < PARTITION_ENTRY_COUNT; ++i) {
        struct partition_info_t p = mbr_get_partition_info(i);
        if ((p.status == BOOTABLE_PARTITION || p.status == INACTIVE_PARTITION)
        &&  p.type == FAT16_PARTITION_TYPE) {
            uint32_t size_100kB = p.size / 100000; /* size in 100kB unit */
            printf("Found FAT16 partition at entry %u\n", i);
            printf("\tstart_sector: %lu\n", p.start_sector);
            printf("\tsize: %lu bytes (%lu.%lu MB)\n", p.size, size_100kB / 10, size_100kB % 10);
            first_sector = p.start_sector;
            break;
        }
    }
    if (i == PARTITION_ENTRY_COUNT)
        printf("Failed to found a FAT16 partition\n");

    return first_sector;
}

static int load_calibration_data(struct mpu6050_calibration_data_t *cdata)
{
    int fd;
    char buffer[128];
    unsigned int len = 0;
    unsigned int i = 0;
    int ret = 0;
    char *beg = NULL;
    unsigned int num = 0;

    fd = fat16_open("/CALIB.TXT", 'r');
    if (fd < 0)
        return -1;

    /* Attempt to store entire file content into buffer */
    while (len < 128 && (ret = fat16_read(fd, &buffer[len], 1)) == 1)
        ++len;

    fat16_close(fd);

    if (i == 128 || ret < 0)
        return -1;

    /* Replace comma and newlines by NULL character */
    for (i = 0; i < len; ++i) {
        if (buffer[i] == ',' || buffer[i] == '\n')
            buffer[i] = '\0';
    }

    /*
     * Parse buffer. We assume that format of this file is:
     *
     * accel.coeff.x, 0, 0, accel.offset.x
     * 0, accel.coeff.y, 0, accel.offset.y
     * 0, 0, accel.coeff.z, accel.offset.z
     * gyro.offset.x, gyro.offset.y, gyro.offset.z
     */
    beg = &buffer[0];
    while (i < len && *beg == ' ') {    /* Skip spaces at beginning of file */
        ++i;
        beg = &buffer[i];
    }
    for (; i < len; ++i) {
        if (buffer[i] == '\0') {
            unsigned int j = i + 1;
            unsigned int n = atoi(beg);

            switch (num) {
            case 0:
                cdata->accel.coeff.x = n;
                break;
            case 2:
                cdata->accel.offset.x = n;
                break;
            case 5:
                cdata->accel.coeff.y = n;
                break;
            case 7:
                cdata->accel.offset.y = n;
                break;
            case 10:
                cdata->accel.coeff.z = n;
                break;
            case 11:
                cdata->accel.offset.z = n;
                break;
            case 12:
                cdata->gyro.offset.x = n;
                break;
            case 13:
                cdata->gyro.offset.y = n;
                break;
            case 14:
                cdata->gyro.offset.z = n;
                break;
            default:
                break;
            }

            ++num;

            beg = &buffer[j];

            /* Skip spaces after comma or newline */
            while (j < len && *beg == ' ') {
                ++j;
                beg = &buffer[j];
            }
        }
    }

    /* We expect 15 numbers in this file */
    if (num != 15)
        return -1;

    return 0;
}

int main(void)
{
    struct board_config_t config = {1, 1};
    struct mpu6050_dev_t mpu6050_dev;
    struct sdcard_spi_dev_t sdcard_dev;

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

    status_configure();
    status_set_mode(STATUS_FAST_BLINK);

    /* Configure uart */
    gpio_init_out(UART_TX_PIN, 1);
    gpio_init_in(UART_RX_PIN);
    RPOR7 |= 0x0300;
    RPINR18 |= 0x000E;
    uart_power_up(UART_1);
    uart_configure(UART_1, UART_BD_9600);
    uart_enable(UART_1);

    printf(welcome_msg);

    /* Configure radio */
    printf("Configuring radio...");
    radio_configure();
    printf("done\n");

    /* Configure output module */
    printf("Configuring output module...");
    output_configure();
    printf("done\n");

    /* Prepare MPU6050 device */
    mpu6050_dev.i2c_num = I2C_1;
    mpu6050_dev.cdata.accel.offset.x = 0;
    mpu6050_dev.cdata.accel.offset.y = 0;
    mpu6050_dev.cdata.accel.offset.z = 0;
    mpu6050_dev.cdata.accel.coeff.x = 16;
    mpu6050_dev.cdata.accel.coeff.y = 16;
    mpu6050_dev.cdata.accel.coeff.z = 16;
    mpu6050_dev.cdata.gyro.offset.x = 0;
    mpu6050_dev.cdata.gyro.offset.y = 0;
    mpu6050_dev.cdata.gyro.offset.z = 0;

    /* Configure MPU6050 device */
    printf("Configuring MPU6050 device...");
    gpio_init_out(I2C_SCL_PIN, 1);
    gpio_init_out(I2C_SDA_PIN, 1);
    i2c_power_up(I2C_1);
    i2c_configure(I2C_1, I2C_FAST_SPEED);
    i2c_enable(I2C_1);
    if (!mpu6050_init(&mpu6050_dev, 1, 1)) {
        printf("done\n");
    } else {
        printf("failed\n");
        config.mpu6050_enabled = 0;
        i2c_disable(I2C_1);
        i2c_power_down(I2C_1);
    }

    /* Configure SD card */
    printf("Configuring SD card...");
    gpio_init_out(MOSI_PIN, 0);
    gpio_init_in(MISO_PIN);
    gpio_init_out(SCK_PIN, 0);
    gpio_init_out(CS_PIN, 1);

    RPOR6bits.RP13R = 0x0007;
    RPINR20bits.SDI1R = 0x0004;
    RPOR3bits.RP6R = 0x0008;

    spi_power_up(SPI_1);
    spi_enable(SPI_1);

    sdcard_dev.spi_num = SPI_1;
    sdcard_dev.cs_pin = CS_PIN;
    if (!sdcard_init(&sdcard_dev)) {
        /*
        * Increase clock frequency:
        *  - Use system clock for SPI module
        *  - Set BRG to 0
        */
        PMD4 &= ~_PMD4_REFOMD_MASK;
        REFOCONL = _REFOCONL_ROEN_MASK;
        spi_disable(SPI_1);
        SPI1BRGL = 0;
        SPI1CON1 |= _SPI1CON1_MCLKEN_MASK;
        spi_enable(SPI_1);

        printf("done\n");
    } else {
        printf("failed\n");
        config.sdcard_enabled = 0;
    }

    if (config.sdcard_enabled) {
        uint32_t partition_offset;
        printf("Configuring SD card cache...");
        sdcard_cache_init(sdcard_dev);
        printf("done\n");

        partition_offset = find_fat16_partition(&sdcard_dev);
        if (partition_offset > 0) {
            partition_offset <<= 9;
            fat16_init(dev, partition_offset);
        } else {
            config.sdcard_enabled = 0;
        }
    }

    if (!config.sdcard_enabled) {
        spi_disable(SPI_1);
        spi_power_down(SPI_1);
    }

    /* Attempt to retrieve calibration data for MPU6050 from SD card */
    if (config.mpu6050_enabled && config.sdcard_enabled) {
        if (load_calibration_data(&mpu6050_dev.cdata) < 0)
            printf("Failed to load calibration data from SD card.\n");
    }

    printf("Initialisation finished\n");

    /* Change status LED depending on MPU6050 and SD card initialisation */
    if (config.mpu6050_enabled && config.sdcard_enabled)
        status_set_mode(STATUS_FLASH);
    else if (!config.mpu6050_enabled && config.sdcard_enabled)
        status_set_mode(STATUS_ONE_PER_2SEC);
    else if (config.mpu6050_enabled && !config.sdcard_enabled)
        status_set_mode(STATUS_TWO_PER_2SEC);
    else
        status_set_mode(STATUS_THREE_PER_2SEC);

    printf("Starting controller\n");
    controller_run(config, mpu6050_dev);

    return 0;
}

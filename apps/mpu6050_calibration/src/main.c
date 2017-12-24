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
#include "fat16/fat16.h"
#include "mbr/mbr.h"
#include "mcu.h"
#include "mpu6050/mpu6050.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph/spi.h"
#include "periph/timer.h"
#include "periph/timer1.h"
#include "periph/uart.h"
#include "periph_conf.h"
#include "sdcard/sdcard.h"
#include "sdcard_cache/sdcard_cache.h"

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

#define LED_PIN         (GPIO_PIN(PORT_B, 5))

#ifndef SAMPLE_COUNT
#define SAMPLE_COUNT    (128U)
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static const char *welcome_msg = "Boat Controller firmware " FIRMWARE_VERSION
                                 " - " __DATE__ " " __TIME__
                                 "\n";

static struct mpu6050_dev_t mpu6050_dev;
static struct storage_dev_t dev = {
    sdcard_cache_read,
    sdcard_cache_read_byte,
    sdcard_cache_write,
    sdcard_cache_seek
};

static struct mpu6050_calibration_data_t cdata;

static void stop(const char *reason)
{
    printf("\n%s\n", reason);
    while (1)
        ;
}

void timer2_callback(void)
{
    gpio_toggle(LED_PIN);
}

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

static void wait_for_user(void)
{
    char c = 0;
    do {
        char buffer[128];
        struct mpu6050_sample_t raw_sample;

        mpu6050_get_acc(&mpu6050_dev, &raw_sample);
        sprintf(buffer,
                "\rPress enter to continue, (%d, %d, %d)\n",
                raw_sample.accel.x, raw_sample.accel.y, raw_sample.accel.z);
        printf("%s", buffer);

        /* If no character is avaible, let's try again 20ms later */
        if (uart_read_noblock(UART_1, &c, 1) == 0)
            mcu_delay(20);
    } while (c != '\n');
}

static void compute_avg(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz)
{
    struct mpu6050_sample_t samples[SAMPLE_COUNT];
    unsigned int i;
    uint64_t avg[6] = {0, 0, 0, 0, 0, 0};

    printf("Retrieving %u samples", SAMPLE_COUNT);
    for (i = 0; i < SAMPLE_COUNT; ++i) {
        printf(".");
        mpu6050_get_acc_gyro(&mpu6050_dev, &samples[i]);
        mcu_delay(10);
    }
    printf("\n");

    /* Compute average for each component */
    for (i = 0; i < SAMPLE_COUNT; ++i) {
        avg[0] += samples[i].accel.x;
        avg[1] += samples[i].accel.y;
        avg[2] += samples[i].accel.z;
        avg[3] += samples[i].gyro.x;
        avg[4] += samples[i].gyro.y;
        avg[5] += samples[i].gyro.z;
    }
    avg[0] >>= 7;
    avg[1] >>= 7;
    avg[2] >>= 7;
    avg[3] >>= 7;
    avg[4] >>= 7;
    avg[5] >>= 7;

    *ax = avg[0];
    *ay = avg[1];
    *az = avg[2];
    *gx = avg[0];
    *gy = avg[1];
    *gz = avg[2];

    printf("avg accel (%d, %d, %d)\n", *ax, *ay, *az);
    printf("avg gyro (%d, %d, %d)\n", *gx, *gy, *gz);
}

static void compute_calibration_data(int16_t *raw_accel, int16_t *raw_gyro)
{
    int32_t x, x2;
    int32_t y, y2;
    int32_t z, z2;

    x = raw_accel[0];
    x2 = raw_accel[6];
    cdata.accel.coeff.x = 131072L / (x2 - x);
    cdata.accel.offset.x = - cdata.accel.coeff.x * x;

    y = raw_accel[1];
    y2 = raw_accel[4];
    cdata.accel.coeff.y = 131072L / (y2 - y);
    cdata.accel.offset.y = - cdata.accel.coeff.y * y;

    z = raw_accel[8];
    z2 = raw_accel[2];
    cdata.accel.coeff.z = 131072L / (z2 - z);
    cdata.accel.offset.z = - cdata.accel.coeff.z * z;

    cdata.gyro.offset.x = (raw_gyro[0] + raw_gyro[3] + raw_gyro[6]) / 3;
    cdata.gyro.offset.y = (raw_gyro[1] + raw_gyro[4] + raw_gyro[7]) / 3;
    cdata.gyro.offset.z = (raw_gyro[2] + raw_gyro[5] + raw_gyro[8]) / 3;

    printf("Calibration data:\n");
    printf("\tAccelerometer:\n");
    printf("\t\tcoeff x = %d, offset x = %d\n", cdata.accel.coeff.x, cdata.accel.offset.x);
    printf("\t\tcoeff y = %d, offset y = %d\n", cdata.accel.coeff.y, cdata.accel.offset.y);
    printf("\t\tcoeff z = %d, offset z = %d\n", cdata.accel.coeff.z, cdata.accel.offset.z);
    printf("\tGyroscope:\n");
    printf("\t\toffset x = %d\n", cdata.gyro.offset.x);
    printf("\t\toffset y = %d\n", cdata.gyro.offset.y);
    printf("\t\toffset z = %d\n", cdata.gyro.offset.z);
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

        sprintf(buffer, "%d, 0, 0, %d\n", cdata.accel.coeff.x, cdata.accel.offset.x);
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

        sprintf(buffer, "0, %d, 0, %d\n", cdata.accel.coeff.y, cdata.accel.offset.y);
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

        sprintf(buffer, "0, 0, %d, %d\n", cdata.accel.coeff.z, cdata.accel.offset.z);
        len = strlen(buffer);
        ret = fat16_write(fd, buffer, len);
        if (ret < 0 || (unsigned int)ret != len) {
            fat16_close(fd);
            printf("failed\n");
            return;
        }
    }

    /* Save calibration data for the gyroscope */
    {
        char buffer[128];
        unsigned int len;
        int ret;

        sprintf(buffer, "%d, %d, %d\n",
                cdata.gyro.offset.x, cdata.gyro.offset.y, cdata.gyro.offset.z);
        len = strlen(buffer);
        ret = fat16_write(fd, buffer, len);
        if (ret < 0 || (unsigned int)ret != len) {
            fat16_close(fd);
            printf("failed\n");
            return;
        }
    }

    fat16_close(fd);
    sdcard_cache_flush();
    printf("done\n");
}


int main(void)
{
    int16_t raw_accel[9], raw_gyro[9];
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

    /* Turn on LED pin */
    gpio_init_out(LED_PIN, 1);

    /* Configure uart */
    gpio_init_out(UART_TX_PIN, 1);
    gpio_init_in(UART_RX_PIN);
    RPOR7bits.RP15R = 0x03;
    RPINR18bits.U1RXR = 0x0E;
    uart_power_up(UART_1);
    uart_configure(UART_1, UART_BD_9600);
    uart_enable(UART_1);

    printf(welcome_msg);

    /* Prepare MPU6050 device */
    mpu6050_dev.i2c_num = I2C_1;
    mpu6050_dev.cdata = mpu6050_create_default_calibration_data();

    /* Configure MPU6050 device */
    printf("Configuring MPU6050 device...");
    gpio_init_out(I2C_SCL_PIN, 1);
    gpio_init_out(I2C_SDA_PIN, 1);
    i2c_power_up(I2C_1);
    i2c_configure(I2C_1, I2C_FAST_SPEED);
    i2c_enable(I2C_1);
    if (!mpu6050_init(&mpu6050_dev, 1, 1))
        printf("done\n");
    else
        stop("Failed to initialise MPU6050");

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
        stop("Failed to configure SD card");
    }

    printf("Configuring SD card cache...");
    sdcard_cache_init(sdcard_dev);
    printf("done\n");


    {
        uint32_t partition_offset = find_fat16_partition(&sdcard_dev);
        if (partition_offset > 0) {
            partition_offset <<= 9;
            fat16_init(dev, partition_offset);
        } else {
            stop("Failed to find a FAT16 partition");
        }
    }

    printf("Initialisation finished\n");

    /* Configure timer 2 to blink LED */
    timer_power_up(TIMER_2);
    timer_configure(TIMER_2, TIMER3_PRESCALER_256, 13000, 1);
    timer_start(TIMER_2);

    printf("Place the MPU6050 on a flat surface such that Z axis is pointing to earth\n");
    wait_for_user();
    compute_avg(&raw_accel[0], &raw_accel[1], &raw_accel[2],
                &raw_gyro[0], &raw_gyro[1], &raw_gyro[2]);

    printf("Place the MPU6050 on a flat surface such that Y axis is pointing to earth\n");
    wait_for_user();
    compute_avg(&raw_accel[3], &raw_accel[4], &raw_accel[5],
                &raw_gyro[3], &raw_gyro[4], &raw_gyro[5]);

    printf("Place the MPU6050 on a flat surface such that X axis is pointing to earth\n");
    wait_for_user();
    compute_avg(&raw_accel[6], &raw_accel[7], &raw_accel[8],
                &raw_gyro[6], &raw_gyro[7], &raw_gyro[8]);

    compute_calibration_data(raw_accel, raw_gyro);
    save_calibration_data();

    printf("\n");

    /* Stop blinking LED */
    timer_stop(TIMER_2);
    timer_power_down(TIMER_2);
    gpio_write(LED_PIN, 1);

    /* Load new calibration data */
    mpu6050_dev.cdata = cdata;

    while (1) {
        char buffer[128];
        struct mpu6050_sample_t raw_sample;
        int16_t ax, ay, az;

        mpu6050_get_acc(&mpu6050_dev, &raw_sample);

        {
            int32_t x;
            x = raw_sample.accel.x;
            x <<= 4;
            x *= cdata.accel.coeff.x;
            x >>= 4;
            x += cdata.accel.offset.x;

            int32_t y;
            y = raw_sample.accel.y;
            y <<= 4;
            y *= cdata.accel.coeff.y;
            y >>= 4;
            y += cdata.accel.offset.y;

            int32_t z;
            z = raw_sample.accel.z;
            z <<= 4;
            z *= cdata.accel.coeff.z;
            z >>= 4;
            z += cdata.accel.offset.z;

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

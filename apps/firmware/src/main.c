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
#include <xc.h>
#include "block_storage.h"
#include "mbr.h"
#include "mcu.h"
#include "mpu6050.h"
#include "output.h"
#include "periph/gpio.h"
#include "periph/timer1.h"
#include "periph/uart.h"
#include "periph_conf.h"
#include "radio.h"
#include "sdcard.h"
#include "status.h"

#define UART_TX_PIN     (GPIO_PIN(PORT_B, 15))
#define UART_RX_PIN     (GPIO_PIN(PORT_B, 14))

struct board_config_t {
    uint8_t mpu6050_enabled;
    uint8_t sdcard_enabled;
};

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static const char *welcome_msg = "Boat Controller firmware " FIRMWARE_VERSION
                                 " - " __DATE__ " " __TIME__
                                 "\n";

int main(void)
{
    struct board_config_t config = {1, 1};
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

    /* Configure timer 1 to enable mcu_delay */
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

    /* Configure MPU6050 device */
    printf("Configuring MPU6050 device...");
    if (!mpu6050_init()) {
        printf("done\n");
    } else {
        printf("failed\n");
        config.mpu6050_enabled = 0;
    }

    /* Configure SD card */
    printf("Configuring SD card...");
    if (!sdcard_init())
        printf("done\n");
    else {
        printf("failed\n");
        config.sdcard_enabled = 0;
    }

    if (config.sdcard_enabled) {
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
                uint32_t size_100kB = (10 * (p.size >> 10)) >> 10; /* size in 100kB unit */
                printf("Found FAT16 partition at entry %u\n", i);
                printf("\tstart_sector: %lu\n", p.start_sector);
                printf("\tsize: %lu bytes (%lu.%lu MB)\n", p.size, size_100kB / 10, size_100kB % 10);
                break;
            }
        }
        if (i == PARTITION_ENTRY_COUNT) {
            printf("Failed to found a FAT16 partition\n");
            config.sdcard_enabled = 0;
        }
    }

    printf("Initialisation finished\n");
    printf("Starting main loop\n");

    /* Change status LED depending on MPU6050 and SD card initialisation */
    if (config.mpu6050_enabled && config.sdcard_enabled)
        status_set_mode(STATUS_FLASH);
    else if (!config.mpu6050_enabled && config.sdcard_enabled)
        status_set_mode(STATUS_ONE_PER_2SEC);
    else if (config.mpu6050_enabled && !config.sdcard_enabled)
        status_set_mode(STATUS_TWO_PER_2SEC);
    else
        status_set_mode(STATUS_THREE_PER_2SEC);

    while (1) {

    }

    return 0;
}

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

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>
#include "mpu6050/mpu6050.h"
#include "sdcard/sdcard.h"

struct board_config_t {
    uint8_t mpu6050_enabled;
    uint8_t sdcard_enabled;
    struct mpu6050_dev_t mpu6050_dev;
    struct sdcard_spi_dev_t sdcard_dev;
};

/* Pin for status LED */
#define LED_PIN         (GPIO_PIN(PORT_B, 5))

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

/* Pins for radio */
#define RADIO_DIR_PIN       (GPIO_PIN(PORT_B, 0))
#define RADIO_SPEED_PIN     (GPIO_PIN(PORT_B, 1))

/* Pins for controlling rudders & motor controllers */
#define LEFT_RUDDER_PIN         (GPIO_PIN(PORT_B, 2))
#define RIGHT_RUDDER_PIN        (GPIO_PIN(PORT_B, 3))
#define LEFT_MOTOR_PIN          (GPIO_PIN(PORT_A, 2))
#define RIGHT_MOTOR_PIN         (GPIO_PIN(PORT_A, 3))

#endif

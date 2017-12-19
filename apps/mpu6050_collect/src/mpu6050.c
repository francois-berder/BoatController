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

#include "mcu.h"
#include "mpu6050.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph_conf.h"

#define SCL_PIN         (GPIO_PIN(PORT_B, 8))
#define SDA_PIN         (GPIO_PIN(PORT_B, 9))

#define MPU6050_ADDRESS     (0x68)

/* Register address */
#define WHO_AM_I            (0x75)
#define CONFIG              (0x1A)
#define GYRO_CONFIG         (0x1B)
#define ACCEL_CONFIG        (0x1C)
#define FIFO_CONFIG         (0x23)
#define USER_CTRL           (0x6A)
#define PWR_MGMT_1          (0x6B)
#define FIFO_COUNT_HIGH     (0x72)
#define FIFO_COUNT_LOW      (0x73)
#define FIFO_DATA           (0x74)
#define ACCEL_X_HIGH        (0x3B)
#define ACCEL_X_LOW         (0x3C)

/* Register values */
#define MPU6050_DEVICE_ID   (0x68)
#define RESET               (0x80)
#define SLEEP               (0x40)
#define ACCEL_RANGE_2G      (0x00)
#define ACCEL_RANGE_4G      (0x08)
#define ACCEL_RANGE_8G      (0x10)
#define ACCEL_RANGE_16G     (0x18)
#define ACCEL_FIFO_EN       (0x08)
#define FIFO_EN             (0x40)
#define FIFO_RESET          (0x04)
#define GYRO_RANGE_250      (0x00)
#define GYRO_RANGE_500      (0x08)
#define GYRO_RANGE_1000     (0x10)
#define GYRO_RANGE_2000     (0x18)
#define GYRO_X_FIFO_EN      (0x40)
#define GYRO_Y_FIFO_EN      (0x20)
#define GYRO_Z_FIFO_EN      (0x10)

static uint8_t read_8bit_reg(uint8_t address)
{
    uint8_t value;

    i2c_write(I2C_1, MPU6050_ADDRESS, &address, 1);
    i2c_read(I2C_1, MPU6050_ADDRESS, &value, sizeof(value));

    return value;
}

static void write_8bit_reg(uint8_t address, uint8_t value)
{
    uint8_t buffer[2];
    buffer[0] = address;
    buffer[1] = value;
    i2c_write(I2C_1, MPU6050_ADDRESS, buffer, sizeof(buffer));
}

static uint16_t to_le(uint8_t msb, uint8_t lsb)
{
    return (msb << 8) | lsb;
}

int mpu6050_init(void)
{
    gpio_init_out(SCL_PIN, 1);
    gpio_init_out(SDA_PIN, 1);

    i2c_power_up(I2C_1);
    i2c_configure(I2C_1, I2C_FAST_SPEED);
    i2c_enable(I2C_1);

    /*
     * Let's wait roughly 100ms to give more time for the
     * MPU6050 to boot.
     */
    mcu_delay(100);

    if (read_8bit_reg(WHO_AM_I) != MPU6050_DEVICE_ID)
        return -1;

    /* Reset device - clears most registers to 0 */
    write_8bit_reg(PWR_MGMT_1, RESET);
    mcu_delay(50);

    /* Wake up from sleep */
    write_8bit_reg(PWR_MGMT_1, 0);
    mcu_delay(10);

    write_8bit_reg(CONFIG, 0);

    /* Configure accelerometer */
    write_8bit_reg(ACCEL_CONFIG, ACCEL_RANGE_4G);

    /* Configure gyroscope */
    write_8bit_reg(GYRO_CONFIG, GYRO_RANGE_500);

    return 0;
}

void mpu6050_get_sample(struct mpu6050_sample_t *sample)
{
    uint8_t buffer[14];
    uint8_t address = ACCEL_X_HIGH;
    i2c_write(I2C_1, MPU6050_ADDRESS, &address, 1);
    i2c_read(I2C_1, MPU6050_ADDRESS, buffer, sizeof(buffer));

    /* Fill accelerometer data */
    sample->accel.x = to_le(buffer[0], buffer[1]);
    sample->accel.y = to_le(buffer[2], buffer[3]);
    sample->accel.z = to_le(buffer[4], buffer[5]);

    /* buffer[6] and buffer[7] contain data about temperature */

    /* Fill gyroscope data */
    sample->gyro.x = to_le(buffer[8], buffer[9]);
    sample->gyro.y = to_le(buffer[10], buffer[11]);
    sample->gyro.z = to_le(buffer[12], buffer[13]);
}

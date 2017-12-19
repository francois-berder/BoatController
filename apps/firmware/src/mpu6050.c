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
#include "periph/timer.h"
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

#define SAMPLE_FIFO_SIZE        (32)

static struct mpu6050_calibration_data_t cdata;
static volatile struct mpu6050_sample_t samples[SAMPLE_FIFO_SIZE];
static volatile unsigned int sample_count;
static volatile unsigned int fifo_start_index;

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

static void rectify_accel_data(struct mpu6050_sample_t *sample)
{
    int32_t ax, ay, az;

    ax = sample->accel.x;
    ax <<= 4;
    ax *= cdata.coeff.x;
    ax >>= 4;
    ax += cdata.offset.x;

    ay = sample->accel.y;
    ay <<= 4;
    ay *= cdata.coeff.y;
    ay >>= 4;
    ay += cdata.offset.y;

    az = sample->accel.z;
    az <<= 4;
    az *= cdata.coeff.z;
    az >>= 4;
    az += cdata.offset.z;

    sample->accel.x = ax >> 4;
    sample->accel.y = ay >> 4;
    sample->accel.z = az >> 4;
}

/* Read a sample from MPU6050 and add it to the FIFO */
void timer5_callback(void)
{
    uint8_t buffer[14];
    uint8_t address = ACCEL_X_HIGH;
    unsigned int index = (fifo_start_index + sample_count) & (SAMPLE_FIFO_SIZE - 1);

    if (sample_count >= SAMPLE_FIFO_SIZE)
        return;

    i2c_write(I2C_1, MPU6050_ADDRESS, &address, 1);
    i2c_read(I2C_1, MPU6050_ADDRESS, buffer, sizeof(buffer));

    /* Fill accelerometer data */
    samples[index].accel.x = to_le(buffer[0], buffer[1]);
    samples[index].accel.y = to_le(buffer[2], buffer[3]);
    samples[index].accel.z = to_le(buffer[4], buffer[5]);

    /* buffer[6] and buffer[7] contain data about temperature */

    /* Fill gyroscope data */
    samples[index].gyro.x = to_le(buffer[8], buffer[9]);
    samples[index].gyro.y = to_le(buffer[10], buffer[11]);
    samples[index].gyro.z = to_le(buffer[12], buffer[13]);


    ++sample_count;
}

int mpu6050_init(void)
{
    /* Set default calibration data */
    cdata.coeff.x = 16;
    cdata.offset.x = 0;
    cdata.coeff.y = 16;
    cdata.offset.y = 0;
    cdata.coeff.z = 16;
    cdata.offset.z = 0;

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

    if (read_8bit_reg(WHO_AM_I) != MPU6050_DEVICE_ID) {
        i2c_disable(I2C_1);
        i2c_power_down(I2C_1);
        return -1;
    }

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

    /* Configure timer 5 to fire every 20ms */
    fifo_start_index = 0;
    sample_count = 0;
    timer_power_up(TIMER_5);
    timer_configure(TIMER_5, TIMER5_PRESCALER_64, 1250, 1);
    timer_start(TIMER_5);

    return 0;
}

void mpu6050_set_calibration_data(struct mpu6050_calibration_data_t _cdata)
{
    cdata = _cdata;
}

unsigned int mpu6050_get_sample_count(void)
{
    unsigned int samples_available;

    mcu_disable_interrupts();
    samples_available = sample_count;
    mcu_enable_interrupts();

    return samples_available;
}

void mpu6050_get_sample(struct mpu6050_sample_t *sample)
{
    unsigned int index = SAMPLE_FIFO_SIZE;

    mcu_disable_interrupts();
    if (sample_count > 0) {
        index = (fifo_start_index + sample_count) & (SAMPLE_FIFO_SIZE - 1);
        *sample = samples[index];
        ++fifo_start_index;
        fifo_start_index &= (SAMPLE_FIFO_SIZE - 1);
        --sample_count;
    }
    mcu_enable_interrupts();

    if (index != SAMPLE_FIFO_SIZE)
        rectify_accel_data(sample);
}

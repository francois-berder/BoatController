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

#include "core_timer.h"
#include "mcu.h"
#include "mpu6050.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph_conf.h"

#define MPU6050_ADDRESS     (0x68)

/* Register address */
#define WHO_AM_I            (0x75)
#define CONFIG              (0x1A)
#define GYRO_CONFIG         (0x1B)
#define ACCEL_CONFIG        (0x1C)
#define FIFO_CONFIG         (0x23)
#define USER_CTRL           (0x6A)
#define PWR_MGMT_1          (0x6B)
#define PWR_MGMT_2          (0x6C)
#define FIFO_COUNT_HIGH     (0x72)
#define FIFO_COUNT_LOW      (0x73)
#define FIFO_DATA           (0x74)
#define ACCEL_X_HIGH        (0x3B)
#define GYRO_X_HIGH         (0x43)

/* Register values */
#define MPU6050_DEVICE_ID   (0x68)
#define RESET               (0x80)
#define SLEEP               (0x40)
#define TEMP_DIS            (0x08)
#define STBY_XA             (0x20)
#define STBY_YA             (0x10)
#define STBY_ZA             (0x08)
#define STBY_XG             (0x04)
#define STBY_YG             (0x02)
#define STBY_ZG             (0x01)
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

static uint16_t compute_timeout(unsigned int i2c_num)
{
    uint32_t speed = i2c_get_speed(i2c_num);
    uint32_t timeout;

    /*
     * We send at most 15 bytes during a i2c operation.
     * So let's give it enough time such that timeout
     * is greater than 24 i2c cycles.
     */
    timeout = TICKS_PER_SEC;
    timeout *= 120;
    timeout /= speed;
    if (timeout == 0)
        timeout = 1;

    return timeout;
}

/**
 * @brief Read content of a 8-bit register
 *
 * @param[in] i2c_num
 * @param[in] address
 * @param[out] data
 * @param[in] timeout
 * @return see i2c_write_safe/i2c_read_safe
 */
static int read_8bit_reg(unsigned int i2c_num, uint8_t address, uint8_t *data, uint16_t timeout)
{
    int ret;
    ret = i2c_write_safe(i2c_num, MPU6050_ADDRESS, &address, sizeof(address), timeout);
    if (ret < 0)
        return ret;

    return i2c_read_safe(i2c_num, MPU6050_ADDRESS, data, sizeof(*data), timeout);
}

/**
 * @brief Write content of a 8-bit register
 *
 * @param[in] i2c_num
 * @param[in] address
 * @param[in] value
 * @param[in] timeout
 * @return see i2c_write_safe
 */
static int write_8bit_reg(unsigned int i2c_num, uint8_t address, uint8_t value, uint16_t timeout)
{
    uint8_t buffer[2];
    buffer[0] = address;
    buffer[1] = value;
    return i2c_write_safe(i2c_num, MPU6050_ADDRESS, buffer, sizeof(buffer), timeout);
}

/**
 * @brief Convert 2 8-bit values to little-endian 16-bit value
 *
 * @param[in] msb most significant byte
 * @param[in] lsb least significant byte
 * @return 16-bit value
 */
static uint16_t to_le(uint8_t msb, uint8_t lsb)
{
    return (msb << 8) | lsb;
}

/**
 * @brief Use calibration data to rectify accelerometer data
 *
 * @param[in] dev
 * @param[in|out] sample
 */
static void rectify_acc(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample)
{
    int32_t x, y, z;

    x = sample->accel.x;
    x <<= 4;
    x *= dev->cdata.accel.coeff.x;
    x >>= 4;
    x += dev->cdata.accel.offset.x;

    y = sample->accel.y;
    y <<= 4;
    y *= dev->cdata.accel.coeff.y;
    y >>= 4;
    y += dev->cdata.accel.offset.y;

    z = sample->accel.z;
    z <<= 4;
    z *= dev->cdata.accel.coeff.z;
    z >>= 4;
    z += dev->cdata.accel.offset.z;

    sample->accel.x = x >> 4;
    sample->accel.y = y >> 4;
    sample->accel.z = z >> 4;
}

/**
 * @brief Use calibration data to rectify gyroscope data
 *
 * @param[in] dev
 * @param[in|out] sample
 */
static void rectify_gyro(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample)
{
    sample->gyro.x -= dev->cdata.gyro.offset.x;
    sample->gyro.y -= dev->cdata.gyro.offset.y;
    sample->gyro.z -= dev->cdata.gyro.offset.z;
}

int mpu6050_init(struct mpu6050_dev_t *dev, unsigned int enable_acc, unsigned int enable_gyro)
{
    uint16_t timeout = compute_timeout(dev->i2c_num);
    uint8_t id;

    if (read_8bit_reg(dev->i2c_num, WHO_AM_I, &id, timeout) < 0)
        return -1;

    if (id != MPU6050_DEVICE_ID)
        return -1;

    /* Reset device - clears most registers to 0 */
    if (write_8bit_reg(dev->i2c_num, PWR_MGMT_1, RESET, timeout) < 0)
        return -1;
    mcu_delay(TICKS_PER_SEC / 20);

    /* Wake up from sleep */
    if (write_8bit_reg(dev->i2c_num, PWR_MGMT_1, 0, timeout) < 0)
        return -1;
    mcu_delay(TICKS_PER_SEC / 100);

    /* Disable temperature sensor */
    if (write_8bit_reg(dev->i2c_num, PWR_MGMT_1, TEMP_DIS, timeout) < 0)
        return -1;

    /* */
    if (write_8bit_reg(dev->i2c_num, CONFIG, 0, timeout) < 0)
        return -1;

    /* Configure accelerometer */
    if (enable_acc) {
        if (write_8bit_reg(dev->i2c_num, ACCEL_CONFIG, ACCEL_RANGE_4G, timeout) < 0)
            return -1;
    } else {
        uint8_t reg;

        if (read_8bit_reg(dev->i2c_num, PWR_MGMT_2, &reg, timeout) < 0)
            return -1;
        reg |= (STBY_XA | STBY_YA | STBY_ZA);
        if (write_8bit_reg(dev->i2c_num, PWR_MGMT_2, reg, timeout) < 0)
            return -1;
    }

    /* Configure gyroscope */
    if (enable_gyro) {
        if (write_8bit_reg(dev->i2c_num, GYRO_CONFIG, GYRO_RANGE_500, timeout) < 0)
            return -1;
    } else {
        uint8_t reg;
        if (read_8bit_reg(dev->i2c_num, PWR_MGMT_2, &reg, timeout) < 0)
            return -1;
        reg |= (STBY_XG | STBY_YG | STBY_ZG);
        if (write_8bit_reg(dev->i2c_num, PWR_MGMT_2, reg, timeout) < 0)
            return -1;
    }

    return 0;
}

int mpu6050_get_acc_gyro(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample)
{
    uint8_t buffer[14];
    uint8_t address = ACCEL_X_HIGH;
    uint16_t timeout = compute_timeout(dev->i2c_num);

    if (i2c_write_safe(dev->i2c_num, MPU6050_ADDRESS, &address, 1, timeout) < 0
    ||  i2c_read_safe(dev->i2c_num, MPU6050_ADDRESS, buffer, sizeof(buffer), timeout) < 0)
        return -1;

    /* Fill accelerometer data */
    sample->accel.x = to_le(buffer[0], buffer[1]);
    sample->accel.y = to_le(buffer[2], buffer[3]);
    sample->accel.z = to_le(buffer[4], buffer[5]);

    /* buffer[6] and buffer[7] contain data about temperature */

    /* Fill gyroscope data */
    sample->gyro.x = to_le(buffer[8], buffer[9]);
    sample->gyro.y = to_le(buffer[10], buffer[11]);
    sample->gyro.z = to_le(buffer[12], buffer[13]);

    rectify_acc(dev, sample);
    rectify_gyro(dev, sample);

    return 0;
}

int mpu6050_get_acc(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample)
{
    uint8_t buffer[6];
    uint8_t address = ACCEL_X_HIGH;
    uint16_t timeout = compute_timeout(dev->i2c_num);

    if (i2c_write_safe(dev->i2c_num, MPU6050_ADDRESS, &address, 1, timeout) < 0
    ||  i2c_read_safe(dev->i2c_num, MPU6050_ADDRESS, buffer, sizeof(buffer), timeout) < 0)
        return -1;

    /* Fill accelerometer data */
    sample->accel.x = to_le(buffer[0], buffer[1]);
    sample->accel.y = to_le(buffer[2], buffer[3]);
    sample->accel.z = to_le(buffer[4], buffer[5]);

    rectify_acc(dev, sample);

    return 0;
}

int mpu6050_get_gyro(struct mpu6050_dev_t *dev, struct mpu6050_sample_t *sample)
{
    uint8_t buffer[6];
    uint8_t address = GYRO_X_HIGH;
    uint16_t timeout = compute_timeout(dev->i2c_num);

    if (i2c_write_safe(dev->i2c_num, MPU6050_ADDRESS, &address, 1, timeout) < 0
    ||  i2c_read_safe(dev->i2c_num, MPU6050_ADDRESS, buffer, sizeof(buffer), timeout) < 0)
        return -1;

    /* Fill gyroscope data */
    sample->gyro.x = to_le(buffer[0], buffer[1]);
    sample->gyro.y = to_le(buffer[2], buffer[3]);
    sample->gyro.z = to_le(buffer[4], buffer[5]);

    rectify_gyro(dev, sample);

    return 0;
}

void mpu6050_power_up(struct mpu6050_dev_t *dev)
{
    uint16_t timeout = compute_timeout(dev->i2c_num);
    uint8_t reg;
    if (read_8bit_reg(dev->i2c_num, PWR_MGMT_1, &reg, timeout) < 0)
        return;
    reg &= ~SLEEP;
    write_8bit_reg(dev->i2c_num, PWR_MGMT_1, reg, timeout);
}

void mpu6050_power_down(struct mpu6050_dev_t *dev)
{
    uint16_t timeout = compute_timeout(dev->i2c_num);
    uint8_t reg;
    if (read_8bit_reg(dev->i2c_num, PWR_MGMT_1, &reg, timeout) < 0)
        return;
    reg |= SLEEP;
    write_8bit_reg(dev->i2c_num, PWR_MGMT_1, reg, timeout);
}

struct mpu6050_calibration_data_t mpu6050_create_default_calibration_data(void)
{
    struct mpu6050_calibration_data_t cdata;

    cdata.accel.offset.x = 0;
    cdata.accel.offset.y = 0;
    cdata.accel.offset.z = 0;
    cdata.accel.coeff.x = 16;
    cdata.accel.coeff.y = 16;
    cdata.accel.coeff.z = 16;
    cdata.gyro.offset.x = 0;
    cdata.gyro.offset.y = 0;
    cdata.gyro.offset.z = 0;

    return cdata;
}

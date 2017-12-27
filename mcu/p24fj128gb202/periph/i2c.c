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

#include <xc.h>
#include "core_timer.h"
#include "mcu.h"
#include "periph/i2c.h"
#include "periph_conf.h"

#define I2CxRCV(I)          (base_address[I][0x0 / 0x2])
#define I2CxTRN(I)          (base_address[I][0x2 / 0x2])
#define I2CxBRG(I)          (base_address[I][0x4 / 0x2])
#define I2CxCONL(I)         (base_address[I][0x6 / 0x2])
#define I2CxCONH(I)         (base_address[I][0x8 / 0x2])
#define I2CxSTAT(I)         (base_address[I][0xA / 0x2])
#define WRITE_MODE              (0)
#define READ_MODE               (1)

static volatile uint16_t *base_address[I2C_COUNT] = {
    &I2C1RCV,
    &I2C2RCV
};

/**
 * @brief Check if operation timed out
 *
 * @retval 0 Not timed out
 * @retval 1 Has timed out
 */
static int has_timed_out(uint32_t start, uint32_t deadline)
{
    uint32_t now;
    int ret = 1;

    if (start == deadline)
        return 0;

    /*
     * Three cases need to be considered:
     *
     * |---S---N-D-----|
     * |-----D-----S-N-|
     * |--N--D-----S---|
     *
     * S: start
     * D: deadline
     * N: now
     */
    now = core_timer_get_ticks();
    if ((start < deadline && now < deadline)
    ||  (start > deadline && (now > start || now < deadline)))
        ret = 0;

    return ret;
}

/**
 * @brief Wait that the bus is idle
 *
 * @param[in] i2c_num
 * @param[in] start
 * @param[in] deadline
 * @retval 0 if successful
 * @retval -2 if it timed out
 */
static int wait_for_idle_bus(uint8_t i2c_num, uint32_t start, uint32_t deadline)
{
    while (I2CxCONL(i2c_num) &
           (_I2C1CONL_SEN_MASK
            | _I2C1CONL_PEN_MASK
            | _I2C1CONL_RSEN_MASK
            | _I2C1CONL_RCEN_MASK
            | _I2C1CONL_ACKEN_MASK)
           || (I2CxSTAT(i2c_num) & _I2C1STAT_TRSTAT_MASK)) {
        if (has_timed_out(start, deadline))
            return I2C_ERROR;
    }

    return I2C_OK;
}

/**
 * @brief Send one byte
 *
 * @param[in] i2c_num
 * @param[in] data
 * @param[in] start
 * @param[in] deadline
 * @retval 0 if successful
 * @retval -1 if an error occured (bus collision or NACK receveid)
 * @retval -2 if timed out
 */
static int send_byte(uint8_t i2c_num, uint8_t data,
                     uint32_t start, uint32_t deadline)
{
    int ret;

    /* Start transmission */
    I2CxTRN(i2c_num) = data;

    ret = wait_for_idle_bus(i2c_num, start, deadline);
    if (ret < 0)
        return ret;

    while (I2CxSTAT(i2c_num) & _I2C1STAT_TBF_MASK) {
        if (has_timed_out(start, deadline))
            return I2C_TIMEOUT;
    }

    if (I2CxSTAT(i2c_num) & _I2C1STAT_BCL_MASK          /* Collision detected */
    || I2CxSTAT(i2c_num) & _I2C1STAT_ACKSTAT_MASK)      /* NACK received */
        return I2C_ERROR;

    return I2C_OK;
}

/**
 * @brief Receive one byte
 *
 * @param[in] i2c_num
 * @param[in] data
 * @param[in] nak 0: send ACK, 1: send NACK
 * @param[in] start
 * @param[in] deadline
 * @retval 0 if successful
 * @retval -1 if an error occured
 * @retval -2 it it timed out
 */
static int receive_byte(uint8_t i2c_num, uint8_t *data, uint8_t nak,
                        uint32_t start, uint32_t deadline)
{
    int ret;

    I2CxCONL(i2c_num) |= _I2C1CONL_RCEN_MASK;

    ret = wait_for_idle_bus(i2c_num, start, deadline);
    if (ret < 0)
        return ret;

    /* Wait for some data in RX FIFO */
    while (!(I2CxSTAT(i2c_num) & _I2C1STAT_RBF_MASK)) {
        if (has_timed_out(start, deadline))
            return I2C_TIMEOUT;
    }

    /* Check for a collision */
    if (I2CxSTAT(i2c_num) & _I2C1STAT_BCL_MASK)
        return I2C_ERROR;

    /* Send ACK/NAK */
    if (nak)
        I2CxCONL(i2c_num) |= _I2C1CONL_ACKDT_MASK;
    else
        I2CxCONL(i2c_num) &= ~_I2C1CONL_ACKDT_MASK;
    I2CxCONL(i2c_num) |= _I2C1CONL_ACKEN_MASK;
    ret = wait_for_idle_bus(i2c_num, start, deadline);
    if (ret < 0)
        return ret;

    *data = I2CxRCV(i2c_num);
    return I2C_OK;
}

/**
 * @brief Send 7-bit address & read/write bit
 *
 * @param[in] i2c_num
 * @param[in] address
 * @param[in] read_byte 0: write, 1: read
 * @param[in] start
 * @param[in] deadline
 * @retval 0 if successful
 * @retval -2 if it timed out
 */
static int send_address(uint8_t i2c_num, uint8_t address, uint8_t read_byte,
                        uint32_t start, uint32_t deadline)
{
    uint8_t tmp = (address << 1) | (read_byte & 0x1);

    return send_byte(i2c_num, tmp, start, deadline);
}

/**
 * @brief Send start condition
 *
 * @param[in] i2c_num
 * @param[in] start
 * @param[in] deadline
 * @retval 0 if successful
 * @retval -2 if it timed out
 */
static int send_start(uint8_t i2c_num, uint32_t start, uint32_t deadline)
{
    I2CxCONL(i2c_num) |= _I2C1CONL_SEN_MASK;
    while (I2CxCONL(i2c_num) & _I2C1CONL_SEN_MASK) {
        if (has_timed_out(start, deadline))
            return I2C_TIMEOUT;
    }
    return I2C_OK;
}

/**
 * @brief Send stop condition
 *
 * @param[in] i2c_num
 * @param[in] start
 * @param[in] deadline
 * @retval 0 if successful
 * @retval -2 if it timed out
 */
static int send_stop(uint8_t i2c_num, uint32_t start, uint32_t deadline)
{
    I2CxCONL(i2c_num) |= _I2C1CONL_PEN_MASK;
    while (I2CxCONL(i2c_num) & _I2C1CONL_PEN_MASK) {
        if (has_timed_out(start, deadline))
            return I2C_TIMEOUT;
    }
    return I2C_OK;
}

void i2c_configure(unsigned int i2c_num, uint32_t speed)
{
    I2CxCONL(i2c_num) = 0;
    I2CxSTAT(i2c_num) = 0;
    I2CxBRG(i2c_num) = ((mcu_get_system_clock() >> 1) / (2 * speed)) - 2;
}

void i2c_enable(unsigned int i2c_num)
{
    I2CxCONL(i2c_num) |= _I2C1CONL_I2CEN_MASK;
}

void i2c_disable(unsigned int i2c_num)
{
    I2CxCONL(i2c_num) &= ~_I2C1CONL_I2CEN_MASK;
}

uint32_t i2c_get_speed(unsigned int i2c_num)
{
    uint32_t speed = mcu_get_system_clock() >> 1;
    speed /= I2CxBRG(i2c_num) + 2;
    return speed;
}

int i2c_write(unsigned int i2c_num, uint8_t address, const void *buffer, uint32_t length)
{
    return i2c_write_safe(i2c_num, address, buffer, length, 0);
}

int i2c_write_safe(unsigned int i2c_num, uint8_t address, const void *buffer, uint32_t length, uint16_t timeout)
{
    int ret = I2C_OK;
    const uint8_t *data = (const uint8_t *)buffer;
    const uint8_t *end = data + length;
    const uint32_t start = core_timer_get_ticks();
    const uint32_t deadline = start + timeout;

    ret = send_start(i2c_num, start, deadline);
    if (ret < 0)
        goto i2c_write_end;

    ret = send_address(i2c_num, address, WRITE_MODE, start, deadline);
    if (ret < 0)
        goto i2c_write_end;

    while (data != end) {
        ret = send_byte(i2c_num, *data++, start, deadline);
        if (ret < 0)
            goto i2c_write_end;
    }

i2c_write_end:
    if (ret < 0)            /* Do not overwrite ret */
        send_stop(i2c_num, start, deadline);
    else
        ret = send_stop(i2c_num, start, deadline);
    return ret;
}

int i2c_read(unsigned int i2c_num, uint8_t address, void *buffer, uint32_t length)
{
    return i2c_read_safe(i2c_num, address, buffer, length, 0);
}

int i2c_read_safe(unsigned int i2c_num, uint8_t address, void *buffer, uint32_t length, uint16_t timeout)
{
    int ret = I2C_OK;
    uint8_t *data = (uint8_t *)buffer;
    const uint8_t *end = data + length;
    const uint32_t start = core_timer_get_ticks();
    const uint32_t deadline = start + timeout;

    ret = send_start(i2c_num, start, deadline);
    if (ret < 0)
        goto i2c_read_end;

    ret = send_address(i2c_num, address, READ_MODE, start, deadline);
    if (ret < 0)
        goto i2c_read_end;

    while (data != end) {
        uint8_t nak = 0;
        if (data + 1 == end)
            nak = 1;

        ret = receive_byte(i2c_num, data++, nak,
                           start, deadline);
        if (ret < 0)
            goto i2c_read_end;
    }

i2c_read_end:
    if (ret < 0)            /* Do not overwrite ret */
        send_stop(i2c_num, start, deadline);
    else
        ret = send_stop(i2c_num, start, deadline);

    return ret;
}

void i2c_power_up(unsigned int i2c_num)
{
    switch (i2c_num) {
    case I2C_1:
        PMD1 &= ~_PMD1_I2C1MD_MASK;
        break;
    case I2C_2:
        PMD3 &= ~_PMD3_I2C2MD_MASK;
        break;
    }
}

void i2c_power_down(unsigned int i2c_num)
{
    switch (i2c_num) {
    case I2C_1:
        PMD1 |= _PMD1_I2C1MD_MASK;
        break;
    case I2C_2:
        PMD3 |= _PMD3_I2C2MD_MASK;
        break;
    }
}

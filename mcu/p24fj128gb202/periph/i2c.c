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

static void wait_for_idle_bus(uint8_t i2c_num)
{
    while (I2CxCONL(i2c_num) &
           (_I2C1CONL_SEN_MASK
            | _I2C1CONL_PEN_MASK
            | _I2C1CONL_RSEN_MASK
            | _I2C1CONL_RCEN_MASK
            | _I2C1CONL_ACKEN_MASK)
           || (I2CxSTAT(i2c_num) & _I2C1STAT_TRSTAT_MASK)) {
    }
}

static int send_byte(uint8_t i2c_num, uint8_t data)
{
    I2CxTRN(i2c_num) = data;
    wait_for_idle_bus(i2c_num);
    while (I2CxSTAT(i2c_num) & _I2C1STAT_TBF_MASK)
        ;

    if (I2CxSTAT(i2c_num) & _I2C1STAT_BCL_MASK          /* Collision detected */
    || I2CxSTAT(i2c_num) & _I2C1STAT_ACKSTAT_MASK)      /* NACK received */
        return 0;

    return 1;
}

static int receive_byte(uint8_t i2c_num, uint8_t *data, uint8_t nak)
{
    I2CxCONL(i2c_num) |= _I2C1CONL_RCEN_MASK;
    wait_for_idle_bus(i2c_num);

    /* Wait for some data in RX FIFO */
    while (!(I2CxSTAT(i2c_num) & _I2C1STAT_RBF_MASK))
        ;

    /* Check for a collision */
    if (I2CxSTAT(i2c_num) & _I2C1STAT_BCL_MASK)
        return 0;

    /* Send ACK/NAK */
    if (nak)
        I2CxCONL(i2c_num) |= _I2C1CONL_ACKDT_MASK;
    else
        I2CxCONL(i2c_num) &= ~_I2C1CONL_ACKDT_MASK;
    I2CxCONL(i2c_num) |= _I2C1CONL_ACKEN_MASK;
    wait_for_idle_bus(i2c_num);

    *data = I2CxRCV(i2c_num);
    return 1;
}

static int send_address(uint8_t i2c_num, uint8_t address, uint8_t read_byte)
{
    uint8_t tmp = (address << 1) | (read_byte & 0x1);

    return send_byte(i2c_num, tmp);
}

static void send_start(uint8_t i2c_num)
{
    I2CxCONL(i2c_num) |= _I2C1CONL_SEN_MASK;
    while (I2CxCONL(i2c_num) & _I2C1CONL_SEN_MASK)
        ;
}

static void send_stop(uint8_t i2c_num)
{
    I2CxCONL(i2c_num) |= _I2C1CONL_PEN_MASK;
    while (I2CxCONL(i2c_num) & _I2C1CONL_PEN_MASK)
        ;
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

int i2c_write(unsigned int i2c_num, uint8_t address, const void *buffer, uint32_t length)
{
    int ret = 0;
    const uint8_t *data = (const uint8_t *)buffer;
    const uint8_t *end = data + length;

    send_start(i2c_num);

    if (send_address(i2c_num, address, WRITE_MODE) != 1) {
        ret = -1;
        goto i2c_write_end;
    }

    while (data != end) {
        if (send_byte(i2c_num, *data++) != 1) {
            ret = -1;
            goto i2c_write_end;
        }
    }

i2c_write_end:
    send_stop(i2c_num);
    return ret;
}

int i2c_read(unsigned int i2c_num, uint8_t address, void *buffer, uint32_t length)
{
    int ret = 0;
    uint8_t *data = (uint8_t *)buffer;
    uint32_t byte_received_count = 0;

    send_start(i2c_num);

    if (send_address(i2c_num, address, READ_MODE) != 1) {
        ret = -1;
        goto i2c_read_end;
    }

    while (byte_received_count < length) {
        if (receive_byte(i2c_num,
                         &data[byte_received_count],
                         (byte_received_count + 1) == length) != 1) {
            ret = -1;
            goto i2c_read_end;
        }
        ++byte_received_count;
    }

i2c_read_end:
    send_stop(i2c_num);
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

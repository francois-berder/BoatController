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
#include "periph/uart.h"
#include "periph_conf.h"

#define UxMODE(U)       (base_address[U][0x0 / 0x2])
#define UxSTA(U)        (base_address[U][0x2 / 0x2])
#define UxTXREG(U)      (base_address[U][0x4 / 0x2])
#define UxRXREG(U)      (base_address[U][0x6 / 0x2])
#define UxBRG(U)        (base_address[U][0x8 / 0x2])

static volatile uint16_t * const base_address[UART_COUNT] = {
    &U1MODE,
    &U2MODE,
    &U3MODE,
    &U4MODE
};

/**
 * @brief Compute UxBRG and set BRGH if needed.
 *
 * @param[in] uart_num
 * @param[in] baudrate
 * @return -1 if the baudrate error is greater than 3%, 0 otherwise
 */
static int compute_divisor(unsigned int uart_num, uint32_t baudrate)
{
    uint32_t divisor0, divisor1;
    uint32_t baudrate0, baudrate1;
    uint32_t error0, error1;
    int use_brgh = 0;
    const uint32_t pclk = mcu_get_system_clock() >> 1;

    /* Compute divisor with BRGH=0 */
    divisor0 = (pclk / (16 * baudrate)) - 1;
    if (divisor0 > UINT16_MAX)
        divisor0 = UINT16_MAX;

    /* Compute divisor with BRGH=1 */
    divisor1 = (pclk / (4 * baudrate)) - 1;
    if (divisor1 > UINT16_MAX)
        divisor1 = UINT16_MAX;

    /* Compute error */
    baudrate0 = pclk / (16 * (divisor0 + 1));
    baudrate1 = pclk / (4 * (divisor1 + 1));
    if (baudrate > baudrate0)
        error0 = baudrate - baudrate0;
    else
        error0 = baudrate0 - baudrate;

    if (baudrate > baudrate1)
        error1 = baudrate - baudrate1;
    else
        error1 = baudrate1 - baudrate;

    use_brgh = error1 < error0;

    /* Check if error is less than 3%. This uses the fact that 1/32 ~ 3% */
    if (use_brgh) {
        if ((error1 << 5) >= baudrate)
            return -1;
    } else if ((error0 << 5) >= baudrate) {
        return -1;
    }

    /* Configure UART module */
    if (use_brgh) {
        UxMODE(uart_num) |= _U1MODE_BRGH_MASK;
        UxBRG(uart_num) = divisor1;
    } else {
        UxBRG(uart_num) = divisor0;
    }

    return 0;
}

int uart_configure(unsigned int uart_num, uint32_t baudrate)
{
    UxMODE(uart_num) = 0;
    UxSTA(uart_num) = 0;
    return compute_divisor(uart_num, baudrate);
}

void uart_enable(unsigned int uart_num)
{
    UxMODE(uart_num) |= _U1MODE_UARTEN_MASK;
    UxSTA(uart_num) |= _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK;
}

void uart_disable(unsigned int uart_num)
{
    UxMODE(uart_num) &= ~_U1MODE_UARTEN_MASK;
    UxSTA(uart_num) &= ~(_U1STA_URXEN_MASK | _U1STA_UTXEN_MASK);
}

void uart_write(unsigned int uart_num, const void *buffer, uint32_t length)
{
    const uint8_t *data = (const uint8_t *)buffer;
    const uint8_t *end = data + length;

    while (data != end) {
        /* Wait until there is some space in TX FIFO */
        while (UxSTA(uart_num) & _U1STA_UTXBF_MASK)
            ;
        UxTXREG(uart_num) = *data++;
    }
}

void uart_read(unsigned int uart_num, void *buffer, uint32_t length)
{
    uint8_t *data = (uint8_t *)buffer;
    uint8_t *end = data + length;

    while (data != end) {
        while (!(UxSTA(uart_num) & _U1STA_URXDA_MASK))
            ;
        *data++ = UxRXREG(uart_num);
    }
}

uint32_t uart_read_noblock(unsigned int uart_num, void *buffer, uint32_t length)
{
    uint8_t *data = (uint8_t *)buffer;
    uint32_t byte_read_count = 0;

    while (byte_read_count < length) {
        if (UxSTA(uart_num) & _U1STA_URXDA_MASK)
            data[byte_read_count] = UxRXREG(uart_num);
        else
            break;
    }

    return byte_read_count;
}

void uart_power_up(unsigned int uart_num)
{
    switch (uart_num) {
    case UART_1:
        PMD1 &= ~_PMD1_U1MD_MASK;
        break;
    case UART_2:
        PMD1 &= ~_PMD1_U2MD_MASK;
        break;
    case UART_3:
        PMD3 &= ~_PMD3_U3MD_MASK;
        break;
    case UART_4:
        PMD4 &= ~_PMD4_U4MD_MASK;
        break;
    }
}

void uart_power_down(unsigned int uart_num)
{
    switch (uart_num) {
    case UART_1:
        PMD1 |= _PMD1_U1MD_MASK;
        break;
    case UART_2:
        PMD1 |= _PMD1_U2MD_MASK;
        break;
    case UART_3:
        PMD3 |= _PMD3_U3MD_MASK;
        break;
    case UART_4:
        PMD4 |= _PMD4_U4MD_MASK;
        break;
    }
}

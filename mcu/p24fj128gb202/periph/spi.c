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
#include "periph/spi.h"
#include "periph_conf.h"

#define SPIxCON1L(S)        (base_address[S][0x0 / 0x2])
#define SPIxCON1H(S)        (base_address[S][0x2 / 0x2])
#define SPIxSTATL(S)        (base_address[S][0x8 / 0x2])
#define SPIxBUFL(S)         (base_address[S][0xC / 0x2])
#define SPIxBRGL(S)         (base_address[S][0x10 / 0x2])

static volatile uint16_t * const base_address[SPI_COUNT] = {
    &SPI1CON1L,
    &SPI2CON1L,
    &SPI3CON1L
};

void spi_configure(unsigned int spi_num, uint32_t frequency, enum SPI_MODE mode)
{
    SPIxCON1L(spi_num) = _SPI1CON1L_MSTEN_MASK | _SPI1CON1L_ENHBUF_MASK;

    switch (mode) {
    case SPI_MODE_0:
        SPIxCON1L(spi_num) |= _SPI1CON1L_CKE_MASK;
        SPIxCON1L(spi_num) &= ~_SPI1CON1L_CKP_MASK;
        break;
    case SPI_MODE_1:
        SPIxCON1L(spi_num) &= ~(_SPI1CON1L_CKE_MASK | _SPI1CON1L_CKP_MASK);
        break;
    case SPI_MODE_2:
        SPIxCON1L(spi_num) |= (_SPI1CON1L_CKE_MASK | _SPI1CON1L_CKP_MASK);
        break;
    case SPI_MODE_3:
        SPIxCON1L(spi_num) &= ~_SPI1CON1L_CKE_MASK;
        SPIxCON1L(spi_num) |= _SPI1CON1L_CKP_MASK;
        break;
    }

    SPIxCON1H(spi_num) = 0;
    SPIxSTATL(spi_num) &= ~_SPI1STATL_SPIROV_MASK;
    SPIxBRGL(spi_num) = ((mcu_get_system_clock() >> 1) / (2 * frequency)) - 1;
}


void spi_enable(unsigned int spi_num)
{
    SPIxCON1L(spi_num) |= _SPI1CON1L_SPIEN_MASK;
}

void spi_disable(unsigned int spi_num)
{
    SPIxCON1L(spi_num) &= ~_SPI1CON1L_SPIEN_MASK;
}

void spi_transfer(unsigned int spi_num, const void *tx_buffer, void *rx_buffer, uint32_t length)
{
    const uint8_t *tx = (const uint8_t *)tx_buffer;
    uint8_t *rx = (uint8_t*)rx_buffer;
    uint32_t bytes_transfered_count = 0;

    while (bytes_transfered_count < length) {
        uint8_t in, out;

        if (tx)
            out = *tx++;
        else
            out = 0xFF;

        while (SPIxSTATL(spi_num) & _SPI1STATL_SPITBF_MASK)
            ;
        SPIxBUFL(spi_num) = out;

        while (SPIxSTATL(spi_num) & _SPI1STATL_SPIRBE_MASK)
            ;
        in = SPIxBUFL(spi_num);

        if (rx) {
            *rx++ = in;
        }

        ++bytes_transfered_count;
    }
}

void spi_fast_read(unsigned int spi_num, void *rx_buffer, uint32_t length)
{
    uint8_t *rx = (uint8_t*)rx_buffer;

    while (length) {
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;
        SPIxBUFL(spi_num) = 0xFF;

        while (!(SPIxSTATL(spi_num) & _SPI1STATL_SPIRBF_MASK))
            ;

        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);
        *rx++ = SPIxBUFL(spi_num);

        length -= 32;
    }
}

void spi_fast_write(unsigned int spi_num, const void *tx_buffer, uint32_t length)
{
    uint8_t *tx = (uint8_t*)tx_buffer;

    while (length) {

        while (!(SPIxSTATL(spi_num) & _SPI1STATL_SPITBE_MASK))
            ;

        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;
        SPIxBUFL(spi_num) = *tx++;

        length -= 32;
    }

    SPIxSTATL(spi_num) &= ~_SPI1STATL_SPIROV_MASK;
    while (!(SPIxSTATL(spi_num) & _SPI1STATL_SPIRBE_MASK)) {
        (void)SPIxBUFL(spi_num);
    }
}

void spi_power_up(unsigned int spi_num)
{
    switch (spi_num) {
    case SPI_1:
        PMD1 &= ~_PMD1_SPI1MD_MASK;
        break;
    case SPI_2:
        PMD1 &= ~_PMD1_SPI2MD_MASK;
        break;
    case SPI_3:
        PMD6 &= ~_PMD6_SPI3MD_MASK;
        break;
    }
}

void spi_power_down(unsigned int spi_num)
{
    switch (spi_num) {
    case SPI_1:
        PMD1 |= _PMD1_SPI1MD_MASK;
        break;
    case SPI_2:
        PMD1 |= _PMD1_SPI2MD_MASK;
        break;
    case SPI_3:
        PMD6 |= _PMD6_SPI3MD_MASK;
        break;
    }
}

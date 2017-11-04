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


#ifndef __MCU_SPI_H__
#define __MCU_SPI_H__

#include <stdint.h>

/**
 * @defgroup spi SPI peripheral
 * @{
 */

/** Define SPI modes */
enum SPI_MODE {
    SPI_MODE_0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3
};

/**
 * @brief Configure a SPI module
 *
 * The module must be powered up before calling this function.
 * This function does not enable the SPI module.
 *
 * @param[in] spi_num Index of the SPI module
 * @param[in] frequency Clock frequency in Hz
 * @param[in] mode SPI mode (see #SPI_MODE)
 */
void spi_configure(unsigned int spi_num, uint32_t frequency, enum SPI_MODE mode);

/**
 * @brief Enable SPI module
 *
 * @param[in] spi_num Index of the SPI module
 */
void spi_enable(unsigned int spi_num);

/**
 * @brief Disable SPI module
 *
 * @param[in] spi_num Index of the SPI module
 */
void spi_disable(unsigned int spi_num);

/**
 * @brief Make a SPI transfer
 *
 * Both tx_buffer and rx_buffer can be NULL. If tx_buffer is NULL, 0xFF bytes are sent.
 * The SPI module must be initialised before calling this function.
 *
 * @param[in] spi_num Index of the SPI module
 * @param[out] tx_buffer Array of bytes to send
 * @param[in] rx_buffer Array of bytes to receive
 * @param[in] length Number of bytes to receive/send
 */
void spi_transfer(unsigned int spi_num, const void *tx_buffer, void *rx_buffer, uint32_t length);

/**
 * @brief Make a fast read from SPI
 *
 * @param[in] spi_num Index of the SPI module
 * @param[in] rx_buffer Array of bytes to receive
 * @param[in] length Number of bytes to receive/send, must be a multiple of 32
 */
void spi_fast_read(unsigned int spi_num, void *rx_buffer, uint32_t length);

/**
 * @brief Make a fast read from SPI
 *
 * @param[in] spi_num Index of the SPI module
 * @param[in] tx_buffer Array of bytes to send
 * @param[in] length Number of bytes to receive/send, must be a multiple of 32
 */
void spi_fast_write(unsigned int spi_num, const void *tx_buffer, uint32_t length);

/**
 * @brief Power-up a SPI module
 *
 * @param[in] spi_num Index of the SPI module
 */
void spi_power_up(unsigned int spi_num);

/**
 * @brief Power-down a SPI module
 *
 * The module must be disabled before calling this function.
 * Once the module is powered down, do not call any other functions
 * except spi_power_up.
 *
 * @param[in] spi_num Index of the SPI module
 */
void spi_power_down(unsigned int spi_num);

/** @} */

#endif

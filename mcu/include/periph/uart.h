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


#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

/**
 * @defgroup uart UART peripheral
 * @{
 */

/** Define all baudrates */
enum UART_BAUDRATE {
    UART_BD_1200 = 1200,
    UART_BD_4800 = 4800,
    UART_BD_9600 = 9600,
    UART_BD_19200 = 19200,
    UART_BD_38400 = 38400,
    UART_BD_57600 = 57600,
    UART_BD_115200 = 115200
};

/**
 * @brief Initialise a UART module
 *
 * The module must be powered up before initialisation.
 * This function does not enable the module.
 * The UART must use 8 bit data, no parity, and 1 stop bit.
 *
 * @param[in] uart_num Index of the UART module
 * @param[in] baudrate
 * @retval 0 if the module was initialised with success
 * @retval -1 if baudrate can only be achieved with an error greater than 3%
 */
int uart_configure(unsigned int uart_num, uint32_t baudrate);

/**
 * @brief Enable UART module
 *
 * @param[in] uart_num Index of the UART module
 */
void uart_enable(unsigned int uart_num);

/**
 * @brief Disable UART module
 *
 * @param[in] uart_num Index of the UART module
 */
void uart_disable(unsigned int uart_num);

/**
 * @brief Send some bytes over UART
 *
 * Note that the module must be initialised before calling
 * this function.
 *
 * This function is blocking: it does not return until all bytes are
 * sent.
 *
 * @param[in] uart_num Index of the UART module
 * @param[in] buffer Pointer to an array of bytes (must not be null)
 * @param[in] length Number of bytes to send
 */
void uart_write(unsigned int uart_num, const void *buffer, uint32_t length);

/**
 * @brief Receive some bytes over UART
 *
 * Note that the module must be initialised before calling
 * this function.
 *
 * @param[in] uart_num Index of the UART module
 * @param[out] buffer Pointer to an array of bytes (must not be null)
 * @param[in] length Number of bytes to receive
 */
void uart_read(unsigned int uart_num, void *buffer, uint32_t length);

/**
 * @brief Receive some bytes over UART
 *
 * Note that the module must be initialised before calling
 * this function.
 * Unlike uart_read, this function does not block.
 *
 * @param[in] uart_num Index of the UART module
 * @param[out] buffer Pointer to an array of bytes (must not be null)
 * @param[in] length Number of bytes to receive
 * @return Number of bytes read, which can be less than @p length
 */
uint32_t uart_read_noblock(unsigned int uart_num, void *buffer, uint32_t length);

/**
 * @brief Power-up a UART module
 *
 * @param[in] uart_num Index of the UART module
 */
void uart_power_up(unsigned int uart_num);

/**
 * @brief Power-down a UART module
 *
 * The module must be disabled before calling this function
 * Once the module is powered down, do not call any functions
 * except uart_power_up.
 *
 * @param[in] uart_num Index of the UART module
 */
void uart_power_down(unsigned int uart_num);

/** @} */

#endif

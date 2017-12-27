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

#ifndef __MCU_I2C_H__
#define __MCU_I2C_H__

#include <stdint.h>

/**
 * @defgroup i2c I2C peripheral
 * @{
 */

/** Define I2C speeds */
enum I2C_SPEED {
    I2C_STANDARD_SPEED = 100000,        /**< Standard speed 100kHz */
    I2C_FAST_SPEED = 400000             /**< Fast speed 400kHz */
};

/**
 * @brief Initialise the I2C module
 *
 * The module must be powered up before calling this function.
 * This function does not enable the module.
 *
 * @param[in] i2c_num Index of the I2C module
 * @param[in] speed Frequency in Hz of the I2C bus (must not be 0)
 * @retval 0 if the I2C module was initialised with success
 * @retval -1 if it failed
 */
void i2c_configure(unsigned int i2c_num, uint32_t speed);

/**
 * @brief Enable I2C module
 *
 * @param[in] i2c_num Index of the I2C module
 */
void i2c_enable(unsigned int i2c_num);

/**
 * @brief Disable I2C module
 *
 * @param[in] i2c_num Index of the I2C module
 */
void i2c_disable(unsigned int i2c_num);

/**
 * @brief Send some bytes to a slave
 *
 * The I2C module must have been initialised before calling this function.
 *
 * @param[in] i2c_num Index of the I2C module
 * @param[in] address 7-bit address of the slave
 * @param[in] buffer Array of bytes to send
 * @param[in] length Number of bytes to send
 * @retval 0 if all bytes were sent with success
 * @retval -1 if it failed
 */
int i2c_write(unsigned int i2c_num, uint8_t address, const void *buffer, uint32_t length);

/**
 * @brief Send some bytes to a slave with timeout
 *
 * The I2C module must have been initialised before calling this function.
 *
 * @param[in] i2c_num Index of the I2C module
 * @param[in] address 7-bit address of the slave
 * @param[in] buffer Array of bytes to send
 * @param[in] length Number of bytes to send
 * @param[in] timeout Maximal number of ticks, write operation must take. If set to 0,
 * timeout is disabled
 * @retval 0 if all bytes were sent with success
 * @retval -1 if it failed
 * @retval -2 if it timed out
 */
int i2c_write_safe(unsigned int i2c_num, uint8_t address, const void *buffer, uint32_t length, uint16_t timeout);

/**
 * @brief Read some bytes from a slave
 *
 * The I2C module must have been initialised before calling this function.
 *
 * @param[in] i2c_num Index of the I2C module
 * @param[in] address 7-bit address of the slave
 * @param[in] buffer Array of bytes to receive
 * @param[in] length Number of bytes to receive
 * @retval 0 if all bytes were received with success
 * @retval -1 if it failed
 */
int i2c_read(unsigned int i2c_num, uint8_t address, void *buffer, uint32_t length);

/**
 * @brief Read some bytes from a slave
 *
 * The I2C module must have been initialised before calling this function.
 *
 * @param[in] i2c_num Index of the I2C module
 * @param[in] address 7-bit address of the slave
 * @param[in] buffer Array of bytes to receive
 * @param[in] length Number of bytes to receive
 * @param[in] timeout Maximal number of ticks read operation must take, if set to 0,
 * timeout is disabled
 * @retval 0 if all bytes were received with success
 * @retval -1 if it failed
 * @retval -2 if it timed out
 */
int i2c_read_safe(unsigned int i2c_num, uint8_t address, void *buffer, uint32_t length, uint16_t timeout);

/**
 * @brief Power-up a I2C module
 *
 * @param[in] i2c_num Index of the I2C module
 */
void i2c_power_up(unsigned int i2c_num);

/**
 * @brief Power-down a I2C module
 *
 * The module must be disabled before calling this function.
 *
 * Do not call any other functions except i2c_power_up after
 * powering down the module.
 *
 * @param[in] i2c_num Index of the I2C module
 */
void i2c_power_down(unsigned int i2c_num);

/** @} */

#endif

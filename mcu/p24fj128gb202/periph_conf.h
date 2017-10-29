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

#ifndef __PERIPH_CONF_H__
#define __PERIPH_CONF_H__

enum PIC24_PORT {
    PORT_A,
    PORT_B,
    PORT_COUNT
};

enum PIC24_UART {
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_COUNT
};

enum PIC24_TIMER1_PRESCALER {
    TIMER1_PRESCALER_1,
    TIMER1_PRESCALER_8,
    TIMER1_PRESCALER_64,
    TIMER1_PRESCALER_256
};

enum PIC24_TIMER {
    TIMER_2,
    TIMER_3,
    TIMER_4,
    TIMER_5,
    TIMER_COUNT
};

enum PIC24_TIMER2_PRESCALER {
    TIMER2_PRESCALER_1,
    TIMER2_PRESCALER_8,
    TIMER2_PRESCALER_64,
    TIMER2_PRESCALER_256
};

enum PIC24_TIMER3_PRESCALER {
    TIMER3_PRESCALER_1,
    TIMER3_PRESCALER_8,
    TIMER3_PRESCALER_64,
    TIMER3_PRESCALER_256
};

enum PIC24_TIMER4_PRESCALER {
    TIMER4_PRESCALER_1,
    TIMER4_PRESCALER_8,
    TIMER4_PRESCALER_64,
    TIMER4_PRESCALER_256
};

enum PIC24_TIMER5_PRESCALER {
    TIMER5_PRESCALER_1,
    TIMER5_PRESCALER_8,
    TIMER5_PRESCALER_64,
    TIMER5_PRESCALER_256
};

enum PIC24_SPI {
    SPI_1,
    SPI_2,
    SPI_3,
    SPI_COUNT
};

enum PIC24_I2C {
    I2C_1,
    I2C_2,
    I2C_COUNT
};

enum PIC24_IC {
    IC_1,
    IC_2,
    IC_3,
    IC_4,
    IC_5,
    IC_6,
    IC_COUNT
};

enum PIC24_PWM {
    PWM_1,
    PWM_2,
    PWM_3,
    PWM_4,
    PWM_5,
    PWM_6,
    PWM_COUNT
};

#endif

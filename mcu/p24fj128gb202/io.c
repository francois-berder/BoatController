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

#include "periph/uart.h"
#include "periph_conf.h"

/* By default, let's enable IO on UART */
#ifndef ENABLE_IO
#define ENABLE_IO (1)
#endif

#ifndef IO_UART
#define IO_UART UART_1
#endif

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len)
{
    switch (handle) {
    case 0:
    case 1:
    case 2:
#if ENABLE_IO
        uart_write(IO_UART, buffer, len);
#endif
        break;
    default:
        break;
    }

    return len;
}

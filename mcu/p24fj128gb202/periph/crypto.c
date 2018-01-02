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
#include "periph/crypto.h"

void crypto_get_random(void *dst, uint32_t length)
{
    uint8_t *buffer = (uint8_t *)dst;

    /* Configure crypto module to generate random numbers */
    CRYCONL &= ~_CRYCONL_OPMOD_MASK;
    CRYCONL |= 0b1010 << _CRYCONL_OPMOD_POSITION;

    while (length > 16) {
        CRYCONL |= _CRYCONL_CRYGO_MASK;
        while (CRYCONL & _CRYCONL_CRYGO_MASK)
            ;

        *buffer++ = CRYTXTA0;
        *buffer++ = CRYTXTA0 >> 8;
        *buffer++ = CRYTXTA1;
        *buffer++ = CRYTXTA1 >> 8;
        *buffer++ = CRYTXTA2;
        *buffer++ = CRYTXTA2 >> 8;
        *buffer++ = CRYTXTA3;
        *buffer++ = CRYTXTA3 >> 8;
        *buffer++ = CRYTXTA4;
        *buffer++ = CRYTXTA4 >> 8;
        *buffer++ = CRYTXTA5;
        *buffer++ = CRYTXTA5 >> 8;
        *buffer++ = CRYTXTA6;
        *buffer++ = CRYTXTA6 >> 8;
        *buffer++ = CRYTXTA7;
        *buffer++ = CRYTXTA7 >> 8;

        length -= 16;
    }

    if (length >= 2) {
        CRYCONL |= _CRYCONL_CRYGO_MASK;
        while (CRYCONL & _CRYCONL_CRYGO_MASK)
            ;

        *buffer++ = CRYTXTA0;
        *buffer++ = CRYTXTA0 >> 8;
        length -= 2;

        if (length >= 2) {
            *buffer++ = CRYTXTA1;
            *buffer++ = CRYTXTA1 >> 8;
            length -= 2;
        }
        if (length >= 2) {
            *buffer++ = CRYTXTA2;
            *buffer++ = CRYTXTA2 >> 8;
            length -= 2;
        }
        if (length >= 2) {
            *buffer++ = CRYTXTA3;
            *buffer++ = CRYTXTA3 >> 8;
            length -= 2;
        }
        if (length >= 2) {
            *buffer++ = CRYTXTA4;
            *buffer++ = CRYTXTA4 >> 8;
            length -= 2;
        }
        if (length >= 2) {
            *buffer++ = CRYTXTA5;
            *buffer++ = CRYTXTA5 >> 8;
            length -= 2;
        }
        if (length >= 2) {
            *buffer++ = CRYTXTA6;
            *buffer++ = CRYTXTA6 >> 8;
            length -= 2;
        }
    }

    if (length) {
        CRYCONL |= _CRYCONL_CRYGO_MASK;
        while (CRYCONL & _CRYCONL_CRYGO_MASK)
            ;
        *buffer++ = CRYTXTA0;
    }
}

void crypto_enable(void)
{
    CRYCONL |= _CRYCONL_CRYON_MASK;
}

void crypto_disable(void)
{
    CRYCONL &= ~_CRYCONL_CRYON_MASK;
}

void crypto_power_up(void)
{
    PMD8 &= ~_PMD8_CRYMD_MASK;
}

void crypto_power_down(void)
{
    PMD8 |= _PMD8_CRYMD_MASK;
}

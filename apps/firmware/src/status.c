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

#include "periph/gpio.h"
#include "periph/timer.h"
#include "periph_conf.h"
#include "status.h"

#define LED_PIN         (GPIO_PIN(PORT_B, 5))

static enum STATUS_MODE mode;
static uint8_t counter = 0;

void timer2_callback(void)
{
    ++counter;

    switch (mode) {
    case STATUS_OFF:
    case STATUS_ON:
        break;
    case STATUS_FLASH:
        if (counter == 49 && !gpio_read(LED_PIN)) {
            counter = 0;
            gpio_write(LED_PIN, 1);
        }
        else if (counter == 1 && gpio_read(LED_PIN)) {
            counter = 0;
            gpio_write(LED_PIN, 0);
        }
        break;
    case STATUS_FAST_BLINK:
        if (counter == 1) {
            counter = 0;
            gpio_toggle(LED_PIN);
        }
        break;
    case STATUS_ONE_PER_2SEC:
        gpio_write(LED_PIN, counter < 2);
        if (counter == 19)
            counter = 0;
        break;
    case STATUS_TWO_PER_2SEC:
        if (counter < 6 && counter != 2 && counter != 3)
            gpio_write(LED_PIN, 1);
        else
            gpio_write(LED_PIN, 0);

        if (counter == 19)
            counter = 0;
        break;
    case STATUS_THREE_PER_2SEC:
        if (counter < 10 && counter != 2 && counter != 3
        && counter != 6 && counter != 7)
            gpio_write(LED_PIN, 1);
        else
            gpio_write(LED_PIN, 0);

        if (counter == 19)
            counter = 0;
        break;
    }
}

void status_configure(void)
{
    counter = 0;
    mode = STATUS_OFF;
    gpio_init_out(LED_PIN, 0);

    /* Configure timer2 to trigger every 100ms */
    timer_configure(TIMER_2, TIMER2_PRESCALER_64, 6250, 1);
}

void status_set_mode(enum STATUS_MODE _mode)
{
    timer_stop(TIMER_2);

    mode = _mode;

    switch (mode) {
    case STATUS_OFF:
        gpio_write(LED_PIN, 0);
        break;
    case STATUS_ON:
        gpio_write(LED_PIN, 1);
        break;
    case STATUS_FLASH:
    case STATUS_FAST_BLINK:
    case STATUS_ONE_PER_2SEC:
    case STATUS_TWO_PER_2SEC:
    case STATUS_THREE_PER_2SEC:
        counter = 0;
        gpio_write(LED_PIN, 1);

        /* Start timer 2 only if we need it */
        timer_start(TIMER_2);
        break;
    }
}

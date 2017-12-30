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

#include "mcu.h"
#include "output.h"
#include "periph/gpio.h"
#include "periph/timer.h"
#include "periph_conf.h"

#define NEUTRAL_POS     (6000)

#define LEFT_RUDDER_PIN         (GPIO_PIN(PORT_B, 2))
#define RIGHT_RUDDER_PIN        (GPIO_PIN(PORT_B, 3))
#define LEFT_MOTOR_PIN          (GPIO_PIN(PORT_A, 2))
#define RIGHT_MOTOR_PIN         (GPIO_PIN(PORT_A, 3))

enum OUTPUT_STATE {
    FETCH_DATA,
    LEFT_RUDDER,
    RIGHT_RUDDER,
    LEFT_MOTOR,
    RIGHT_MOTOR
};

static volatile enum OUTPUT_STATE state;
static volatile struct output_frame_t current_frame, next_frame;

void timer3_callback(void)
{
    /* Move on to next state */
    ++state;
    if (state > RIGHT_MOTOR)
        state = FETCH_DATA;

    switch (state) {
    case FETCH_DATA:
        current_frame = next_frame;
        break;
    case LEFT_RUDDER:
        gpio_write(LEFT_RUDDER_PIN, 1);
        timer_configure(TIMER_4, TIMER4_PRESCALER_1, current_frame.left_rudder, 1);
        break;
    case RIGHT_RUDDER:
        gpio_write(RIGHT_RUDDER_PIN, 1);
        timer_configure(TIMER_4, TIMER4_PRESCALER_1, current_frame.right_rudder, 1);
        break;
    case LEFT_MOTOR:
        gpio_write(LEFT_MOTOR_PIN, 1);
        timer_configure(TIMER_4, TIMER4_PRESCALER_1, current_frame.left_motor, 1);
        break;
    case RIGHT_MOTOR:
        gpio_write(RIGHT_MOTOR_PIN, 1);
        timer_configure(TIMER_4, TIMER4_PRESCALER_1, current_frame.right_motor, 1);
        break;
    }

    if (state != FETCH_DATA) {
        timer_power_up(TIMER_4);
        timer_start(TIMER_4);
    }
}

void timer4_callback(void)
{
    switch (state) {
    case FETCH_DATA:
        break;
    case LEFT_RUDDER:
        gpio_write(LEFT_RUDDER_PIN, 0);
        break;
    case RIGHT_RUDDER:
        gpio_write(RIGHT_RUDDER_PIN, 0);
        break;
    case LEFT_MOTOR:
        gpio_write(LEFT_MOTOR_PIN, 0);
        break;
    case RIGHT_MOTOR:
        gpio_write(RIGHT_MOTOR_PIN, 0);
        break;
    }

    timer_stop(TIMER_4);
    timer_power_down(TIMER_4);
}

void output_configure(void)
{
    /* Ensure that no timers are running during configuration */
    timer_stop(TIMER_3);
    timer_stop(TIMER_4);

    /* Configure timer 3 to call callback every 4 ms */
    timer_power_up(TIMER_3);
    timer_configure(TIMER_3, TIMER3_PRESCALER_8, 2000, 1);

    state = RIGHT_MOTOR;

    current_frame.left_rudder = NEUTRAL_POS;
    current_frame.right_rudder = NEUTRAL_POS;
    current_frame.left_motor = NEUTRAL_POS;
    current_frame.right_motor = NEUTRAL_POS;
    next_frame = current_frame;

    gpio_init_out(LEFT_RUDDER_PIN, 0);
    gpio_init_out(RIGHT_RUDDER_PIN, 0);
    gpio_init_out(LEFT_MOTOR_PIN, 0);
    gpio_init_out(RIGHT_MOTOR_PIN, 0);
}

void output_enable(void)
{
    gpio_write(LEFT_RUDDER_PIN, 0);
    gpio_write(RIGHT_RUDDER_PIN, 0);
    gpio_write(LEFT_MOTOR_PIN, 0);
    gpio_write(RIGHT_MOTOR_PIN, 0);
    timer_start(TIMER_3);
}

void output_disable(void)
{
    timer_stop(TIMER_3);
    timer_stop(TIMER_4);

    gpio_write(LEFT_RUDDER_PIN, 0);
    gpio_write(RIGHT_RUDDER_PIN, 0);
    gpio_write(LEFT_MOTOR_PIN, 0);
    gpio_write(RIGHT_MOTOR_PIN, 0);
}

void output_set_frame(struct output_frame_t frame)
{
    mcu_disable_interrupts();
    next_frame = frame;
    mcu_enable_interrupts();
}

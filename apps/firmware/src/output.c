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

#include "config.h"
#include "mcu.h"
#include "output.h"
#include "periph/gpio.h"
#include "periph/timer.h"
#include "periph_conf.h"

#define NEUTRAL_POS     (6000)

enum OUTPUT_STATE {
    START_COMMAND_SERVO,
    END_COMMAND_SERVO,
    START_COMMAND_MOTOR,
    END_COMMAND_MOTOR
};

static volatile enum OUTPUT_STATE state;
static volatile struct output_frame_t current_frame, next_frame;

void timer3_callback(void)
{
    /* Move on to next state */
    ++state;
    if (state > END_COMMAND_MOTOR)
        state = START_COMMAND_SERVO;

    switch (state) {
    case START_COMMAND_SERVO:
        current_frame = next_frame;

        /* Configure timer 3 & 4 */
        timer_power_up(TIMER_4);
        timer_configure(TIMER_3, TIMER3_PRESCALER_1, current_frame.left_rudder, 1);
        timer_configure(TIMER_4, TIMER4_PRESCALER_1, current_frame.right_rudder, 1);
        timer_start(TIMER_3);
        timer_start(TIMER_4);

        gpio_write(LEFT_RUDDER_PIN, 1);
        gpio_write(RIGHT_RUDDER_PIN, 1);
        break;
    case END_COMMAND_SERVO:
        gpio_write(LEFT_RUDDER_PIN, 0);

        /* Configure timer 3 to trigger in 2.5ms - data0 */
        timer_configure(TIMER_3, TIMER3_PRESCALER_1, 10000 - current_frame.left_rudder, 1);
        timer_start(TIMER_3);
        break;
    case START_COMMAND_MOTOR:
        /* Configure timer 3 & 4 */
        timer_power_up(TIMER_4);
        timer_configure(TIMER_3, TIMER3_PRESCALER_1, current_frame.left_motor, 1);
        timer_configure(TIMER_4, TIMER4_PRESCALER_1, current_frame.right_motor, 1);
        timer_start(TIMER_3);
        timer_start(TIMER_4);

        gpio_write(LEFT_MOTOR_PIN, 1);
        gpio_write(RIGHT_MOTOR_PIN, 1);
        break;
    case END_COMMAND_MOTOR:
        gpio_write(LEFT_MOTOR_PIN, 0);

        /* Configure timer 3 to trigger in 17.5ms - data2 */
        timer_configure(TIMER_3, TIMER3_PRESCALER_8, 8750 - (current_frame.left_motor >> 3), 1);
        timer_start(TIMER_3);
        break;
    }
}

void timer4_callback(void)
{
    switch (state) {
    case START_COMMAND_SERVO:
    case END_COMMAND_SERVO:
        gpio_write(RIGHT_RUDDER_PIN, 0);
        break;
    case START_COMMAND_MOTOR:
    case END_COMMAND_MOTOR:
        gpio_write(RIGHT_MOTOR_PIN, 0);
        break;
    }

    timer_stop(TIMER_4);
}

void output_configure(void)
{
    /* Ensure that no timers are running during configuration */
    timer_stop(TIMER_3);
    timer_stop(TIMER_4);
    timer_power_up(TIMER_3);
    timer_power_up(TIMER_4);

    current_frame.left_rudder = NEUTRAL_POS;
    current_frame.right_rudder = NEUTRAL_POS;
    current_frame.left_motor = NEUTRAL_POS;
    current_frame.right_motor = NEUTRAL_POS;
    next_frame = current_frame;
}

void output_enable(void)
{
    timer_stop(TIMER_3);
    timer_stop(TIMER_4);

    state = END_COMMAND_MOTOR;

    gpio_write(LEFT_RUDDER_PIN, 0);
    gpio_write(RIGHT_RUDDER_PIN, 0);
    gpio_write(LEFT_MOTOR_PIN, 0);
    gpio_write(RIGHT_MOTOR_PIN, 0);

    /* Configure timer 3 to trigger as soon as it starts */
    timer_configure(TIMER_3, TIMER3_PRESCALER_1, 1, 1);
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
    unsigned int ctx = mcu_save_context();
    next_frame = frame;
    mcu_restore_context(ctx);
}

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
#include "periph/pwm.h"
#include "periph/timer.h"
#include "periph_conf.h"

#define OCxCON1(O)          (base_address[O][0x0 / 0x2])
#define OCxCON2(O)          (base_address[O][0x2 / 0x2])
#define OCxRS(O)            (base_address[O][0x4 / 0x2])
#define OCxR(O)             (base_address[O][0x6 / 0x2])

static volatile uint16_t * const base_address[PWM_COUNT] = {
    &OC1CON1,
    &OC2CON1,
    &OC3CON1,
    &OC4CON1,
    &OC5CON1,
    &OC6CON1
};

void pwm_configure(unsigned int pwm_num, unsigned int timer_num)
{
    OCxCON1(pwm_num) = (timer_num << _OC1CON1_OCTSEL_POSITION) & _OC1CON1_OCTSEL_MASK;
    OCxCON2(pwm_num) = 0b11111;
    OCxR(pwm_num) = 0;

    switch (timer_num) {
    case TIMER_2:
        OCxRS(pwm_num) = PR2;
        break;
    case TIMER_3:
        OCxRS(pwm_num) = PR3;
        break;
    case TIMER_4:
        OCxRS(pwm_num) = PR4;
        break;
    case TIMER_5:
        OCxRS(pwm_num) = PR5;
        break;
    }
}

void pwm_enable(unsigned int pwm_num)
{
    /* Edge-aligned PWM */
    OCxCON1(pwm_num) &= ~_OC1CON1_OCM_MASK;
    OCxCON1(pwm_num) |= 0b110 << _OC1CON1_OCM_POSITION;
}

void pwm_disable(unsigned int pwm_num)
{
    OCxCON1(pwm_num) &= ~_OC1CON1_OCM_MASK;
}

uint32_t pwm_get_period(unsigned int pwm_num)
{
    unsigned int timer_num = (OCxCON1(pwm_num) & _OC1CON1_OCTSEL_MASK) >> _OC1CON1_OCTSEL_POSITION;
    return timer_get_period(timer_num);
}

uint32_t pwm_get_duty_cycle(unsigned int pwm_num)
{
    uint64_t duty_cycle = OCxR(pwm_num);
    unsigned int timer_num = (OCxCON1(pwm_num) & _OC1CON1_OCTSEL_MASK) >> _OC1CON1_OCTSEL_POSITION;
    unsigned int prescaler = 0;

    switch (timer_num) {
    case TIMER_2:
        prescaler = (T2CON & _T2CON_TCKPS_MASK) >> _T2CON_TCKPS_POSITION;
        break;
    case TIMER_3:
        prescaler = (T3CON & _T3CON_TCKPS_MASK) >> _T3CON_TCKPS_POSITION;
        break;
    case TIMER_4:
        prescaler = (T4CON & _T4CON_TCKPS_MASK) >> _T4CON_TCKPS_POSITION;
        break;
    case TIMER_5:
        prescaler = (T5CON & _T5CON_TCKPS_MASK) >> _T5CON_TCKPS_POSITION;
        break;
    }

    switch (prescaler) {
    case TIMER2_PRESCALER_1:
        break;
    case TIMER2_PRESCALER_8:
        duty_cycle <<= 3;
        break;
    case TIMER2_PRESCALER_64:
        duty_cycle <<= 6;
        break;
    case TIMER2_PRESCALER_256:
        duty_cycle <<= 8;
        break;
    }

    /* Convert duty cycles to nanoseconds */
    duty_cycle *= 1000;
    duty_cycle *= 1000;
    duty_cycle *= 1000;

    duty_cycle /= (mcu_get_system_clock() >> 1);

    return duty_cycle;
}

void pwm_set_duty_cycle(unsigned int pwm_num, uint32_t _duty_cycle)
{
    uint64_t duty_cycle = _duty_cycle;
    unsigned int timer_num = (OCxCON1(pwm_num) & _OC1CON1_OCTSEL_MASK) >> _OC1CON1_OCTSEL_POSITION;
    unsigned int prescaler = 0;

    duty_cycle *= (mcu_get_system_clock() >> 1);

    switch (timer_num) {
    case TIMER_2:
        prescaler = (T2CON & _T2CON_TCKPS_MASK) >> _T2CON_TCKPS_POSITION;
        break;
    case TIMER_3:
        prescaler = (T3CON & _T3CON_TCKPS_MASK) >> _T3CON_TCKPS_POSITION;
        break;
    case TIMER_4:
        prescaler = (T4CON & _T4CON_TCKPS_MASK) >> _T4CON_TCKPS_POSITION;
        break;
    case TIMER_5:
        prescaler = (T5CON & _T5CON_TCKPS_MASK) >> _T5CON_TCKPS_POSITION;
        break;
    }

    switch (prescaler) {
    case TIMER2_PRESCALER_1:
        break;
    case TIMER2_PRESCALER_8:
        duty_cycle >>= 3;
        break;
    case TIMER2_PRESCALER_64:
        duty_cycle >>= 6;
        break;
    case TIMER2_PRESCALER_256:
        duty_cycle >>= 8;
        break;
    }

    duty_cycle /= 1000;
    duty_cycle /= 1000;
    duty_cycle /= 1000;

    if (duty_cycle > OCxRS(pwm_num))
        duty_cycle = OCxRS(pwm_num);

    OCxR(pwm_num) = duty_cycle;
}

void pwm_power_up(unsigned int pwm_num)
{
    switch (pwm_num) {
    case PWM_1:
        PMD2 &= ~_PMD2_OC1MD_MASK;
        break;
    case PWM_2:
        PMD2 &= ~_PMD2_OC2MD_MASK;
        break;
    case PWM_3:
        PMD2 &= ~_PMD2_OC3MD_MASK;
        break;
    case PWM_4:
        PMD2 &= ~_PMD2_OC4MD_MASK;
        break;
    case PWM_5:
        PMD2 &= ~_PMD2_OC5MD_MASK;
        break;
    case PWM_6:
        PMD2 &= ~_PMD2_OC6MD_MASK;
        break;
    }
}

void pwm_power_down(unsigned int pwm_num)
{
    switch (pwm_num) {
    case PWM_1:
        PMD2 |= _PMD2_OC1MD_MASK;
        break;
    case PWM_2:
        PMD2 |= _PMD2_OC2MD_MASK;
        break;
    case PWM_3:
        PMD2 |= _PMD2_OC3MD_MASK;
        break;
    case PWM_4:
        PMD2 |= _PMD2_OC4MD_MASK;
        break;
    case PWM_5:
        PMD2 |= _PMD2_OC5MD_MASK;
        break;
    case PWM_6:
        PMD2 |= _PMD2_OC6MD_MASK;
        break;
    }
}

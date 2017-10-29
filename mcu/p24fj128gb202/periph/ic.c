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
#include "periph/ic.h"
#include "periph_conf.h"

#define ICxCON1(I)              (base_address[I][0x0 / 0x2])
#define ICxCON2(I)              (base_address[I][0x2 / 0x2])
#define ICxBUF(I)               (base_address[I][0x4 / 0x2])

static volatile uint16_t * const base_address[IC_COUNT] = {
    &IC1CON1,
    &IC2CON1,
    &IC3CON1,
    &IC4CON1,
    &IC5CON1,
    &IC6CON1
};
static enum IC_MODE mode[IC_COUNT];

void __attribute__ ((weak)) ic1_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine ic1_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__ ((weak)) ic2_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine ic2_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__ ((weak)) ic3_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine ic3_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__ ((weak)) ic4_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine ic4_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__ ((weak)) ic5_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine ic5_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__ ((weak)) ic6_callback(void)
{
    /*
     * Do not add your own code here !
     *
     * Redefine ic6_callback without the weak attribute
     * and the linker will pick your function instead of
     * this one.
     */
}

void __attribute__ ((interrupt, no_auto_psv)) _IC1Interrupt(void)
{
    ic1_callback();
    IFS0 &= ~_IFS0_IC1IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _IC2Interrupt(void)
{
    ic2_callback();
    IFS0 &= ~_IFS0_IC2IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _IC3Interrupt(void)
{
    ic3_callback();
    IFS2 &= ~_IFS2_IC3IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _IC4Interrupt(void)
{
    ic4_callback();
    IFS2 &= ~_IFS2_IC4IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _IC5Interrupt(void)
{
    ic5_callback();
    IFS2 &= ~_IFS2_IC5IF_MASK;
}

void __attribute__ ((interrupt, no_auto_psv)) _IC6Interrupt(void)
{
    ic6_callback();
    IFS2 &= ~_IFS2_IC6IF_MASK;
}

void ic_configure(unsigned int ic_num, enum IC_MODE _mode, uint8_t enable_interrupt)
{
    mode[ic_num] = _mode;

    /* Use System Clock / 2 as clock source */
    ICxCON1(ic_num) = (0b111 << _IC1CON1_ICTSEL_POSITION) & _IC1CON1_ICTSEL_MASK;
    ICxCON2(ic_num) = _IC1CON2_ICTRIG_MASK;

    if (enable_interrupt) {
        switch (ic_num) {
        case IC_1:
            IEC0 |= _IEC0_IC1IE_MASK;
            break;
        case IC_2:
            IEC0 |= _IEC0_IC2IE_MASK;
            break;
        case IC_3:
            IEC2 |= _IEC2_IC3IE_MASK;
            break;
        case IC_4:
            IEC2 |= _IEC2_IC4IE_MASK;
            break;
        case IC_5:
            IEC2 |= _IEC2_IC5IE_MASK;
            break;
        case IC_6:
            IEC2 |= _IEC2_IC6IE_MASK;
            break;
        }
    } else {
        switch (ic_num) {
        case IC_1:
            IEC0 &= ~_IEC0_IC1IE_MASK;
            break;
        case IC_2:
            IEC0 &= ~_IEC0_IC2IE_MASK;
            break;
        case IC_3:
            IEC2 &= ~_IEC2_IC3IE_MASK;
            break;
        case IC_4:
            IEC2 &= ~_IEC2_IC4IE_MASK;
            break;
        case IC_5:
            IEC2 &= ~_IEC2_IC5IE_MASK;
            break;
        case IC_6:
            IEC2 &= ~_IEC2_IC6IE_MASK;
            break;
        }
    }
}

void ic_enable(unsigned int ic_num)
{
    ICxCON1(ic_num) &= ~_IC1CON1_ICM_MASK;
    ICxCON1(ic_num) |= ((uint16_t)mode[ic_num] << _IC1CON1_ICM_POSITION) & _IC1CON1_ICM_MASK;
    ICxCON2(ic_num) |= _IC1CON2_TRIGSTAT_MASK;
}

void ic_disable(unsigned int ic_num)
{
    ICxCON1(ic_num) &= ~_IC1CON1_ICM_MASK;
}

uint8_t ic_has_data(unsigned int ic_num)
{
    return !!(ICxCON1(ic_num) & _IC1CON1_ICBNE_MASK);
}

uint16_t ic_get_data(unsigned int ic_num)
{
    return ICxBUF(ic_num);
}

void ic_power_up(unsigned int ic_num)
{
    switch (ic_num) {
    case IC_1:
        PMD2 &= ~_PMD2_IC1MD_MASK;
        break;
    case IC_2:
        PMD2 &= ~_PMD2_IC2MD_MASK;
        break;
    case IC_3:
        PMD2 &= ~_PMD2_IC3MD_MASK;
        break;
    case IC_4:
        PMD2 &= ~_PMD2_IC4MD_MASK;
        break;
    case IC_5:
        PMD2 &= ~_PMD2_IC5MD_MASK;
        break;
    case IC_6:
        PMD2 &= ~_PMD2_IC6MD_MASK;
        break;
    }
}

void ic_power_down(unsigned int ic_num)
{
    switch (ic_num) {
    case IC_1:
        PMD2 |= _PMD2_IC1MD_MASK;
        break;
    case IC_2:
        PMD2 |= _PMD2_IC2MD_MASK;
        break;
    case IC_3:
        PMD2 |= _PMD2_IC3MD_MASK;
        break;
    case IC_4:
        PMD2 |= _PMD2_IC4MD_MASK;
        break;
    case IC_5:
        PMD2 |= _PMD2_IC5MD_MASK;
        break;
    case IC_6:
        PMD2 |= _PMD2_IC6MD_MASK;
        break;
    }
}

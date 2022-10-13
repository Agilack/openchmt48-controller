/**
 * @file  valve.c
 * @brief definitions of functions driving valves on pnp
 *
 * @author Axel Paolillo <axel.paolillo@agilack.fr>
 * @copyright Agilack (c) 2022
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */

#include "valve.h"
#include "types.h"
#include "hardware.h"

#define OFF 0
#define ON  1

void valve_right(uint state)
{
    if(state == ON)
    {
        reg_set((u32)GPIOB_BSRR, (1 << 15));         //PB15 HIGH, switch on right valve
    }
    else if(state == OFF)
    {
        reg_set((u32)GPIOB_BSRR, (1 << 31));         //PB15 LOW, switch off right valve
    }
}

void valve_left(uint state)
{
    if(state == ON)
    {
        reg_set((u32)GPIOD_BSRR, (1 << 8));         //PD8 HIGH, switch on left valve
    }
    else if(state == OFF)
    {
        reg_set((u32)GPIOD_BSRR, (1 << 24));         //PD8 LOW, switch off left valve
    }
}

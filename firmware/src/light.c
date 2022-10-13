/**
 * @file  light.c
 * @brief definitions of functions driving lights on pnp
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

#include "light.h"
#include "types.h"
#include "hardware.h"

#define OFF 0
#define ON  1

void down_light(uint state)
{
    if(state == ON)
    {
        reg_set((u32)GPIOD_BSRR, (1 << 9));         //PD9 HIGH, turn on downward light
    }
    else if(state == OFF)
    {
        reg_set((u32)GPIOD_BSRR, (1 << 25));         //PD9 LOW, turn off downward light
    }
}

void up_light(uint state)
{
    if(state == ON)
    {
        reg_set((u32)GPIOD_BSRR, (1 << 10));         //PD10 HIGH, turn on upward light
    }
    else if(state == OFF)
    {
        reg_set((u32)GPIOD_BSRR, (1 << 26));         //PD10 LOW, turn off upward light
    }
}

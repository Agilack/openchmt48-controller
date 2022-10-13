/**
 * @file  needle.c
 * @brief definitions of functions driving the solenoid on pnp
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

#include "needle.h"
#include "types.h"
#include "hardware.h"

#define OFF 0
#define ON  1

void needle(uint state)
{
    if(state == ON)
    {
        reg16_set((u32)TIM12_CCER, (1 << 0));        //Turn on solenoid
    }
    else if(state == OFF)
    {
        reg16_clr((u32)TIM12_CCER, (1 << 0));        //Turn off solenoid    NONONO does it need to be down or up when turned off ?
    }
}

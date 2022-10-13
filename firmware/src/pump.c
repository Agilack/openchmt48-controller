/**
 * @file  pump.c
 * @brief definitions of functions driving the pumps on pnp
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

#include "pump.h"
#include "hardware.h"
#include "types.h"

#define OFF 0
#define ON  1

void pump_vacuum(uint state)
{
    if(state == ON)
    {
        reg16_set((u32)TIM4_CCER, (1 << 4));        //Enable VACUUM PWM
    }
    else if(state == OFF)
    {
        reg16_clr((u32)TIM4_CCER, (1 << 4));        //Disable VACUUM PWM
    }
}

void pump_blow(uint state)
{
    if(state == ON)
    {
        reg16_set((u32)TIM4_CCER, (1 << 0));        //Enable BLOW PWM
    }
    else if(state == OFF)
    {
        reg16_clr((u32)TIM4_CCER, (1 << 0));        //Disable BLOW PWM
    }
}

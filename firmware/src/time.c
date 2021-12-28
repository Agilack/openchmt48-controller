/**
 * @file  time.c
 * @brief Functions to handle time
 *
 * @author Saint-Genest Gwenael <gwen@agilack.fr>
 * @copyright Agilack (c) 2018-2020
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */

#include "time.h"
#include "types.h"
#include "rtos/include/FreeRTOS.h"
#include "rtos/include/queue.h"
#include "rtos/include/task.h"

/**
 * @brief Return the current time (in ms) based on squeduler ticks
 *
 * @return u32 Time (in 1ms) since module started
 */
u32 time_now(void)
{
    u32 ticks = xTaskGetTickCount();
    
    ticks = ticks * (1000 / configTICK_RATE_HZ);
    
    return ticks;
}

/**
 * @brief Compute the time elapsed from a reference
 *
 * @param  ref Reference time (in ms)
 * @return uint32 Number of ms since "ref"
 */
u32 time_since(u32 ref)
{
    u32 tm_diff;

    tm_diff = (time_now() - ref);

    return(tm_diff);
}
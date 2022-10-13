/**
 * @file  pump.h
 * @brief declarations of functions driving the pumps on pnp
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

#ifndef PUMP_H
#define PUMP_H

#include "types.h"

void pump_vacuum(uint state);
void pump_blow(uint state);

#endif

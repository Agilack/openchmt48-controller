/**
 * @file  pnp.h
 * @brief Headers for pnp task
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
#ifndef PNP_H
#define PNP_H

#define IDLE 0
#define HEAD_MOVE 1
#define NOZZLE_MOVE 2
#define AIR_PUMP 3
#define HOOK 4

void fsm_idle(void);
void fsm_head_move(void);

#endif

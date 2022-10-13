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

#define IDLE                0
#define PROCESS_COMP        1
#define HEAD_TO_TRAY_RIB    2
#define NEEDLE_DOWN         3
#define HEAD_PULL_RIB       4
#define NEEDLE_UP           5
#define PULL_RIB_COVER      6
#define HEAD_TO_TRAY        7
#define NOZZLE_TAKE_DOWN    8
#define PUMP_VACUUM         9
#define NOZZLE_TAKE_UP      10
#define HEAD_TO_DELIVER     11
#define NOZZLE_ROTATE       12
#define NOZZLE_DELIVER_DOWN 13
#define PUMP_BLOW           14
#define NOZZLE_DELIVER_UP   15
#define HEAD_TO_PARK        16

#define STOP                100

void pnp_init_zero(void);

#endif

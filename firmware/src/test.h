/**
 * @file  test.h
 * @brief test functions definitions
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
#ifndef TEST_H
#define TEST_H

void adc_test(void);
void pump_vacuum_blow_test(void);
void nozzle_test(void);
void valve_light_test(void);
void test_hw_encoder(void);
void encoder_init(void);

#endif

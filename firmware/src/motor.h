/**
 * @file  motor.h
 * @brief declarations of functions driving motors on pnp
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

#ifndef MOTOR_H
#define MOTOR_H

void encoder_Y_A_irq(void);
void encoder_Y_B_irq(void);
void encoder_X_A_irq(void);
void encoder_X_B_irq(void);
void motor_set_dir(int motor, int dir);
void motor_pwm(int motor, int toggle);
int limit_switch(int motor);
void set_x_pwm_period(int period);
void set_y_pwm_period(int period);
void move_x(int pos_encoder_target);
void move_y(int pos_encoder_target);
void move_x_y(int pos_x_target, int pos_y_target);

#endif

/**
 * @file  motor.c
 * @brief Functions to handle motors
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

#include "hardware.h"
#include <stdbool.h>

#define MOTOR_X 1
#define MOTOR_Y 2

#define FORWARD 1
#define BACKWARD 2

#define ENABLE 1
#define DISABLE 2

void motor_set_dir(int motor, int dir);
void motor_pwm(int motor, int toggle);
int limit_switch(int motor);


void motor_set_dir(int motor, int dir)
{
    switch(motor)
    {
        case MOTOR_X:
            if(dir == FORWARD)
                reg_set((u32)GPIOC_BSRR, (1 << 0));         //PC0 HIGH, set X direction to right
            else if(dir == BACKWARD)
                reg_set((u32)GPIOC_BSRR, (1 << 16));         //PC0 LOW, set X direction to left
            break;
        case MOTOR_Y:
            if(dir == FORWARD)
                reg_set((u32)GPIOC_BSRR, (1 << 1));         //PC1 HIGH, set Y direction to UP
            else if(dir == BACKWARD)
                reg_set((u32)GPIOC_BSRR, (1 << 17));         //PC1 LOW, set Y direction to DOWN
            break;
    }
}

void motor_pwm(int motor, int toggle)
{
    switch(motor)
    {
        case MOTOR_X:
            if(toggle == ENABLE)
                reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM
            else if(toggle == DISABLE)
                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
            break;

        case MOTOR_Y:
            if(toggle == ENABLE)
                reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM
            else if(toggle == DISABLE)
                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM
            break;
    }
}

/**
 * @brief Return 1 if limit switch on this axis is closed
 *
 * @param motor Motor axis on which the limit switch is tested
 */
int limit_switch(int motor)
{
    switch(motor)
    {
        case MOTOR_X:
            if((reg_rd(GPIO_IDR(GPIOC)) & (1 << 10)) == 0)
                return 1;        //X limit switch is closed
            else
                return 0;
            break;

        case MOTOR_Y:
            if((reg_rd(GPIO_IDR(GPIOC)) & (1 << 11)) == 0)
                return 1;       //Y limit switch is closed
            else
                return 0;
            break;
    }
    return -1;
}
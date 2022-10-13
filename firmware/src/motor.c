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

#include "motor.h"
#include "hardware.h"
#include <stdbool.h>
#include "uart.h"

#define MOTOR_X 1
#define MOTOR_Y 2

#define FORWARD 1
#define BACKWARD 2

#define ENABLE 1
#define DISABLE 2

extern volatile int pos_x, pos_y;

void encoder_Y_A_irq(void)
{
    if (reg_rd(GPIO_IDR(GPIOB)) & (1 << 12))                //Rising edge on Y_A
    {
        //uart_puts("R on Y_A\r\n");
        if(reg_rd(GPIO_IDR(GPIOB)) & (1 << 13))             //Y_B HIGH
            pos_y--;
        else if(!(reg_rd(GPIO_IDR(GPIOB)) & (1 << 13)))     //Y_B LOW
            pos_y++;
    }
    else if(!(reg_rd(GPIO_IDR(GPIOB)) & (1 << 12)))         //Falling edge on Y_A
    {
        //uart_puts("F on Y_A\r\n");
        if(reg_rd(GPIO_IDR(GPIOB)) & (1 << 13))             //Y_B HIGH
            pos_y++;
        else if(!(reg_rd(GPIO_IDR(GPIOB)) & (1 << 13)))     //Y_B LOW
            pos_y--;
    }
}

void encoder_Y_B_irq(void)
{
    if (reg_rd(GPIO_IDR(GPIOB)) & (1 << 13))                //Rising edge on Y_B
    {
        //uart_puts("R on Y_B\r\n");
        if(reg_rd(GPIO_IDR(GPIOB)) & (1 << 12))             //Y_A HIGH
            pos_y++;
        else if(!(reg_rd(GPIO_IDR(GPIOB)) & (1 << 12)))     //Y_A LOW
            pos_y--;
    }
    else if(!(reg_rd(GPIO_IDR(GPIOB)) & (1 << 13)))         //Falling edge on Y_B
    {
        //uart_puts("F on Y_B\r\n");
        if(reg_rd(GPIO_IDR(GPIOB)) & (1 << 12))             //Y_A HIGH
            pos_y--;
        else if(!(reg_rd(GPIO_IDR(GPIOB)) & (1 << 12)))     //Y_A LOW
            pos_y++;   
    }
}

void encoder_X_A_irq(void)
{
    if (reg_rd(GPIO_IDR(GPIOE)) & (1 << 14))                //Rising edge on X_A
    {
        //uart_puts("R on X_A\r\n");
        if(reg_rd(GPIO_IDR(GPIOE)) & (1 << 15))             //X_B HIGH
            pos_x++;
        else if(!(reg_rd(GPIO_IDR(GPIOE)) & (1 << 15)))     //X_B LOW
            pos_x--;
    }
    else if(!(reg_rd(GPIO_IDR(GPIOE)) & (1 << 14)))         //Falling edge on X_A
    {
        //uart_puts("F on X_A\r\n");
        if(reg_rd(GPIO_IDR(GPIOE)) & (1 << 15))             //X_B HIGH
            pos_x--;
        else if(!(reg_rd(GPIO_IDR(GPIOE)) & (1 << 15)))     //X_B LOW
            pos_x++;
    }
}

void encoder_X_B_irq(void)
{
    if (reg_rd(GPIO_IDR(GPIOE)) & (1 << 15))                //Rising edge on X_B
    {
        //uart_puts("R on X_B\r\n");
        if(reg_rd(GPIO_IDR(GPIOE)) & (1 << 14))             //X_A HIGH
            pos_x--;
        else if(!(reg_rd(GPIO_IDR(GPIOE)) & (1 << 14)))     //X_A LOW
            pos_x++;
    }
    else if(!(reg_rd(GPIO_IDR(GPIOE)) & (1 << 15)))         //Falling edge on X_B
    {
        //uart_puts("F on X_B\r\n");
        if(reg_rd(GPIO_IDR(GPIOE)) & (1 << 14))             //X_A HIGH
            pos_x++;
        else if(!(reg_rd(GPIO_IDR(GPIOE)) & (1 << 14)))     //X_A LOW
            pos_x--;
    }
}

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

void set_x_pwm_period(int period)  //Period in us, only x
{
    reg_wr((u32)TIM2_ARR, period*16);     //Preload

    reg_wr((u32)TIM2_CCR1, period*8);    //Duty cycle 50 percent
}

void set_y_pwm_period(int period)  //Period in us, only Y
{
    reg_wr((u32)TIM5_ARR, period*16);     //Preload

    reg_wr((u32)TIM5_CCR2, period*8);    //Duty cycle 50 percent
}

//Go to X position pos_encoder_target based on encoder, verification via encoder
void move_x(int pos_encoder_target)
{
    if(pos_encoder_target > 31745 || pos_encoder_target < 0)
    {
        uart_puts("Error, target position not whithin range.\r\n");
        return;
    }
    int delta = pos_encoder_target - pos_x;

    if (delta > 0)
    {
        reg_set((u32)GPIOC_BSRR, (1 << 0));         //PC0 HIGH, set X direction to right
        if ((delta + pos_x) > 31745)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }

        reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM

        while (pos_x < pos_encoder_target)         //Wait until reaching target position
        {
        }

        reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM

    }
    else
    {
        reg_set((u32)GPIOC_BSRR, (1 << 16));         //PC0 LOW, set X direction to left
        delta = -delta;

        if (delta > pos_x)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }

        reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM

        while (pos_x > pos_encoder_target);          //Wait until reaching target position

        reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
    }
}

//Go to Y position pos_encoder_target based on encoder, verification via encoder
void move_y(int pos_encoder_target)
{
    if(pos_encoder_target > 31745 || pos_encoder_target < 0)
    {
        uart_puts("Error, target position not whithin range.\r\n");
        return;
    }
    int delta = pos_encoder_target - pos_y;

    if (delta > 0)
    {
        reg_set((u32)GPIOC_BSRR, (1 << 1));         //PC1 HIGH, set Y direction to UP
        if ((delta + pos_y) > 31745)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }

        reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM

        while (pos_y < pos_encoder_target)         //Wait until reaching target position
        {
        }

        reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM

    }
    else
    {
        reg_set((u32)GPIOC_BSRR, (1 << 17));         //PC1 LOW, set Y direction to DOWN
        delta = -delta;

        if (delta > pos_y)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }

        reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM

        while (pos_y > pos_encoder_target);          //Wait until reaching target position

        reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM
    }
}

void move_x_y(int pos_x_target, int pos_y_target)
{
    if(pos_x_target > 31745 || pos_x_target < 0 || pos_y_target > 31745 || pos_y_target < 0)
    {
        uart_puts("Error, target position not whithin range.\r\n");
        return;
    }
    int delta_x = pos_x_target - pos_x;
    int delta_y = pos_y_target - pos_y;

    if(delta_x > 0)
    {
        reg_set((u32)GPIOC_BSRR, (1 << 0));         //PC0 HIGH, set X direction to right
        if ((delta_x + pos_x) > 31745)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }
    }
    else
    {
        reg_set((u32)GPIOC_BSRR, (1 << 16));         //PC0 LOW, set X direction to left
        uart_puts("\r\nX NO BUG\r\n");
        delta_x = -delta_x;

        if (delta_x > pos_x)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }
        delta_x = -delta_x;
    }

    if(delta_y > 0)
    {
        reg_set((u32)GPIOC_BSRR, (1 << 1));         //PC1 HIGH, set Y direction to UP
        if ((delta_y + pos_y) > 31745)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }
    }
    else
    {
        reg_set((u32)GPIOC_BSRR, (1 << 17));         //PC1 LOW, set Y direction to DOWN
        delta_y = -delta_y;

        if (delta_y > pos_y)
        {
            uart_puts("Error, target position exceeds limits.\r\n");
            return;
        }
        delta_y = -delta_y;
    }

    if(delta_x > 0)
    {
        if(delta_y > 0)
        {
            //CAS 1 X FW Y FW
            uart_puts("X forward, Y forward.\r\n");
            reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM
            reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM

            while((pos_x < pos_x_target) && (pos_y < pos_y_target));    //Wait until one of them reaches destination

            if (pos_x >= pos_x_target)           //X is done first
            {
                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM

                while(pos_y < pos_y_target);    //Wait for Y

                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM
                return;

            }
            else if(pos_y >= pos_y_target)       //Y is done first
            {
                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM

                while(pos_x < pos_x_target);    //Wait for X

                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
                return;
            }
        }
        else
        {
            //CAS 2 X FW Y BW
            uart_puts("X forward, Y backward.\r\n");
            reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM
            reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM

            while((pos_x < pos_x_target) && (pos_y > pos_y_target));    //Wait until one of them reaches destination

            if (pos_x >= pos_x_target)           //X is done first
            {
                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM

                while(pos_y > pos_y_target);    //Wait for Y

                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM
                return;

            }
            else if(pos_y <= pos_y_target)       //Y is done first
            {
                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM

                while(pos_x < pos_x_target);    //Wait for X

                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
                return;
            }
        }
    }
    else
    {
        if (delta_y > 0)
        {
            //CAS 3 X BW Y FW
            uart_puts("X backward, Y forward.\r\n");
            reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM
            reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM

            while((pos_x > pos_x_target) && (pos_y < pos_y_target));    //Wait until one of them reaches destination

            if (pos_x <= pos_x_target)           //X is done first
            {
                //uart_puts("X is done first.\r\n");
                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM

                while(pos_y < pos_y_target);    //Wait for Y

                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM
                return;

            }
            else if(pos_y >= pos_y_target)       //Y is done first
            {
                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM

                while(pos_x > pos_x_target);    //Wait for X

                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
                return;
            }
        }
        else
        {
            //CAS 4 X BW Y BW
            uart_puts("X backward, Y backward.\r\n");
            reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM
            reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM

            while((pos_x > pos_x_target) && (pos_y > pos_y_target));    //Wait until one of them reaches destination

            if (pos_x <= pos_x_target)           //X is done first
            {
                //uart_puts("X is done first.\r\n");
                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM

                while(pos_y > pos_y_target);    //Wait for Y

                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM
                return;

            }
            else if(pos_y <= pos_y_target)       //Y is done first
            {
                reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM

                while(pos_x > pos_x_target);    //Wait for X

                reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
                return;
            }
        }
    }
}

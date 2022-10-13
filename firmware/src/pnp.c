/**
 * @file  pnp.c
 * @brief state machine of PnP
 *
 * @author Axel Paolillo <axel.paolillo@agilack.fr>
 * @copyright Agilack (c) 2021-2022
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */

#include "types.h"
#include "pnp.h"
#include "rtos/include/FreeRTOS.h"
#include "rtos/include/task.h"
#include "queue.h"
#include "hardware.h"
#include "uart.h"
#include "test.h"
#include "motor.h"
#include "needle.h"
#include "pump.h"

static uint fsm_state;    /* Current state of the pnp FSM */
static int cmd_id;
extern xQueueHandle cmd_queue;

static void fsm_transition(uint state);
static void fsm_idle(void);
static void fsm_head_move(void);
void delay_rtos(int ms);

void task_pnp(void *pvParameters)
{
    uart_puts(" * Init task PNP\r\n");

    delay_rtos(3000);

    pnp_init_zero();

    fsm_state = IDLE;

    uart_puts("[TASK_PNP] while 1\r\n");
    while(1)
    {
        switch(fsm_state)
        {
            case IDLE:
                fsm_idle();
                break;

            case PROCESS_COMP:
                break;

            case HEAD_TO_TRAY_RIB:
                break;

            case NEEDLE_DOWN:
                break;

            case HEAD_PULL_RIB:
                break;

            case NEEDLE_UP:
                break;

            case PULL_RIB_COVER:
                break;

            case HEAD_TO_TRAY:
                break;

            case NOZZLE_TAKE_DOWN:
                break;

            case PUMP_VACUUM:
                break;

            case NOZZLE_TAKE_UP:
                break;

            case HEAD_TO_DELIVER:
                fsm_head_move();
                break;

            case NOZZLE_ROTATE:
                break;

            case NOZZLE_DELIVER_DOWN:
                break;

            case PUMP_BLOW:
                break;

            case NOZZLE_DELIVER_UP:
                break;

            case HEAD_TO_PARK:
                break;

            case STOP:
                break;

        }

        //test_hw_encoder();
    }
}

void pnp_init_zero(void)
{
    reg_set((u32)GPIOC_BSRR, (1 << 16));         //PC0 LOW, set X direction to left
    reg_set((u32)GPIOC_BSRR, (1 << 17));         //PC1 LOW, set Y direction to DOWN

    set_x_pwm_period(3200);
    set_y_pwm_period(3200);

    reg16_set((u32)TIM2_CCER, (1 << 0));        //Enable X PWM
    reg16_set((u32)TIM5_CCER, (1 << 4));        //Enable Y PWM
    
    while(1)
    {
    while((reg_rd(GPIO_IDR(GPIOC)) & (1 << 10)) != 0 && (reg_rd(GPIO_IDR(GPIOC)) & (1 << 11)) != 0);    //Wait until one of them reaches end

    if((reg_rd(GPIO_IDR(GPIOC)) & (1 << 10)) == 0)  //X reached end first
    {
        reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM

        while((reg_rd(GPIO_IDR(GPIOC)) & (1 << 11)) != 0);  //Wait for Y to reach end

        reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM


    }
    else if((reg_rd(GPIO_IDR(GPIOC)) & (1 << 11)) == 0) //Y reached end first
    {
        reg16_clr((u32)TIM5_CCER, (1 << 4));        //Disable Y PWM

        while((reg_rd(GPIO_IDR(GPIOC)) & (1 << 10)) != 0);  //Wait for X to reach end

        reg16_clr((u32)TIM2_CCER, (1 << 0));        //Disable X PWM
    }
    else                                            //In case of switch bounce loop again
        continue;
    break;
    }
}

static void fsm_idle(void)
{
    cmd_id=0;
    int val=0;

    if(uxQueueMessagesWaiting(cmd_queue) > 0)
    {
        if(xQueueReceive(cmd_queue, &cmd_id, portMAX_DELAY) == pdPASS)
        {
            uart_puts("Element found\r\n");
            if (cmd_id == 1 || cmd_id == 2 || cmd_id == 3)
            {
                fsm_transition(HEAD_TO_DELIVER);
            }
            else if(cmd_id == 4)
            {
                if(xQueueReceive(cmd_queue, &val, portMAX_DELAY) == pdPASS)
                {
                    uart_putdec(val);
                }
            }
        }
    }
    //uart_puts("Element not found\r\n");
}

static void fsm_head_move(void)
{
    int x_target=0;
    int y_target=0;
    int flag_x_target_received = 0;
    int flag_y_target_received = 0;

    switch(cmd_id)
    {
        case 1:
            if(xQueueReceive(cmd_queue, &x_target, portMAX_DELAY) == pdPASS)
            {
                uart_puts("Command MOVE X to position ");
                uart_putdec(x_target);
                uart_puts(" received.\r\n\n");

                set_x_pwm_period(2500);

                move_x(x_target);

                uart_puts("Target position reached.\r\n\n");

                fsm_transition(IDLE);
            }
            break;
        case 2:
            if(xQueueReceive(cmd_queue, &y_target, portMAX_DELAY) == pdPASS)
            {
                uart_puts("Command MOVE Y to position ");
                uart_putdec(y_target);
                uart_puts(" received.\r\n\n");

                set_y_pwm_period(2500);

                move_y(y_target);

                uart_puts("Target position reached.\r\n\n");

                fsm_transition(IDLE);
            }
            break;
        case 3:
            uart_puts("Command MOVE X and Y received.\r\n");
            if (xQueueReceive(cmd_queue, &x_target, portMAX_DELAY) == pdPASS)
            {
                uart_puts("MOVE X to position ");
                uart_putdec(x_target);
                uart_puts(".\r\n\n");
                flag_x_target_received = 1;
            }
            if (xQueueReceive(cmd_queue, &y_target, portMAX_DELAY) == pdPASS)
            {
                uart_puts("MOVE Y to position ");
                uart_putdec(y_target);
                uart_puts(".\r\n\n");
                flag_y_target_received = 1;
            }

            if(flag_x_target_received && flag_y_target_received)
            {
                flag_x_target_received = 0;
                flag_y_target_received = 0;
                set_x_pwm_period(2500);
                set_y_pwm_period(2500);
                move_x_y(x_target, y_target);
                uart_puts("Target position reached.\r\n\n");
                fsm_transition(IDLE);
            }
            break;
    }
}

/**
 * @brief Update the FSM state (do transition)
 *
 * @param state New state for the FSM
 */
static void fsm_transition(uint state)
{
    fsm_state    = state;
    return;
}
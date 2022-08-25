/**
 * @file  main.c
 * @brief Main function of the firmware
 *
 * @authors Axel Paolillo <axel.paolillo@agilack.fr>
 *          Saint-Genest Gwenael <gwen@agilack.fr>
 * @copyright Agilack (c) 2021
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */

/**
 * @brief Entry point of the firmware
 *
 * @return integer Exit code (but should never returns)
 */

#include "hardware.h"
#include "types.h"
#include "rtos/include/FreeRTOS.h"
#include "rtos/include/task.h"
#include "time.h"
#include "pnp.h"
#include "queue.h"
#include "uart.h"

static uint fsm_state;    /* Current state of the pnp FSM */
static int cmd_id;
volatile int pos_x, pos_y;

void delay_us(int us);
void delay_rtos(int ms);
static void task_pnp(void *pvParameters);
void task_console(void *pvParameters);
static void pwm_initilize(void);
static void init_io(void);
static void set_x_pwm_period(int period);
static void set_y_pwm_period(int period);
void Setup_X_A_interupt(void);
void Setup_X_B_interupt(void);
void Setup_Y_A_interupt(void);
void Setup_Y_B_interupt(void);
//void encoder_init(void);
static void fsm_transition(uint state);
void move_x(int pos_encoder_target);
void move_y(int pos_encoder_target);
void move_x_y(int pos_x_target, int pos_y_target);
void pnp_init_zero(void);
void test_hw_encoder(void);

xQueueHandle cmd_queue;

int main(void)
{
    cmd_queue = xQueueCreate(10, sizeof(int));

    xTaskCreate(task_console, "Console", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY+1), NULL);
    xTaskCreate(task_pnp, "PickNPlace", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY+1), NULL);

    
    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    uart_puts(" * FAILED TO START SQUEDULER\r\n");

    while(1);
}

static void task_pnp(void *pvParameters)
{
    uart_puts(" * Init task PNP\r\n");
    init_io();
    pwm_initilize();
    //encoder_init();
    Setup_X_A_interupt();
    Setup_X_B_interupt();
    Setup_Y_A_interupt();
    Setup_Y_B_interupt();

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
            case HEAD_MOVE:
                fsm_head_move();
                break;
            case NOZZLE_MOVE:
                break;
            case AIR_PUMP:
                break;
            case HOOK:
                break;
        }

        //test_hw_encoder();
    }
}

void test_hw_encoder(void)
{
    delay_rtos(200);
    if ((reg_rd(GPIO_IDR(GPIOA)) & (1 << 11)) == 0)
    {
        reg_clr((u32)TIM_CNT(TIM1), 0xFFFF);
        uart_puts("Encoder value reset !\r\n");
    }
    uart_puts("Encoder value : ");
    uart_putdec(reg_rd((u32)TIM_CNT(TIM1)));
    uart_puts("\r\n");
}


static void pwm_initilize(void)
{
    u32 val;
    u32 a_mode;

     /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 0); /* GPIO-A */
    reg_wr((u32)RCC_AHB1ENR, val);

    a_mode = reg_rd((u32)GPIOA_MODER);
    a_mode |= (2 << 0);                 //PA0 alternate function mode
    reg_wr((u32)GPIOA_MODER, a_mode); 

    val = reg_rd((u32)GPIOA_AFRL);
    val |= (1 << 0);                    //AF1 (TIM2_CH1) on PA0
    reg_wr((u32)GPIOA_AFRL, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 0);                    //TIM2 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM2_CR1, (1 << 7));     //Auto-reload preload enable
    //reg16_set((u32)TIM2_PSC, 0);          //Prescaler = 0
    reg_wr((u32)TIM2_ARR, 3200);            //Preload = 3200 (Period = 400us)
    reg_wr((u32)TIM2_CCR1, 1600);           //Duty cycle 50 percent
    //reg_set((u32)TIM2_CCMR1, (0 << 0));   //CC1 channel configured as output
    //reg_set((u32)TIM2_CCER, (0 << 1));    //Set polarity to active high
    reg_set((u32)TIM2_CCMR1, (6 << 4));     //Output Compare 1 mode set to PWM mode 1
    //reg_set((u32)TIM2_CCMR1, (0 << 16)); 
    reg_set((u32)TIM2_CCMR1, (1 << 3));     //Preload enable
    //reg16_set((u32)TIM2_CCER, (1 << 0));  //Enable CC1
    reg16_set((u32)TIM2_EGR, (1 << 0));     //Update generation
    ////reg_set((u32)TIM2_BDTR, (1 << 15)); //Main Output Enable

    a_mode = reg_rd((u32)GPIOA_MODER);
    a_mode |= (2 << 2);                 //PA1 alternate function mode
    reg_wr((u32)GPIOA_MODER, a_mode); 

    val = reg_rd((u32)GPIOA_AFRL);
    val |= (2 << 4);                    //AF2 (TIM5_CH2) on PA1
    reg_wr((u32)GPIOA_AFRL, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 3);                    //TIM5 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM5_CR1, (1 << 7));     //Auto-reload preload enable
    reg_wr((u32)TIM5_ARR, 3200);            //Preload = 3200 (Period = 400us)
    reg_wr((u32)TIM5_CCR2, 1600);           //Duty cycle 50 percent
    reg_set((u32)TIM5_CCMR1, (6 << 12));    //Output Compare 2 mode set to PWM mode 1
    reg_set((u32)TIM5_CCMR1, (1 << 11));    //Preload enable
    //reg16_set((u32)TIM5_CCER, (1 << 4));  //Enable CC2

    reg16_set((u32)TIM5_EGR, (1 << 0));     //Update generation

    reg16_set((u32)TIM2_CR1, (1 << 0));     //Start timer 2
    reg16_set((u32)TIM5_CR1, (1 << 0));     //Start timer 5
}

static void init_io(void)
{
    u32 val;

    /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 0); /* GPIO-A */
    val |= (1 << 1); /* GPIO-B */
    val |= (1 << 2); /* GPIO-C */
    val |= (1 << 4); /* GPIO-E */
    val |= (1 << 5); /* GPIO-F */
    reg_wr((u32)RCC_AHB1ENR, val);

//XYZ_EN PA4
    reg_clr((u32)GPIOA_MODER, 0x300);       //Clear bits 8 and 9
    reg_set((u32)GPIOA_MODER, (1 << 8));    //Output mode

    reg_set((u32)GPIOA_BSRR, (1 << 4));     //PA4 HIGH = ENABLE

//BCD_EN PA5
    reg_clr((u32)GPIOA_MODER, 0xC00);       //Clear bits 10 and 11
    reg_set((u32)GPIOA_MODER, (1 << 10));    //Output mode

    reg_set((u32)GPIOA_BSRR, (1 << 5));     //PA5 HIGH = ENABLE

//X_DIR PC0
    reg_clr((u32)GPIOC_MODER, 0x3);         //Clear bits 0 and 1
    reg_set((u32)GPIOC_MODER, (1 << 0));    //Output mode

    reg_set((u32)GPIOC_BSRR, (1 << 0));     //PC0 HIGH

//Y_DIR PC1
    reg_clr((u32)GPIOC_MODER, 0xC);         //Clear bits 2 and 3
    reg_set((u32)GPIOC_MODER, (1 << 2));    //Output mode

    reg_set((u32)GPIOC_BSRR, (1 << 1));     //PC1 HIGH

//Nozzle_CLK PA6
    reg_clr((u32)GPIOA_MODER, 0x3000);       //Clear bits 12 and 13
    reg_set((u32)GPIOA_MODER, (1 << 12));     //Output mode

    //reg_set((u32)GPIOA_BSRR, (1 << 6));     //PA6 HIGH = ENABLE

//Rot-Noz1_CLK PA7
    reg_clr((u32)GPIOA_MODER, 0xC000);       //Clear bits 14 and 15
    reg_set((u32)GPIOA_MODER, (1 << 14));     //Output mode

    //reg_set((u32)GPIOA_BSRR, (1 << 7));     //PA7 HIGH = ENABLE

//Rot-Noz2_CLK PB0
    reg_clr((u32)GPIOB_MODER, 0x3);           //Clear bits 0 and 1
    reg_set((u32)GPIOB_MODER, (1 << 0));      //Output mode

    //reg_set((u32)GPIOB_BSRR, (1 << 0));     //PB0 HIGH = ENABLE

//Nozzle_DIR PF13
    reg_clr((u32)GPIOF_MODER, 0xC000000);     //Clear bits 26 and 27
    reg_set((u32)GPIOF_MODER, (1 << 26));     //Output mode

    reg_set((u32)GPIOF_BSRR, (1 << 13));     //PF13 HIGH = ENABLE

//Rot-Noz2_DIR PF14
    reg_clr((u32)GPIOF_MODER, 0x30000000);     //Clear bits 28 and 29
    reg_set((u32)GPIOF_MODER, (1 << 28));     //Output mode

    reg_set((u32)GPIOF_BSRR, (1 << 14));     //PF14 HIGH = ENABLE

//Rot-Noz1_DIR PF15
    reg_clr((u32)GPIOF_MODER, 0xC0000000);     //Clear bits 30 and 31
    reg_set((u32)GPIOF_MODER, (1 << 30));     //Output mode

    reg_set((u32)GPIOF_BSRR, (1 << 15));     //PF15 HIGH = ENABLE

//X_A PE14
    reg_clr((u32)GPIOE_MODER, 0x30000000);     //Clear bits 28 and 29
    reg_set((u32)GPIOE_MODER, (0 << 28));      //Input mode

//X_B PE15
    reg_clr((u32)GPIOE_MODER, 0xC0000000);     //Clear bits 30 and 31
    reg_set((u32)GPIOE_MODER, (0 << 30));      //Input mode

//Y_A PB12
    reg_clr((u32)GPIOB_MODER, 0x3000000);      //Clear bits 24 and 25
    reg_set((u32)GPIOB_MODER, (0 << 24));      //Input mode

//Y_B PB13
    reg_clr((u32)GPIOB_MODER, 0xC000000);     //Clear bits 26 and 27
    reg_set((u32)GPIOB_MODER, (0 << 26));      //Input mode

//X_lim PC10
    reg_clr((u32)GPIOC_MODER, 0x300000);     //Clear bits 20 and 21
    reg_set((u32)GPIOC_MODER, (0 << 20));      //Input mode

//Y_lim PC11
    reg_clr((u32)GPIOC_MODER, 0xC00000);     //Clear bits 22 and 23
    reg_set((u32)GPIOC_MODER, (0 << 22));      //Input mode

    val = reg_rd((u32)GPIO_PUPD(GPIOC));
    val &= ~(0xF00000);                        //Clear bits 20 to 23
    val |= 0x500000;                            //Pull up on PC10 and PC11
    reg_wr((u32)GPIO_PUPD(GPIOC), val);
}

//Setup interruption from the X_A signal (PE14)
void Setup_X_A_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 14 to use PE14 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0xF0FF; /* Clear EXTI14 */
    val |= (4 << 8); /* EXTI14 is now PE14 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 14)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 14)); /* Falling trigger */
    /* Activate EXTI8 (IMR) */
    reg_set(EXTI + 0x00, 1 << 14);
}

//Setup interruption for the X_B signal (PE15)
void Setup_X_B_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 15 to use PE15 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0x0FFF; /* Clear EXTI15 */
    val |= (4 << 12); /* EXTI15 is now PE15 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 15)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 15)); /* Falling trigger */

    /* Activate EXTI9 (IMR) */
    reg_set(EXTI + 0x00, 1 << 15);
}

//Setup interruption from the Y_A signal (PB12)
void Setup_Y_A_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 12 to use PB12 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0xFFF0; /* Clear EXTI12 */
    val |= (1 << 0); /* EXTI12 is now PB12 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 12)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 12)); /* Falling trigger */
    /* Activate EXTI12 (IMR) */
    reg_set(EXTI + 0x00, 1 << 12);
}

//Setup interruption for the Y_B signal (PB13)
void Setup_Y_B_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 13 to use PB13 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0xFF0F; /* Clear EXTI13 */
    val |= (1 << 4); /* EXTI13 is now PB13 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 13)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 13)); /* Falling trigger */

    /* Activate EXTI13 (IMR) */
    reg_set(EXTI + 0x00, 1 << 13);
}

//Interruptions from pins 15 to 10
void EXTI15_10_IRQHandler(void)
{
    u32 val;
    val = reg_rd(EXTI + 0x14);     //Checks which interruption is called
    reg_wr(EXTI + 0x14, val & 0xFC00); // Clear pending register from 15 to 10

    if((reg_rd(GPIO_IDR(GPIOC)) & (1 << 10)) == 0)          //X limit switch reached
        pos_x=0;

    if((reg_rd(GPIO_IDR(GPIOC)) & (1 << 11)) == 0)          //Y limit switch reached
        pos_y=0;

    if (val & (1 << 12))        //EXTI12    Y_A
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

    if (val & (1 << 13))        //EXTI13    Y_B
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

    if (val & (1 << 14))        //EXTI14    X_A
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

    if (val & (1 << 15))        //EXTI15    X_B
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
}

// void encoder_init(void)
// {
//     u32 val;

//     reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG
//     reg_set((u32)RCC_APB2ENR, (1 << 0));     //TIM1 enable

// //Add_IO8 PA8
//     reg_clr((u32)GPIOA_MODER, 0x30000);       //Clear bits 16 and 17
//     reg_set((u32)GPIOA_MODER, (2 << 16));    //AF mode

//     val = reg_rd((u32)GPIOA_AFRH);
//     val |= (1 << 0);                    //AF1 (TIM1_CH1) on PA8
//     reg_wr((u32)GPIOA_AFRH, val);

// //Add_IO9 PA9
//     reg_clr((u32)GPIOA_MODER, 0xC0000);       //Clear bits 18 and 19
//     reg_set((u32)GPIOA_MODER, (2 << 18));    //AF mode

//     val = reg_rd((u32)GPIOA_AFRH);
//     val |= (1 << 4);                    //AF1 (TIM1_CH2) on PA9
//     reg_wr((u32)GPIOA_AFRH, val);

//     val = reg_rd((u32)GPIO_PUPD(GPIOA));
//     val &= ~(0xF0000);                        //Clear bits 16 to 19
//     val |= 0x50000;                            //Pull down on PA8 and PA9
//     reg_wr((u32)GPIO_PUPD(GPIOA), val);

// //Ass_IO11 PA11    Encoder pusher button
//     reg_clr((u32)GPIOA_MODER, 0xC00000);       //Clear bits 22 and 23
//     reg_set((u32)GPIOA_MODER, (0 << 22));    //Input mode


//     reg_set((u32)TIM1_CCMR1, (1 << 0));      //Channel 1 as TI1
//     reg_set((u32)TIM1_CCMR1, (1 << 8));      //Channel 2 as TI2

//     reg_set((u32)TIM1_SMCR, (3 << 0));        //Encoder mode 3
//     reg_set((u32)TIM1_CR1, (1 << 0));        //Enable
// }

static void set_x_pwm_period(int period)  //Period in us, only x
{
    reg_wr((u32)TIM2_ARR, period*16);     //Preload

    reg_wr((u32)TIM2_CCR1, period*8);    //Duty cycle 50 percent
}

static void set_y_pwm_period(int period)  //Period in us, only Y
{
    reg_wr((u32)TIM5_ARR, period*16);     //Preload

    reg_wr((u32)TIM5_CCR2, period*8);    //Duty cycle 50 percent
}



/**
 * @brief Creates a delay, cycles are lost.
 *
 * @param us Length of delay.
 */
void delay_us(int us)
{
    for (int i = 0; i < us*6 ; i++)
    {
        asm volatile("nop");
    }

}

/**
 * @brief Creates a delay. Max precision is 10ms
 *
 * @param ms Length of delay.
 */
void delay_rtos(int ms)
{
    u32 time = time_now();

    while(time_since(time) < ms);
}

void fsm_idle(void)
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
                fsm_transition(HEAD_MOVE);
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

void fsm_head_move(void)
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

/**
 * @brief Called by FreeRTOS memory manager when an malloc() has failed
 *
 */
void vApplicationMallocFailedHook(void)
{
    //uart_puts("vApplicationMallocFailedHook()\r\n");
    while(1);
}

/**
 * @brief Called periodically by squeduler as thread-safe tick handler
 *
 */
void vApplicationTickHook(void)
{
    //TickType_t tickCount = xTaskGetTickCountFromISR();
}

/**
 * @brief Called by FreeRTOS memory manager when a stack overflow is detected
 *
 * @param pxTask Id of the task where an overflow has been detected
 * @param pcTaskName Pointer to the task name
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //uart_puts("vApplicationStackOverflowHook()\r\n");
    while(1);
}

/**
 * @brief Called by FreeRTOS when an "assert" test has failed
 *
 * @param ulLine Line number where the assert has failed (in source)
 * @param pcFileName Pointer to a string with the name of the source file
 */
void vAssertCalled(unsigned long ulLine, const char * const pcFileName)
{
    //uart_flush();
    //uart_puts("vAssertCalled() file=");
    //uart_puts((char *)pcFileName);
    //uart_puts(" line=");
    //uart_putdec(ulLine);
    //uart_puts("\r\n");
    //uart_flush();
    while(1);
}
/* EOF */

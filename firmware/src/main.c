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

void delay_us(int us);
void delay_rtos(int ms);
static void task_first_task(void *pvParameters);
void task_console(void *pvParameters);
static void pwm_initilize(void);
static void init_io(void);
static void set_pwm_period(int period);



int main(void)
{
    u32 val;
    u32 a_mode, b_mode, g_mode;

    /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 0); /* GPIO-A */
    val |= (1 << 1); /* GPIO-B */
    val |= (1 << 2); /* GPIO-C */
    val |= (1 << 6); /* GPIO-G */
    reg_wr((u32)RCC_AHB1ENR, val);

    //a_mode = reg_rd((u32)GPIOA_MODER);
    //a_mode |= (1 << 0);
    //reg_wr((u32)GPIOA_MODER, a_mode); 

    b_mode = reg_rd((u32)GPIOB_MODER);
    b_mode |= (1 << 14);
    reg_wr((u32)GPIOB_MODER, b_mode); 

    g_mode = reg_rd((u32)GPIOG_MODER);
    g_mode |= (1 << 4);
    reg_wr((u32)GPIOG_MODER, g_mode);   


	while(1)
    {
        val = reg_rd((u32)GPIOG_BSRR);
        val |= (1 << 2);
        reg_wr((u32)GPIOG_BSRR, val);

        val = reg_rd((u32)GPIOB_BSRR);
        val |= (1 << 23);
        reg_wr((u32)GPIOB_BSRR, val);

        delay_us(1000000);

        val = reg_rd((u32)GPIOB_BSRR);
        val |= (1 << 7);
        reg_wr((u32)GPIOB_BSRR, val);

        delay_us(1000000);

        xTaskCreate(task_first_task, "My first Task", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY+1), NULL);
        xTaskCreate(task_console, "Console", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY+1), NULL);

        
        /* Start the tasks and timer running. */
        vTaskStartScheduler();

        while(1);           
    }
}

static void task_first_task(void *pvParameters)
{
    u32 val;

    delay_us(1000000);

    pwm_initilize();
    init_io();

    while(1)
    {
        reg_set((u32)GPIOC_BSRR, (1 << 1));     //Y GO UP - PC1 HIGH
        set_pwm_period(400);
        reg16_set((u32)TIM5_CCER, (1 << 4));    //Enable PWM

        delay_us(2000000);

        set_pwm_period(100);

        delay_us(2000000);

    }
    
    while(1)
    {
        reg16_clr((u32)TIM5_CCER, (1 << 4));    //Disable PWM

        delay_us(2000000);

        reg_set((u32)GPIOC_BSRR, (1 << 1));     //Y GO UP - PC1 HIGH

        reg16_set((u32)TIM5_CCER, (1 << 4));    //Enable PWM


        delay_us(1000000);
        //for(int i=0;i<1600000;i++) //Wait 1 s
        //{
        //    val++;
        //}

        reg16_clr((u32)TIM5_CCER, (1 << 4));    //Disable PWM

        delay_us(2000000);

        reg_set((u32)GPIOC_BSRR, (1 << 17));     //Y GO DOWN - PC1 LOW

        reg16_set((u32)TIM5_CCER, (1 << 4));    //Enable PWM

        delay_us(1000000);
    }

    while(1)
    {
        reg16_clr((u32)TIM2_CCER, (1 << 0));    //Disable PWM

        delay_us(2000000);

        reg_set((u32)GPIOC_BSRR, (1 << 0));     //X GO RIGHT - PC0 HIGH

        reg16_set((u32)TIM2_CCER, (1 << 0));    //Enable PWM


        delay_us(1000000);
        //for(int i=0;i<1600000;i++) //Wait 1 s
        //{
        //    val++;
        //}

        reg16_clr((u32)TIM2_CCER, (1 << 0));    //Disable PWM

        delay_us(2000000);

        reg_set((u32)GPIOC_BSRR, (1 << 16));     //X GO LEFT - PC0 LOW

        reg16_set((u32)TIM2_CCER, (1 << 0));    //Enable PWM

        delay_us(1000000);
    }
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

    reg16_set((u32)TIM2_CR1, (1 << 7)); //Auto-reload preload enable

    //reg16_set((u32)TIM2_PSC, 0);        //Prescaler = 0

    reg_wr((u32)TIM2_ARR, 3200);     //Preload = 3200 (Period = 400us)

    reg_wr((u32)TIM2_CCR1, 1600);    //Duty cycle 50 percent

    //reg_set((u32)TIM2_CCMR1, (0 << 0)); //CC1 channel configured as output

    //reg_set((u32)TIM2_CCER, (0 << 1));  //Set polarity to active high

    reg_set((u32)TIM2_CCMR1, (6 << 4)); //Output Compare 1 mode set to PWM mode 1
    //reg_set((u32)TIM2_CCMR1, (0 << 16)); 

    reg_set((u32)TIM2_CCMR1, (1 << 3)); //Preload enable

    reg16_set((u32)TIM2_CCER, (1 << 0));  //Enable CC1

    reg16_set((u32)TIM2_EGR, (1 << 0)); //Update generation

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

    reg16_set((u32)TIM5_CR1, (1 << 7)); //Auto-reload preload enable

    reg_wr((u32)TIM5_ARR, 3200);     //Preload = 3200 (Period = 400us)

    reg_wr((u32)TIM5_CCR2, 1600);    //Duty cycle 50 percent

    reg_set((u32)TIM5_CCMR1, (6 << 12)); //Output Compare 2 mode set to PWM mode 1

    reg_set((u32)TIM5_CCMR1, (1 << 11)); //Preload enable
   
    reg16_set((u32)TIM5_CCER, (1 << 4));  //Enable CC2

    reg16_set((u32)TIM5_EGR, (1 << 0)); //Update generation

    reg16_set((u32)TIM2_CR1, (1 << 0)); //Start timer

    reg16_set((u32)TIM5_CR1, (1 << 0)); //Start timer
}

static void init_io(void)
{
//XYZ_EN PA4
    reg_clr((u32)GPIOA_MODER, 0x300);       //Clear bits 8 and 9
    reg_set((u32)GPIOA_MODER, (1 << 8));    //Output mode

    reg_set((u32)GPIOA_BSRR, (1 << 4));     //PA4 HIGH = ENABLE

//X_DIR PC0
    reg_clr((u32)GPIOC_MODER, 0x3);         //Clear bits 0 and 1
    reg_set((u32)GPIOC_MODER, (1 << 0));    //Output mode

    reg_set((u32)GPIOC_BSRR, (1 << 0));     //PC0 HIGH

//Y_DIR PC1
    reg_clr((u32)GPIOC_MODER, 0x7);         //Clear bits 2 and 3
    reg_set((u32)GPIOC_MODER, (1 << 2));    //Output mode

    reg_set((u32)GPIOC_BSRR, (1 << 1));     //PC1 HIGH    
}

static void set_pwm_period(int period)  //Period in us, only Y
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

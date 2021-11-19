/**
 * @file  main.c
 * @brief Main function of the firmware
 *
 * @author Saint-Genest Gwenael <gwen@agilack.fr>
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

#include "types.h"
#include "hardware.h"
#include "rtos/include/FreeRTOS.h"
#include "rtos/include/task.h"

void delay_us(int us);
static void task_first_task(void *pvParameters);
static void pwm_initilize(void);

int main(void)
{
    u32 val;
    u32 a_mode, b_mode, g_mode;

    /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 0); /* GPIO-A */
    val |= (1 << 1); /* GPIO-B */
    val |= (1 << 6); /* GPIO-G */
    reg_wr((u32)RCC_AHB1ENR, val);

    a_mode = reg_rd((u32)GPIOA_MODER);
    a_mode |= (1 << 0);
    reg_wr((u32)GPIOA_MODER, a_mode); 

    b_mode = reg_rd((u32)GPIOB_MODER);
    b_mode |= (1 << 14);
    reg_wr((u32)GPIOB_MODER, b_mode); 

    g_mode = reg_rd((u32)GPIOG_MODER);
    g_mode |= (1 << 4);
    reg_wr((u32)GPIOG_MODER, g_mode);   

    pwm_initilize();

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
        
        /* Start the tasks and timer running. */
        vTaskStartScheduler();

        while(1);           
    }
}

static void task_first_task(void *pvParameters)
{
    u32 val;


    while(1)
    {
        val = reg_rd((u32)GPIOG_BSRR);
        val |= (1 << 18);
        reg_wr((u32)GPIOG_BSRR, val);

        delay_us(100000);

        val = reg_rd((u32)GPIOG_BSRR);
        val |= (1 << 2);
        reg_wr((u32)GPIOG_BSRR, val);

        //delay_us(100000);
        for(int i=0;i<16000000;i++)
        {
            val++;
        }
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

    reg_wr((u32)TIM2_ARR, 3200);     //Preload = 3200 (Period = 200us)

    reg_wr((u32)TIM2_CCR1, 1600);    //Duty cycle 50 percent

    //reg_set((u32)TIM2_CCMR1, (0 << 0)); //CC1 channel configured as output

    //reg_set((u32)TIM2_CCER, (0 << 1));  //Set polarity to active high

    reg_set((u32)TIM2_CCMR1, (6 << 4)); //Output Compare 1 mode set to PWM mode 1
    //reg_set((u32)TIM2_CCMR1, (0 << 16)); 

    reg_set((u32)TIM2_CCMR1, (1 << 3)); //Preload enable

    reg16_set((u32)TIM2_CCER, (1 << 0));  //Enable CC1

    reg16_set((u32)TIM2_EGR, (1 << 0)); //Update generation

    ////reg_set((u32)TIM2_BDTR, (1 << 15)); //Main Output Enable

    reg16_set((u32)TIM2_CR1, (1 << 0)); //Start timer
}

/**
 * @brief Creates a delay, cycles are lost.
 *
 * @param us Length of delay.
 */
void delay_us(int us)
{
    for (int i = 0; i < us ; i++)
    {
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
    }

}

/**
 * @brief Read the value of a 32bits memory mapped register
 *
 * @param reg Address of the register to read
 * @return u32 Value of the register
 */
u32 reg_rd(u32 reg)
{
    return ( *(volatile u32 *)reg );
}

/**
 * @brief Write a value to a 32bits memory mapped register
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg_wr(u32 reg, u32 value)
{
    *(volatile u32 *)reg = value;
}

/**
 * @brief Modify a 32bits memory mapped register by setting some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to set
 */
void reg_set(u32 reg, u32 value)
{
    *(volatile u32 *)reg = ( *(volatile u32 *)reg | value );
}

/**
 * @brief Modify a 32bits memory mapped register by clearing some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to clear
 */
void reg_clr(u32 reg, u32 value)
{
    *(volatile u32 *)reg = ( *(volatile u32 *)reg & ~value );
}

/**
 * @brief Read the value of a 16bits memory mapped register
 *
 * @param  reg Address of the register to read
 * @return u16 Value of the register
 */
u16 reg16_rd(u32 reg)
{
        return ( *(volatile u16 *)reg );
}


/**
 * @brief Write a value to a 16bits memory mapped register
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg16_wr(u32 reg, u16 value)
{
        *(volatile u16 *)reg = value;
}

/**
 * @brief Modify a 16bits memory mapped register by clearing some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to clear
 */
void reg16_clr(u32 reg, u16 value)
{
        *(volatile u16 *)reg = ( *(volatile u16 *)reg & ~value );
}

/**
 * @brief Modify a 16bits memory mapped register by setting some bits
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg16_set(u32 reg, u16 value)
{
        *(volatile u16 *)reg = ( *(volatile u16 *)reg | value );
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

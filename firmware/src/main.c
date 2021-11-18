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

int main(void)
{
    u32 val;
    u32 b_mode;

    /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 1); /* GPIO-B */
    reg_wr((u32)RCC_AHB1ENR, val);

    b_mode = reg_rd((u32)GPIOB_MODER);
    b_mode |= (1 << 14);
    reg_wr((u32)GPIOB_MODER, b_mode);   

	while(1)
    {
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
        val = reg_rd((u32)GPIOB_BSRR);
        val |= (1 << 23);
        reg_wr((u32)GPIOB_BSRR, val);

        delay_us(500000);

        val = reg_rd((u32)GPIOB_BSRR);
        val |= (1 << 7);
        reg_wr((u32)GPIOB_BSRR, val);

        delay_us(500000);
    }
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

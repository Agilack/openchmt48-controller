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
#include "motor.h"
#include "time.h"
#include "pnp.h"
#include "queue.h"
#include "uart.h"
#include "test.h"

void delay_us(int us);
void delay_rtos(int ms);
void task_pnp(void *pvParameters);
void task_console(void *pvParameters);

volatile int pos_x, pos_y;

xQueueHandle cmd_queue;

int main(void)
{
    cmd_queue = xQueueCreate(10, sizeof(int));

    xTaskCreate(task_console, "Console", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY+1), NULL);
    xTaskCreate(task_pnp, "PickNPlace", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY+1), NULL);

    hw_init();

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    uart_puts(" * FAILED TO START SQUEDULER\r\n");

    while(1);
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
        encoder_Y_A_irq();
    }

    if (val & (1 << 13))        //EXTI13    Y_B
    {
        encoder_Y_B_irq();
    }

    if (val & (1 << 14))        //EXTI14    X_A
    {
        encoder_X_A_irq();
    }

    if (val & (1 << 15))        //EXTI15    X_B
    {
        encoder_X_B_irq();
    }
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

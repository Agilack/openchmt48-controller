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
static void task_console(void *pvParameters);
static void pwm_initilize(void);
static void init_io(void);
static void set_pwm_period(int period);
static void init_uart(void);
static void uart_putc(char c);
void uart_puts (char *s);
int uart_getc(unsigned char *c);

#define BUFFER_SIZE 1024
static u8 buffer[BUFFER_SIZE];
u8 rx_buffer[BUFFER_SIZE];
static int buffer_r, buffer_w;
static volatile int i_rx_buff, i_rx_buff2;

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

static void task_console(void *pvParameters)
{
    init_uart();

    unsigned char mychar=0;

    while(1)
    {
        if(i_rx_buff != i_rx_buff2)
        {
            uart_putc(rx_buffer[i_rx_buff2]);
            i_rx_buff2++;
        }
        
        //    uart_putc(mychar);
        uart_puts("ABC\r\n");
        delay_us(1000000);
    }

    while(1)
    {
        //uart_putc('A');
        //uart_puts("TEST\n\r");
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

static void init_uart(void)
{
    u32 val;

      /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 4); /* GPIO-E */
    reg_wr((u32)RCC_AHB1ENR, val);

    val = reg_rd((u32)GPIOE_MODER);
    val |= (2 << 14);                 //PE7 alternate function mode
    reg_wr((u32)GPIOE_MODER, val); 

    val = reg_rd((u32)GPIOE_MODER);
    val |= (2 << 16);                 //PE8 alternate function mode
    reg_wr((u32)GPIOE_MODER, val); 

    val = reg_rd((u32)GPIOE_AFRL);
    val |= (8 << 28);                    //AF8 (UART7_Rx) on PE7
    reg_wr((u32)GPIOE_AFRL, val);

    val = reg_rd((u32)GPIOE_AFRH);
    val |= (8 << 0);                     //AF8 (UART7_Tx) on PE8
    reg_wr((u32)GPIOE_AFRH, val);

    val = reg_rd((u32)RCC_APB1ENR);     //UART7 Enable
    val |= (1 << 30);
    reg_wr((u32)RCC_APB1ENR, val);

    reg_wr(UART_BRR, 139); /* 115200 @ 16MHz       */

    reg_wr(UART_CR1,   0x0C); /* Set TE & RE bits     */
    reg_wr(UART_CR1,   0x0D); /* Set USART Enable bit */

    buffer_r = 0;
    buffer_w = 0;

    i_rx_buff = 0;
    i_rx_buff2 = 0;

    reg_wr(0xE000E108, (1 << 18)); /* USART7 NVIC */

    reg_set(UART_CR1, (1 << 5));    //RXNE interupt enable
}

/**
 * @brief Interrupt handler
 *
 * This function is called by CPU when an UART interrupt signal is
 * triggered.
 */
void UART7_IRQHandler(void)
{
    u32 isr = reg_rd(UART_ISR);

    if (isr & (1 << 7))
    {
        if (buffer_r != buffer_w)
        {
            reg_wr(UART_TDR, buffer[buffer_r]);
            buffer_r++;
            if (buffer_r > (BUFFER_SIZE-1))
                buffer_r = 0;
        }
        else
            reg_clr(UART_CR1, (1 << 7));
    }
    if(isr & (1 << 5))
    {
        rx_buffer[i_rx_buff]=reg_rd(UART_RDR);
        i_rx_buff++;
        if(i_rx_buff > (BUFFER_SIZE-1))
            i_rx_buff = 0;
    }
    if(isr & (1 << 3))
    {
        reg_set(UART_ICR, (1 << 3));
    }

}

static void uart_putc(char c)
{
    int next;
    int use_isr;

    /* Tests if UART interrupt is active into NVIC */
    use_isr = (reg_rd(0xE000E108) & (1 << 18)) ? 1 : 0; /* USART7 */

    /* If UART interrupt is active, put byte into TX buffer */
    if (use_isr)
    {
        next = (buffer_w + 1);
        if (next > (BUFFER_SIZE-1))
            next = 0;
        if (next == buffer_r)
            return;
        buffer[buffer_w] = c;
        buffer_w = next;
        reg_set(UART_CR1, (1 << 7));
    }
    /* UART interrupt is inactive, use synchronous write to uart */
    else
    {
        while ((reg_rd(UART_ISR) & (1 << 7)) == 0)
            ;
        reg_wr(UART_TDR, c);
    }
}

/**
 * @brief Send a text string to UART
 *
 * @param s Pointer to the string to send
 */
void uart_puts (char *s)
{
    while (*s)
    {
        uart_putc(*s);
        s++;
    }
}

/**
 * @brief Read one byte received on UART
 *
 * @param c Pointer to a byte variable where to store recived data
 * @return True if a byte has been received, False if no data available
 */
int uart_getc(unsigned char *c)
{
    unsigned char rx;

    if (reg_rd(UART_ISR) & (1 << 5))
    {
        /* Get the received byte from RX fifo */
        rx = reg_rd(UART_RDR);
        /* If a data pointer has been defined, copy received byte */
        if (c)
            *c = rx;
        return(1);
    }
    return (0);
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

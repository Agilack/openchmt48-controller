/**
 * @file  console.c
 * @brief Functions related to the console
 *
 * @author Axel Paolillo <axel.paolillo@agilack.fr>
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

#include "types.h"
#include "hardware.h"
#include "time.h"
#include <string.h>
#include <math.h>

void delay_us(int us);
void delay_rtos(int ms);
void task_console(void *pvParameters);
static void init_uart(void);
static void uart_putc(char c);
void uart_puts (char *s);
int uart_getc(unsigned char *c);

#define BUFFER_SIZE 1024
static u8 buffer[BUFFER_SIZE];
u8 rx_buffer[BUFFER_SIZE];
char cmd_buffer[BUFFER_SIZE];
char cmd_buffer_old[BUFFER_SIZE];
static int buffer_r, buffer_w;
volatile int i_rx_buff, i_rx_buff2, i_cmd;

int target_pos_x = 0, target_pos_y = 0;

void task_console(void *pvParameters)
{
    init_uart();

    char flag_cmd=0;
    char flag_esc_seq=0;

    //u8 cmd_string[1024];

    while(1)
    {
        if(i_rx_buff != i_rx_buff2)
        {
            if (flag_esc_seq)
            {
                if (flag_esc_seq==1 && rx_buffer[i_rx_buff2] == '[')
                {
                    flag_esc_seq = 2;
                }
                else if(flag_esc_seq==2 && rx_buffer[i_rx_buff2] == 'A')
                {
                    //flag_esc_seq = 0;
                    //uart_puts("FLECHE HAUT!");
                    strcpy(cmd_buffer, cmd_buffer_old);
                    uart_puts("\033[2K");               //Clears line
                    uart_puts("\033[G");                //Cursor column 1
                    uart_puts(cmd_buffer);
                    i_cmd = strlen(cmd_buffer);
                }
                else
                    flag_esc_seq = 0;
            }
            


            switch(rx_buffer[i_rx_buff2])
            {
                case '\033':  //ESCAPE SEQUENCE
                flag_esc_seq=1;
                break;

                case 8:
                uart_puts("\b \b");
                if(i_cmd>0)
                    i_cmd--;
                break;

                case 13:
                flag_cmd = 1;
                uart_puts("\r\n");
                cmd_buffer[i_cmd]='\0';
                break;

                default:
                if(flag_esc_seq == 0)
                {
                    uart_putc(rx_buffer[i_rx_buff2]);
                    cmd_buffer[i_cmd]=rx_buffer[i_rx_buff2];
                    i_cmd++;
                }
                break;

            }

            i_rx_buff2++;
            if (i_rx_buff2 > (BUFFER_SIZE-1))
                i_rx_buff2=0;
        }



        if(flag_cmd)
        {
            flag_cmd = 0;
            //uart_puts(cmd_buffer);
            //uart_puts("\r\n");

            i_cmd = 0;

        /*Commands with arguments*/
            if (strchr(cmd_buffer, ' ')) 
            {

            //uart_puts("SPAAAAAAAACE\r\n");
                if (strncmp(cmd_buffer, "move ", 5) == 0) 
                {
                    char *args = &(cmd_buffer[5]);
                    if(strncmp(args, "x:", 2) == 0)
                    {
                    //uart_puts("MOVE X OR X and Y DETECTED\r\n");
                        if (strchr(args, ' '))
                        {
                            char *args2 = strchr(args,' ');
                            int x_digits = strlen(args)-strlen(args2)-2;
                            int y_digits = strlen(args2)-3;

                            int x_decimal=0;
                            for(int i=0;i<x_digits;i++)
                            {
                                x_decimal += (args[2+i]-'0')*pow(10,x_digits-1-i);
                            }
                            target_pos_x=x_decimal;

                            int y_decimal=0;
                            for(int i=0;i<y_digits;i++)
                            {
                                y_decimal += (args2[3+i]-'0')*pow(10,y_digits-1-i);
                            }
                            target_pos_y=y_decimal;

                            uart_puts("MOVE X AND Y DETECTED\r\n");
                            //x_goto_2(target_pos_x);
                            //y_goto_2(target_pos_y);

                        }
                        else
                        {
                            int x_digits = strlen(cmd_buffer)-7;
                            int x_decimal=0;
                            for(int i=0;i<x_digits;i++)
                            {
                                x_decimal += (args[2+i]-'0')*pow(10,x_digits-1-i);
                            }
                            target_pos_x=x_decimal;
                            uart_puts("MOVE ONLY X DETECTED\r\n");
                            //x_goto_2(target_pos_x);
                        }
                    }
                    else if(strncmp(args, "y:", 2) == 0)
                    {
                        int y_digits = strlen(cmd_buffer)-7;
                        int y_decimal=0;
                        for(int i=0;i<y_digits;i++)
                        {
                            y_decimal += (args[2+i]-'0')*pow(10,y_digits-1-i);
                        }
                        target_pos_y=y_decimal;
                        uart_puts("MOVE ONLY Y DETECTED\r\n");
                        //y_goto_2(target_pos_y);
                    }
                    else
                    {
                        uart_puts("MOVE ERROR\r\n");
                    }
                }
                if (strncmp(cmd_buffer, "nozzle ", 7) == 0)
                {
                    char *args = &(cmd_buffer[7]);
                    if(strncmp(args, "move ", 5) == 0)
                    {

                        uart_puts("NOZZLE MOVE DETECTED\r\n");
                        char *args2 = &(args[5]);
                        if(strncmp(args2, "1", 1) == 0)
                        {
                            if (strchr(args2, ' '))
                            {
                                char *args3 = &(args2[2]);
                                int z_digits = strlen(cmd_buffer)-14;
                                int z_decimal=0;
                                for(int i=0;i<z_digits;i++)
                                {
                                    z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
                                }
                                uart_puts("MOVE NOZZLE 1 DETECTED\r\n");
                            }
                        }
                        else if(strncmp(args2, "2", 1) == 0)
                        {
                            if (strchr(args2, ' '))
                            {
                                char *args3 = &(args2[2]);
                                int z_digits = strlen(cmd_buffer)-14;
                                int z_decimal=0;
                                for(int i=0;i<z_digits;i++)
                                {
                                    z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
                                }

                                uart_puts("MOVE NOZZLE 2 DETECTED\r\n");
                            }
                        }
                    }
                    else if(strncmp(args, "down ", 5) == 0)
                    {
                        char *args2 = &(args[5]);
                        if(strncmp(args2, "1", 1) == 0)
                        {
                            //nozzle_down_1();
                        }
                        else if(strncmp(args2, "2", 1)==0)
                        {
                            //nozzle_down_2();
                        }
                    }
                    else if(strncmp(args, "up ", 3) == 0)
                    {
                        char *args2 = &(args[3]);
                        if(strncmp(args2, "1", 1) == 0)
                        {
                            //nozzle_up_1();
                        }
                        else if(strncmp(args2, "2", 1)==0)
                        {
                            //nozzle_up_2();
                        }
                    }
                    else if(strncmp(args, "reset", 5) == 0)
                    {
                        //nozzle_reset();
                    }
                    else if(strncmp(args, "rotate ", 7) == 0)
                    {
                        uart_puts("NOZZLE ROTATE DETECTED\r\n");

                        char *args2 = &(args[7]);
                        if(strncmp(args2, "1", 1) == 0)
                        {
                            if (strchr(args2, ' '))
                            {
                                char *args3 = &(args2[2]);
                                int z_digits = strlen(cmd_buffer)-16;
                                int z_decimal=0;
                                for(int i=0;i<z_digits;i++)
                                {
                                    z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
                                }
                                uart_puts("ROTATE NOZZLE 1 DETECTED\r\n");
                            }
                        }
                        else if(strncmp(args2, "2", 1) == 0)
                        {
                            if (strchr(args2, ' '))
                            {
                                char *args3 = &(args2[2]);
                                int z_digits = strlen(cmd_buffer)-16;
                                int z_decimal=0;
                                for(int i=0;i<z_digits;i++)
                                {
                                    z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
                                }
                                uart_puts("ROTATE NOZZLE 2 DETECTED\r\n");
                            }
                        }
                    }
                }
                uart_puts("DONE\r\n");
            }

        /*Commands without arguments*/
            else 
            {
                if (strcmp(cmd_buffer, "enable-xyz") == 0) 
                {
                    uart_puts("WOW ENABLE\r\n");
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //XYZ ENABLE
                }
                else if (strcmp(cmd_buffer, "disable-xyz") == 0) 
                {
                    uart_puts("WOW DISABLE\r\n");
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   //XYZ DISABLE
                }
                else if (strcmp(cmd_buffer, "enable-bcd") == 0) 
                {
                    uart_puts("WOW ENABLE\r\n");
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //BCD ENABLE
                }
                else if (strcmp(cmd_buffer, "disable-bcd") == 0) 
                {
                    uart_puts("WOW DISABLE\r\n");
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   //BCD DISABLE
                }
                else if (strcmp(cmd_buffer, "square") == 0) 
                {
                    //while(1)
                    //amazing_square();
                }

            }
            if(strcmp(cmd_buffer, "\0") != 0)
            {
                uart_puts("\n");
            }

            strcpy(cmd_buffer_old, cmd_buffer);


            i_cmd=0;
        }
    }
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

    i_cmd = 0;

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
        reg_wr(UART_ICR, (1 << 3));
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
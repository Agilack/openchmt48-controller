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
#include "FreeRTOS.h"
#include "queue.h"
#include "uart.h"

void delay_us(int us);
void delay_rtos(int ms);
void task_console(void *pvParameters);
void nozzle_reset(void);
void nozzle_down_1(void);
void nozzle_down_2(void);
void nozzle_rot_1(void);
void nozzle_rot_2(void);


#define BUFFER_SIZE 1024
u8 buffer[BUFFER_SIZE];
u8 rx_buffer[BUFFER_SIZE];
char cmd_buffer[BUFFER_SIZE];
char cmd_buffer_old[BUFFER_SIZE];
int buffer_r, buffer_w;
volatile int i_rx_buff, i_rx_buff2, i_cmd;

int target_pos_x = 0, target_pos_y = 0;
extern xQueueHandle cmd_queue;
extern int pos_x, pos_y;

void task_console(void *pvParameters)
{
    init_uart();

    char flag_cmd=0;
    char flag_esc_seq=0;
    int cmd_id=0;

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
                else if(flag_esc_seq==2 && rx_buffer[i_rx_buff2] == 'B')    //Down arrow
                {
                    uart_puts("\033[2K");               //Clears line
                    uart_puts("\033[G");                //Cursor column 1
                    i_cmd = 0;
                }
                else if(flag_esc_seq==2 && rx_buffer[i_rx_buff2] == 'C')    //Right arrow
                {
                    uart_puts("\033[1C");
                    if(i_cmd < 70)
                    {
                        i_cmd++;
                    }
                }
                else if(flag_esc_seq==2 && rx_buffer[i_rx_buff2] == 'D')    //Left arrow
                {
                    uart_puts("\033[1D");
                    if(i_cmd > 0)
                    {
                        i_cmd--;
                    }
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
                            cmd_id = 3;
                            xQueueSendToBack(cmd_queue, &cmd_id, portMAX_DELAY);
                            xQueueSendToBack(cmd_queue, &x_decimal, portMAX_DELAY);
                            xQueueSendToBack(cmd_queue, &y_decimal, portMAX_DELAY);                            

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
                            cmd_id = 1;
                            xQueueSendToBack(cmd_queue, &cmd_id, portMAX_DELAY);
                            xQueueSendToBack(cmd_queue, &x_decimal, portMAX_DELAY);
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
                        cmd_id = 2;
                        xQueueSendToBack(cmd_queue, &cmd_id, portMAX_DELAY);
                        xQueueSendToBack(cmd_queue, &y_decimal, portMAX_DELAY);
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
                            nozzle_down_1();
                        }
                        else if(strncmp(args2, "2", 1)==0)
                        {
                            nozzle_down_2();
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
                        nozzle_reset();
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
                                nozzle_rot_1();
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
                                nozzle_rot_2();
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
                    reg_set((u32)GPIOA_BSRR, (1 << 4));     //XYZ ENABLE
                }
                else if (strcmp(cmd_buffer, "disable-xyz") == 0) 
                {
                    uart_puts("WOW DISABLE\r\n");
                    reg_set((u32)GPIOA_BSRR, (1 << 20));     //XYZ DISABLE
                }
                else if (strcmp(cmd_buffer, "enable-bcd") == 0) 
                {
                    uart_puts("WOW ENABLE\r\n");
                    reg_set((u32)GPIOA_BSRR, (1 << 5));     //BCD ENABLE
                }
                else if (strcmp(cmd_buffer, "disable-bcd") == 0) 
                {
                    uart_puts("WOW DISABLE\r\n");
                    reg_set((u32)GPIOA_BSRR, (1 << 21));     //BCD DISABLE
                }
                else if (strcmp(cmd_buffer, "square") == 0) 
                {
                    //while(1)
                    //amazing_square();
                }
                else if (strcmp(cmd_buffer, "pos") == 0)
                {
                    uart_puts("X position : ");
                    uart_putdec(pos_x);
                    uart_puts("\r\n");
                    uart_puts("Y position : ");
                    uart_putdec(pos_y);
                    uart_puts("\r\n");
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


void nozzle_down_1(void)
{
    reg_set((u32)GPIOF_BSRR, (1 << 13));     //Left nozzle down

    for (int i = 0; i < 125; i++) 
    {
        reg_set((u32)GPIOA_BSRR, (1 << 6));     //NOZZLE_CLK HIGH
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
        reg_set((u32)GPIOA_BSRR, (1 << 22));     //NOZZLE_CLK LOW
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
    }

}

void nozzle_down_2(void)
{
    reg_set((u32)GPIOF_BSRR, (1 << 29));     //Right nozzle down

    for (int i = 0; i < 125; i++) 
    {
        reg_set((u32)GPIOA_BSRR, (1 << 6));     //ROT1-NOZZLE_CLK HIGH
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
        reg_set((u32)GPIOA_BSRR, (1 << 22));    //ROT1-NOZZLE_CLK LOW
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
    }
}

void nozzle_rot_1(void)
{
    //reg_set((u32)GPIOF_BSRR, (1 << 15)); //Counter-Clockwise
    reg_set((u32)GPIOF_BSRR, (1 << 31));   //Clockwise


    for (int i = 0; i < 125; i++) 
    {
        reg_set((u32)GPIOA_BSRR, (1 << 7));     //ROT1-NOZZLE_CLK HIGH
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
        reg_set((u32)GPIOA_BSRR, (1 << 23));    //ROT1-NOZZLE_CLK LOW
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
    }
}

void nozzle_rot_2(void)
{
    //reg_set((u32)GPIOF_BSRR, (1 << 14));   //Counter-Clockwise
    reg_set((u32)GPIOF_BSRR, (1 << 30));     //Clockwise

    for (int i = 0; i < 125; i++) 
    {
        reg_set((u32)GPIOB_BSRR, (1 << 0));     //ROT1-NOZZLE_CLK HIGH
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
        reg_set((u32)GPIOB_BSRR, (1 << 16));    //ROT1-NOZZLE_CLK LOW
        delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
    }
}

void nozzle_reset(void)
{
    reg_set((u32)GPIOA_BSRR, (1 << 21));     //BCD DISABLE
   
    delay_rtos(100);    //100ms

    reg_set((u32)GPIOA_BSRR, (1 << 5));     //BCD ENABLE
}




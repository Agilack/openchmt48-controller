/**
 * @file  uart.c
 * @brief Functions to handle uart
 *
 * @authors Saint-Genest Gwenael <gwen@agilack.fr>
 *          Axel Paolillo <axel.paolillo@agilack.fr>
 * @copyright Agilack (c) 2018-2020
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
#include "uart.h"
#include "hardware.h"

#define BUFFER_SIZE 1024

static int b2ds(char *d, int n, int pad, int zero);
extern int buffer_r;
extern int buffer_w;
extern int i_rx_buff;
extern int i_rx_buff2;
extern int i_cmd;
extern u8 buffer[BUFFER_SIZE];

void init_uart(void)
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

void uart_putc(u8 c)
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
 * @brief Print a numerical value in decimal
 *
 * @param v Value to display
 */
void uart_putdec(const u32 v)
{
	char str[16];

	b2ds(str, v, 0, 1);
	uart_puts(str);
}

/**
 * @brief Convert an integer value to his decimal representation into an ASCII string
 *
 * @param d Pointer to a buffer for output string
 * @param n Interger value to convert
 * @param pad Align output value to ad least "pad" digits
 * @param zero Boolean flag, if set a \0 is added at the end of string
 */
static int b2ds(char *d, int n, int pad, int zero)
{
	unsigned int decade = 1000000000;
	int count = 0;
	int i;

	for (i = 0; i < 9; i++)
	{
		if ((n > (decade - 1)) || count || (pad >= (10-i)))
		{
			*d = (n / decade) + '0';
			n -= ((n / decade) * decade);
			d++;
			count++;
		}
		decade = (decade / 10);
	}
	*d = n + '0';
	count ++;

	if (zero)
		d[1] = 0;

	return(count);
}
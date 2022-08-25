/**
 * @file  uart.h
 * @brief Headers and definitions for UART driver
 *
 * @author Saint-Genest Gwenael <gwen@agilack.fr>
 * @copyright Agilack (c) 2019-2021
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */
#ifndef UART_H
#define UART_H

void init_uart(void);

int  uart_getc(unsigned char *c);
void uart_putc(u8 c);
void uart_puts (char *s);

void uart_putdec  (const u32 v);

#endif

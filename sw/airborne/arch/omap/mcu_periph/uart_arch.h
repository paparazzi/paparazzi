/*
 * $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef UART_ARCH_H
#define UART_ARCH_H

#include "std.h"
//coment to avoid redefinition
/*#define B9600     9600
#define B38400   38400
 #define B57600   57600
#define B115200 115200
*/

//junk for gps_configure_uart in gps_ubx.c to compile
#define UART_BAUD(baud) (baud)


#define Uart1_init uart1_init()
#define Uart2_init uart2_init()
#define Uart3_init uart3_init()
#define Uart5_init uart5_init()

#define UART1_irq_handler usart1_irq_handler
#define UART2_irq_handler usart2_irq_handler
#define UART3_irq_handler usart3_irq_handler
#define UART5_irq_handler usart5_irq_handler

#if defined USE_UART0 || OVERRIDE_UART0_IRQ_HANDLER
extern void uart0_handler(void);
#endif

#ifdef USE_UART0

void uart0_init( void );

#endif /* USE_UART0 */


#if defined USE_UART1 || OVERRIDE_UART1_IRQ_HANDLER
extern void uart1_handler(void);
#endif

#ifdef USE_UART1

void uart1_init( void );

#endif /* USE_UART1 */

#endif /* UART_ARCH_H */

/*
 * $Id$
 *
 * Copyright (C) 2009-2010 The Paparazzi Team
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

/*
 *\brief STM32 usart functions
 *
 */

#ifndef STM32_UART_ARCH_H
#define STM32_UART_ARCH_H

#include "std.h"

#define B4800     4800
#define B9600     9600
#define B38400   38400
#define B57600   57600
#define B115200 115200

#if defined USE_UART1 || OVERRIDE_UART1_IRQ_HANDLER
extern void usart1_isr(void);
#endif

#if defined USE_UART2 || OVERRIDE_UART2_IRQ_HANDLER
extern void usart2_isr(void);
#endif

#if defined USE_UART3 || OVERRIDE_UART3_IRQ_HANDLER
extern void usart3_isr(void);
#endif

#if defined USE_UART5 || OVERRIDE_UART5_IRQ_HANDLER
extern void uart5_isr(void);
#endif

#endif /* STM32_UART_ARCH_H */

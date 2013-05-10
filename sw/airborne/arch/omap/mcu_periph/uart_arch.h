/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/omap/mcu_periph/uart_arch.h
 * omap uart handling
 */

#ifndef UART_ARCH_H
#define UART_ARCH_H

#include "mcu_periph/uart.h"
#include "std.h"

#define UART1_irq_handler usart1_irq_handler
#define UART2_irq_handler usart2_irq_handler
#define UART3_irq_handler usart3_irq_handler
#define UART5_irq_handler usart5_irq_handler

#if defined USE_UART0 || OVERRIDE_UART0_IRQ_HANDLER
extern void uart0_handler(void);
#endif

#if defined USE_UART1 || OVERRIDE_UART1_IRQ_HANDLER
extern void uart1_handler(void);
#endif

#endif /* UART_ARCH_H */

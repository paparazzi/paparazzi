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

#define B9600     9600
#define B38400   38400
#define B57600   57600
#define B115200 115200

/* junk for gps_configure_uart in gps_ubx.c to compile */
#define UART_8N1 1
#define UART_FIFO_8 1

/* sort out the problem of UART5 already defined in stm32.h */
#define USART5               ((USART_TypeDef *) UART5_BASE)
#undef UART5

#define UART1_TxPin GPIO_Pin_9
#define UART2_TxPin GPIO_Pin_2
#define UART3_TxPin GPIO_Pin_10
#define UART5_TxPin GPIO_Pin_12

#define UART1_RxPin GPIO_Pin_10
#define UART2_RxPin GPIO_Pin_3
#define UART3_RxPin GPIO_Pin_11
#define UART5_RxPin GPIO_Pin_2

#define UART1_TxPort GPIOA
#define UART2_TxPort GPIOA
#define UART3_TxPort GPIOC
#define UART5_TxPort GPIOC

#define UART1_RxPort GPIOA
#define UART2_RxPort GPIOA
#define UART3_RxPort GPIOC
#define UART5_RxPort GPIOD

#define UART1_Periph RCC_APB2Periph_GPIOA
#define UART2_Periph RCC_APB2Periph_GPIOA
#define UART3_Periph RCC_APB2Periph_GPIOC
#define UART5_PeriphTx RCC_APB2Periph_GPIOC
#define UART5_PeriphRx RCC_APB2Periph_GPIOD

/* this is unexpected the macros in spektrum_arch.c
   didn't expect that rx and tx would be spilt over
   two ports. As the spektrum code is only interested
   in the rx pin we define this to be the Peripheral */
#define UART5_Periph RCC_APB2Periph_GPIOD

#define UART1_UartPeriph RCC_APB2Periph_USART1
#define UART2_UartPeriph RCC_APB1Periph_USART2
#define UART3_UartPeriph RCC_APB1Periph_USART3
#define UART5_UartPeriph RCC_APB1Periph_UART5

#define UART1_remap {}
#define UART2_remap {}
#define UART3_remap {RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); \
                     GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);}
#define UART5_remap {}

#define UART1_clk(_periph, _val) RCC_APB2PeriphClockCmd(_periph, _val)
#define UART2_clk(_periph, _val) RCC_APB1PeriphClockCmd(_periph, _val)
#define UART3_clk(_periph, _val) RCC_APB1PeriphClockCmd(_periph, _val);
#define UART5_clk(_periph, _val) RCC_APB1PeriphClockCmd(_periph, _val)

#define Uart1_init uart1_init()
#define Uart2_init uart2_init()
#define Uart3_init uart3_init()
#define Uart5_init uart5_init()

#define UART1_irq_handler usart1_irq_handler
#define UART2_irq_handler usart2_irq_handler
#define UART3_irq_handler usart3_irq_handler
#define UART5_irq_handler usart5_irq_handler

#define UART1_IRQn USART1_IRQn
#define UART2_IRQn USART2_IRQn
#define UART3_IRQn USART3_IRQn

#define UART1_reg USART1
#define UART2_reg USART2
#define UART3_reg USART3
#define UART5_reg USART5


#if defined USE_UART1 || OVERRIDE_UART1_IRQ_HANDLER
extern void usart1_irq_handler(void);
#endif

#if defined USE_UART2 || OVERRIDE_UART2_IRQ_HANDLER
extern void usart2_irq_handler(void);
#endif

#if defined USE_UART3 || OVERRIDE_UART3_IRQ_HANDLER
extern void usart3_irq_handler(void);
#endif

#if defined USE_UART5 || OVERRIDE_UART5_IRQ_HANDLER
extern void usart5_irq_handler(void);
#endif

//void uart_init( void );

#endif /* STM32_UART_ARCH_H */

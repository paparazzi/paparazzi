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

/*
 *\brief STM32 usart functions 
 *
 */

#ifndef UART_HW_H
#define UART_HW_H

#include "std.h"

#define B9600     9600
#define B38400   38400
#define B57600   57600
#define B115200 115200


#define Uart1_TxPin UART1_TxPin 
#define Uart2_TxPin UART2_TxPin
#define Uart3_TxPin UART3_TxPin
#define UART1_TxPin GPIO_Pin_9
#define UART2_TxPin GPIO_Pin_2
#define UART3_TxPin GPIO_Pin_10

#define Uart1_RxPin UART1_RxPin  
#define Uart2_RxPin UART2_RxPin
#define Uart3_RxPin UART3_RxPin
#define UART1_RxPin GPIO_Pin_10
#define UART2_RxPin GPIO_Pin_3
#define UART3_RxPin GPIO_Pin_11

#define Uart1_TxPort UART1_TxPort
#define Uart2_TxPort UART2_TxPort
#define Uart3_TxPort UART3_TxPort
#define UART1_TxPort GPIOA
#define UART2_TxPort GPIOA
#define UART3_TxPort GPIOC

#define Uart1_RxPort UART1_RxPort
#define Uart2_RxPort UART2_RxPort
#define Uart3_RxPort UART3_RxPort
#define UART1_RxPort GPIOA
#define UART2_RxPort GPIOA
#define UART3_RxPort GPIOC

#define UART1_Periph RCC_APB2Periph_GPIOA
#define UART2_Periph RCC_APB2Periph_GPIOA
#define UART3_Periph RCC_APB2Periph_GPIOC
#define Uart1_UartPeriph UART1_UartPeriph
#define Uart2_UartPeriph UART2_UartPeriph
#define Uart3_UartPeriph UART3_UartPeriph


#define Uart1_Periph UART1_Periph
#define Uart2_Periph UART2_Periph
#define Uart3_Periph UART3_Periph
#define UART1_UartPeriph RCC_APB2Periph_USART1
#define UART2_UartPeriph RCC_APB1Periph_USART2
#define UART3_UartPeriph RCC_APB1Periph_USART3

#define UART1_remap Uart1_remap
#define UART2_remap Uart2_remap
#define UART3_remap Uart3_remap
#define UART5_remap Uart5_remap
#define Uart1_remap {}
#define Uart2_remap {}
#define Uart3_remap {RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); \
                     GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);}
#define Uart5_remap {}

#define Uart1_init uart1_init()
#define Uart2_init uart2_init()
#define Uart3_init uart3_init()
#define Uart5_init uart5_init()

#define UART1_irq_handler Uart1_irq_handler
#define UART2_irq_handler Uart2_irq_handler
#define UART3_irq_handler Uart3_irq_handler
#define UART5_irq_handler Uart5_irq_handler

#define Uart1_irq_handler usart1_irq_handler
#define Uart2_irq_handler usart2_irq_handler
#define Uart3_irq_handler usart3_irq_handler
#define Uart5_irq_handler uart5_irq_handler

#define UART1_IRQn Uart1_IRQn
#define UART2_IRQn Uart2_IRQn
#define UART3_IRQn Uart3_IRQn

#define Uart1_IRQn USART1_IRQn
#define Uart2_IRQn USART2_IRQn 
#define Uart3_IRQn USART3_IRQn
#define Uart5_IRQn UART5_IRQn

#define UART1_reg Uart1_reg 
#define UART2_reg Uart2_reg
#define UART3_reg Uart3_reg
#define UART5_reg Uart5_reg 

#define Uart1_reg USART1
#define Uart2_reg USART2
#define Uart3_reg USART3
#define Uart5_reg UART5  



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

#ifdef USE_UART1
#define UART1_RX_BUFFER_SIZE 128
#define UART1_TX_BUFFER_SIZE 128

extern volatile uint16_t uart1_rx_insert_idx, uart1_rx_extract_idx;
extern uint8_t  uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

extern volatile uint16_t uart1_tx_insert_idx, uart1_tx_extract_idx;
extern volatile bool_t   uart1_tx_running;
extern uint8_t  uart1_tx_buffer[UART1_TX_BUFFER_SIZE];

#define Uart1ChAvailable() (uart1_rx_insert_idx != uart1_rx_extract_idx)
#define Uart1Getch() ({							\
      uint8_t ret = uart1_rx_buffer[uart1_rx_extract_idx];		\
      uart1_rx_extract_idx = (uart1_rx_extract_idx + 1)%UART1_RX_BUFFER_SIZE; \
      ret;								\
    })

#endif /* USE_UART1 */


#ifdef USE_UART2

#define UART2_RX_BUFFER_SIZE 128
#define UART2_TX_BUFFER_SIZE 128

extern volatile uint16_t uart2_rx_insert_idx, uart2_rx_extract_idx;
extern uint8_t  uart2_rx_buffer[UART2_RX_BUFFER_SIZE];

extern volatile uint16_t uart2_tx_insert_idx, uart2_tx_extract_idx;
extern volatile bool_t   uart2_tx_running;
extern uint8_t  uart2_tx_buffer[UART2_TX_BUFFER_SIZE];

#define Uart2ChAvailable() (uart2_rx_insert_idx != uart2_rx_extract_idx)
#define Uart2Getch() ({							\
      uint8_t ret = uart2_rx_buffer[uart2_rx_extract_idx];		\
      uart2_rx_extract_idx = (uart2_rx_extract_idx + 1)%UART2_RX_BUFFER_SIZE; \
      ret;								\
    })

#endif /* USE_UART2 */


#ifdef USE_UART3

#define UART3_RX_BUFFER_SIZE 128
#define UART3_TX_BUFFER_SIZE 128

extern volatile uint16_t uart3_rx_insert_idx, uart3_rx_extract_idx;
extern uint8_t  uart3_rx_buffer[UART3_RX_BUFFER_SIZE];

extern volatile uint16_t uart3_tx_insert_idx, uart3_tx_extract_idx;
extern volatile bool_t   uart3_tx_running;
extern uint8_t  uart3_tx_buffer[UART3_TX_BUFFER_SIZE];

#define Uart3ChAvailable() (uart3_rx_insert_idx != uart3_rx_extract_idx)
#define Uart3Getch() ({							\
      uint8_t ret = uart3_rx_buffer[uart3_rx_extract_idx];		\
      uart3_rx_extract_idx = (uart3_rx_extract_idx + 1)%UART3_RX_BUFFER_SIZE; \
      ret;								\
    })

#endif /* USE_UART3 */


void uart_init( void );

#endif /* UART_HW_H */

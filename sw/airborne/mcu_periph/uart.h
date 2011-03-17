/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 *
 */

/** \file mcu_periph/uart.h
 *  \brief arch independant UART (Universal Asynchronous Receiver/Transmitter) API
 *
 */

#ifndef MCU_PERIPH_UART_H
#define MCU_PERIPH_UART_H

#include "mcu_periph/uart_arch.h"
#include "std.h"


#ifdef USE_UART0

extern void uart0_init( void );
extern void uart0_transmit( uint8_t data );
extern bool_t uart0_check_free_space( uint8_t len);

#define Uart0Init uart0_init
#define Uart0CheckFreeSpace(_x) uart0_check_free_space(_x)
#define Uart0Transmit(_x) uart0_transmit(_x)
#define Uart0SendMessage() {}

#define Uart0TxRunning uart0_tx_running
#define Uart0InitParam uart0_init_param

/* I want to trigger USE_UART and generate macros with the makefile same variable */
#define UART0Init           Uart0Init
#define UART0CheckFreeSpace Uart0CheckFreeSpace
#define UART0Transmit       Uart0Transmit
#define UART0SendMessage    Uart0SendMessage
#define UART0ChAvailable    Uart0ChAvailable
#define UART0Getch          Uart0Getch

#endif /* USE_UART0 */

#ifdef USE_UART1

extern void uart1_init( void );
extern void uart1_transmit( uint8_t data );
extern bool_t uart1_check_free_space( uint8_t len);

#define Uart1Init uart1_init
#define Uart1CheckFreeSpace(_x) uart1_check_free_space(_x)
#define Uart1Transmit(_x) uart1_transmit(_x)
#define Uart1SendMessage() {}

#define Uart1TxRunning uart1_tx_running
#define Uart1InitParam uart1_init_param

#define UART1Init           Uart1Init
#define UART1CheckFreeSpace Uart1CheckFreeSpace
#define UART1Transmit       Uart1Transmit
#define UART1SendMessage    Uart1SendMessage
#define UART1ChAvailable    Uart1ChAvailable
#define UART1Getch          Uart1Getch

#endif /* USE_UART1 */

#ifdef USE_UART2

extern void uart2_init( void );
extern void uart2_transmit( uint8_t data );
extern bool_t uart2_check_free_space( uint8_t len);

#define Uart2Init uart2_init
#define Uart2CheckFreeSpace(_x) uart2_check_free_space(_x)
#define Uart2Transmit(_x) uart2_transmit(_x)
#define Uart2SendMessage() {}

#define UART2Init           Uart2Init
#define UART2CheckFreeSpace Uart2CheckFreeSpace
#define UART2Transmit       Uart2Transmit
#define UART2SendMessage    Uart2SendMessage
#define UART2ChAvailable    Uart2ChAvailable
#define UART2Getch          Uart2Getch

#endif /* USE_UART2 */

#ifdef USE_UART3

extern void   uart3_init( void );
extern void   uart3_transmit( uint8_t data );
extern bool_t uart3_check_free_space( uint8_t len);

#define Uart3Init uart3_init
#define Uart3CheckFreeSpace(_x) uart3_check_free_space(_x)
#define Uart3Transmit(_x)       uart3_transmit(_x)
#define Uart3SendMessage() {}

#define UART3Init           Uart3Init
#define UART3CheckFreeSpace Uart3CheckFreeSpace
#define UART3Transmit       Uart3Transmit
#define UART3SendMessage    Uart3SendMessage
#define UART3ChAvailable    Uart3ChAvailable
#define UART3Getch          Uart3Getch

#endif /* USE_UART3 */

#ifdef USE_UART5

extern void   uart5_init( void );
extern void   uart5_transmit( uint8_t data );
extern bool_t uart5_check_free_space( uint8_t len);

#define Uart5Init uart5_init
#define Uart5CheckFreeSpace(_x) uart5_check_free_space(_x)
#define Uart5Transmit(_x)       uart5_transmit(_x)
#define Uart5SendMessage() {}

#define UART5Init           Uart5Init
#define UART5CheckFreeSpace Uart5CheckFreeSpace
#define UART5Transmit       Uart5Transmit
#define UART5SendMessage    Uart5SendMessage
#define UART5ChAvailable    Uart5ChAvailable
#define UART5Getch          Uart5Getch

#endif /* USE_UART5 */

#endif /* MCU_PERIPH_UART_H */

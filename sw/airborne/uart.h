/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

/** \file uart.h
 *  \brief arch independant UART (Universal Asynchronous Receiver/Transmitter) API
 *
 */

#ifndef UART_H
#define UART_H

#include "uart_hw.h"
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

#endif /* USE_UART1 */

#ifdef USE_UART2

extern void uart2_init( void );
extern void uart2_transmit( uint8_t data );
extern bool_t uart2_check_free_space( uint8_t len);

#define Uart2Init uart2_init
#define Uart2CheckFreeSpace(_x) uart2_check_free_space(_x)
#define Uart2Transmit(_x) uart2_transmit(_x)
#define Uart2SendMessage() {}

#endif /* USE_UART2 */

#ifdef USE_UART3

extern void   uart3_init( void );
extern void   uart3_transmit( uint8_t data );
extern bool_t uart3_check_free_space( uint8_t len);

#define Uart3Init uart3_init
#define Uart3CheckFreeSpace(_x) uart3_check_free_space(_x)
#define Uart3Transmit(_x)       uart3_transmit(_x)
#define Uart3SendMessage() {}

#endif /* USE_UART3 */

#endif /* UART_H */

/*
 * $Id$
 *  
 * Copyright (C) 2009  Pascal Brisset, Antoine Drouin
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

/*
 *\brief STM32 usart functions 
 *
 */

#ifndef UART_HW_H
#define UART_HW_H

#include "std.h"

#define B38400 38400
#define B57600 57600

#ifdef USE_UART3
#define UART3_RX_BUFFER_SIZE 128
#define UART3_TX_BUFFER_SIZE 128

extern uint16_t uart3_rx_insert_idx, uart3_rx_extract_idx;
extern uint8_t  uart3_rx_buffer[UART3_RX_BUFFER_SIZE];
extern uint8_t  uart3_tx_buffer[UART3_TX_BUFFER_SIZE];
extern volatile uint16_t uart3_tx_insert_idx, uart3_tx_extract_idx;
extern volatile bool_t   uart3_tx_running;

extern void usart3_irq_handler(void);
#endif /* USE_UART3 */

#endif /* UART_HW_H */

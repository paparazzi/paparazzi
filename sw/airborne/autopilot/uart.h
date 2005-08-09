/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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
#ifndef _UART_H_
#define _UART_H_

#include <inttypes.h>

extern void uart0_init(void);
extern void uart1_init(void);

extern void uart0_print_string(const uint8_t*);
extern void uart0_print_hex(const uint8_t);
extern void uart0_transmit(const uint8_t);
extern void uart1_transmit(const uint8_t);

#define ReceiveUart0(cb) \
  SIGNAL( SIG_UART0_RECV ) { \
    uint8_t c = UDR0; \
    cb(c); \
}
#define ReceiveUart1(cb) \
  SIGNAL( SIG_UART1_RECV ) { \
    uint8_t c = UDR1; \
    cb(c); \
}

#endif

/*
 * $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file quad_ins.h
 *  \brief Device independent INS code
 *
*/


#ifndef QUAD_INS_H
#define QUAD_INS_H

#include "std.h"

extern volatile uint8_t ins_msg_received;

extern void quad_ins_init( void );
extern void quad_ins_periodic_task( void );
void parse_ins_msg( void );
void parse_ins_buffer( uint8_t );

#ifndef SITL
#include "uart.h"

#define __InsLink(dev, _x) dev##_x
#define _InsLink(dev, _x)  __InsLink(dev, _x)
#define InsLink(_x) _InsLink(INS_LINK, _x)

#define InsBuffer() InsLink(ChAvailable())
#define ReadInsBuffer() { while (InsBuffer()&&!ins_msg_received) parse_ins_buffer(InsLink(Getch())); }
#define InsUartSend1(c) InsLink(Transmit(c))
#define InsUartInitParam(_a,_b,_c) InsLink(InitParam(_a,_b,_c))
#define InsUartRunning InsLink(TxRunning)

#endif /** !SITL */

#define QuadInsEventCheckAndHandle(handler) { \
  if (InsBuffer()) {				  \
    ReadInsBuffer();				  \
  }						                \
  if (ins_msg_received) {			\
    parse_ins_msg();          \
    handler();                \
    ins_msg_received = FALSE; \
  }						                \
}


#endif /* QUAD_INS_H */

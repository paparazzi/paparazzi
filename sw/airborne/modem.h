/*
 * $Id$
 *
 * Paparazzi mcu0 cmx469 modem functions
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

#ifndef MODEM_H
#define MODEM_H

#include "inttypes.h"

extern uint8_t modem_nb_ovrn;

#ifdef MODEM

#include "airframe.h"
#include "modem_hw.h"


#define TX_BUF_SIZE     255
extern uint8_t           tx_head;
extern volatile uint8_t  tx_tail;
extern uint8_t           tx_buf[ TX_BUF_SIZE ];

extern uint8_t    tx_byte;
extern uint8_t    tx_byte_idx;

extern uint8_t ck_a, ck_b;


/** 5 = STX + ac_id + msg_id + ck_a + ck_b */
#define ModemSizeOf(_payload) (_payload+5)

#define ModemStartMessage(id) \
  { ModemPut1Byte(STX); ModemPut1Byte(id); ck_a = id; ck_b = id; ModemPut1ByteUpdateCs(AC_ID);}

#define ModemEndMessage() \
  { ModemPut1Byte(ck_a); ModemPut1Byte(ck_b); MODEM_CHECK_RUNNING(); }

#if TX_BUF_SIZE == 256
#define UPDATE_HEAD() {			   \
    tx_head++;				   \
}
#else
#define UPDATE_HEAD() {			   \
  tx_head++;				   \
  if (tx_head >= TX_BUF_SIZE) tx_head = 0; \
}
#endif

#define ModemCheckFreeSpace(_space) (tx_head>=tx_tail? _space < (TX_BUF_SIZE - (tx_head - tx_tail)) : _space < (tx_tail - tx_head))

#define ModemPut1Byte(_byte) { \
  tx_buf[tx_head] = _byte;	  \
  UPDATE_HEAD();		  \
}

#define ModemPut1ByteUpdateCs(_byte) { \
  ck_a += _byte;			  \
  ck_b += ck_a;			  \
  ModemPut1Byte(_byte) \
}

#define ModemPut1ByteByAddr(_byte) { \
  uint8_t _x = *(_byte);		  \
  ModemPut1ByteUpdateCs(_x); \
}

#define ModemPut2ByteByAddr(_byte) { \
  ModemPut1ByteByAddr(_byte); \
  ModemPut1ByteByAddr(_byte+1); \
}

#define ModemPut4ByteByAddr(_byte) { \
  ModemPut2ByteByAddr(_byte); \
  ModemPut2ByteByAddr(_byte+2); \
}

#define MODEM_LOAD_NEXT_BYTE() { \
  tx_byte = tx_buf[tx_tail]; \
  tx_byte_idx = 0; \
  tx_tail++; \
  if( tx_tail >= TX_BUF_SIZE ) \
    tx_tail = 0; \
}

#define ModemTransmit(_x) ModemPut1Byte(_x)

#endif // MODEM

#endif /* MODEM_H */

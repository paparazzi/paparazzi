/*
 * Paparazzi $Id$
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

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <inttypes.h>

extern uint8_t ck_a, ck_b;
extern uint8_t downlink_nb_ovrn;
#define STX  0x99

#define __DownlinkCheckFreeSpace(dev, _x) dev ##_check_free_space(_x)
#define _DownlinkCheckFreeSpace(dev, _x)  __DownlinkCheckFreeSpace(dev, _x)
#define DownlinkCheckFreeSpace(_x) _DownlinkCheckFreeSpace(DOWNLINK_DEVICE, _x)

#define __DownlinkPut1Byte(dev, _x) dev ##_transmit(_x)
#define _DownlinkPut1Byte(dev, _x)  __DownlinkPut1Byte(dev, _x)
#define DownlinkPut1Byte(_x) _DownlinkPut1Byte(DOWNLINK_DEVICE, _x)

/** 5 = STX + len + ac_id + msg_id + ck_a + ck_b */
#define DownlinkSizeOf(_payload) (_payload+6)

#define DownlinkPut1ByteUpdateCs(_byte) { \
    ck_a += _byte;			  \
    ck_b += ck_a;			  \
    DownlinkPut1Byte(_byte);		  \
  }

#define DownlinkPut1ByteByAddr(_byte) {	 \
    uint8_t _x = *(_byte);		 \
    DownlinkPut1ByteUpdateCs(_x);	 \
  }

#define DownlinkPut2ByteByAddr(_byte) { \
    DownlinkPut1ByteByAddr(_byte);	\
    DownlinkPut1ByteByAddr(_byte+1);	\
  }

#define DownlinkPut4ByteByAddr(_byte) { \
    DownlinkPut2ByteByAddr(_byte);	\
    DownlinkPut2ByteByAddr(_byte+2);	\
  }

#define DownlinkStartMessage(msg_id, payload_len) {	\
    DownlinkPut1Byte(STX);				\
    uint8_t msg_len = DownlinkSizeOf(payload_len);	\
    DownlinkPut1Byte(msg_len);				\
    ck_a = msg_len; ck_b = msg_len;			\
    DownlinkPut1ByteUpdateCs(AC_ID);			\
    DownlinkPut1ByteUpdateCs(msg_id);			\
  }

#define DownlinkEndMessage() {			\
    DownlinkPut1Byte(ck_a);			\
    DownlinkPut1Byte(ck_b);			\
  }

#endif /* DOWNLINK_H */

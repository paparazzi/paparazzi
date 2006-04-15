/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2006- Pascal Brisset, Antoine Drouin
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

#ifndef FBW_DOWNLINK_H
#define FBW_DOWNLINK_H

#include "inttypes.h"
#include "messages_fbw.h"
#include "airframe.h"

#include "uart.h"
#include "main_fbw.h"
#include "radio_control.h"

extern uint8_t ck_a, ck_b;
extern uint8_t downlink_nb_ovrn;
#define STX  0x05

#define __DownlinkCheckFreeSpace(dev, _x) dev ##_check_free_space(_x)
#define _DownlinkCheckFreeSpace(dev, _x)  __DownlinkCheckFreeSpace(dev, _x)
#define DownlinkCheckFreeSpace(_x) _DownlinkCheckFreeSpace(DOWNLINK_FBW_DEVICE, _x)

#define __DownlinkPut1Byte(dev, _x) dev ##_transmit(_x)
#define _DownlinkPut1Byte(dev, _x)  __DownlinkPut1Byte(dev, _x)
#define DownlinkPut1Byte(_x) _DownlinkPut1Byte(DOWNLINK_FBW_DEVICE, _x)

#define PERIODIC_SEND_PPM() {}
//#define PERIODIC_SEND_SERVOS() { Uart0PrintString("SERVOS\n");}
#define PERIODIC_SEND_SERVOS() {}
#define PERIODIC_SEND_FBW_STATUS() {DOWNLINK_SEND_FBW_STATUS(&fbw_mode, &rc_status, &fbw_mode)}
#define PERIODIC_SEND_RC() {}


/** 5 = STX + ac_id + msg_id + ck_a + ck_b */
#define DownlinkSizeOf(_payload) (_payload+5)

#define DownlinkPut1ByteUpdateCs(_byte) { \
  ck_a += _byte;			  \
  ck_b += ck_a;				  \
  DownlinkPut1Byte(_byte);		  \
}

#define DownlinkPut1ByteByAddr(_byte) {	  \
  uint8_t _x = *(_byte);		  \
  DownlinkPut1ByteUpdateCs(_x);		  \
}

#define DownlinkStartMessage(id)					\
  { DownlinkPut1Byte(STX); DownlinkPut1Byte(id); ck_a = id; ck_b = id; DownlinkPut1ByteUpdateCs(AC_ID);}

#define DownlinkEndMessage()				\
  { DownlinkPut1Byte(ck_a); DownlinkPut1Byte(ck_b); }


static inline void fbw_downlink_periodic_task(void) {
  PeriodicSend()
    }


#endif /* FBW_DOWNLINK_H */

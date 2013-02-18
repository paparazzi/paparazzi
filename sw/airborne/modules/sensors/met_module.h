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

/** \file met_module.h
 *  \brief Device independent serial meteo code
 *
*/


#ifndef MET_H
#define MET_H

#include "std.h"
#include "led.h"

extern volatile uint8_t ins_msg_received;
extern volatile uint8_t new_ins_attitude;

#ifndef SITL
#include "mcu_periph/uart.h"

#define __MetLink(dev, _x) dev##_x
#define _MetLink(dev, _x)  __MetLink(dev, _x)
#define MetLink(_x) _MetLink(MET_LINK, _x)

#define MetBuffer() MetLink(ChAvailable())
#define ReadMetBuffer() { while (MetLink(ChAvailable())&&!met_msg_received) parse_met_buffer(MetLink(Getch())); }
#define MetSend1(c) MetLink(Transmit(c))
#define MetUartSend1(c) MetSend1(c)
#define MetSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) MetSend1(_dat[i]); };
#define MetUartSetBaudrate(_b) MetLink(SetBaudrate(_b))
#define MetUartRunning MetLink(TxRunning)

#endif /** !SITL */

#define InsEventCheckAndHandle(handler) {			\
    if (InsBuffer()) {						\
      ReadInsBuffer();						\
    }						                \
    if (ins_msg_received) {					\
      LED_TOGGLE(2);						\
      parse_ins_msg();						\
      handler;							\
      ins_msg_received = FALSE;					\
    }						                \
  }


#endif /* MET_H */

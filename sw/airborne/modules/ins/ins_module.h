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

/** \file ins.h
 *  \brief Device independent INS code
 *
*/


#ifndef INS_H
#define INS_H

#include "std.h"
#include "led.h"

#ifndef INS_FORMAT
#define INS_FORMAT float
#endif

extern INS_FORMAT ins_x;
extern INS_FORMAT ins_y;
extern INS_FORMAT ins_z;

extern INS_FORMAT ins_vx;
extern INS_FORMAT ins_vy;
extern INS_FORMAT ins_vz;

extern INS_FORMAT ins_phi;
extern INS_FORMAT ins_theta;
extern INS_FORMAT ins_psi;

extern INS_FORMAT ins_p;
extern INS_FORMAT ins_q;
extern INS_FORMAT ins_r;

extern INS_FORMAT ins_ax;
extern INS_FORMAT ins_ay;
extern INS_FORMAT ins_az;

extern INS_FORMAT ins_mx;
extern INS_FORMAT ins_my;
extern INS_FORMAT ins_mz;

extern INS_FORMAT ins_roll_neutral;
extern INS_FORMAT ins_pitch_neutral;


extern volatile uint8_t ins_msg_received;
extern volatile uint8_t new_ins_attitude;

extern void ins_init( void );
extern void ins_periodic_task( void );
void handle_ins_msg( void);
void parse_ins_msg( void );
void parse_ins_buffer( uint8_t );

#ifndef SITL
#include "mcu_periph/uart.h"

#define __InsLink(dev, _x) dev##_x
#define _InsLink(dev, _x)  __InsLink(dev, _x)
#define InsLink(_x) _InsLink(INS_LINK, _x)

#define InsBuffer() InsLink(ChAvailable())
#define ReadInsBuffer() { while (InsLink(ChAvailable())&&!ins_msg_received) parse_ins_buffer(InsLink(Getch())); }
#define InsSend1(c) InsLink(Transmit(c))
#define InsUartSend1(c) InsSend1(c)
#define InsSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) InsSend1(_dat[i]); };
#define InsUartSetBaudrate(_b) InsLink(SetBaudrate(_b))
#define InsUartRunning InsLink(TxRunning)

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


#endif /* INS_H */

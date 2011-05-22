/*
 * $Id: demo_module.h 3079 2009-03-11 16:55:42Z gautier $
 *
 * Copyright (C) 2010  Gautier Hattenberger
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

/** \file cam_track.h
 *
 * blob tracking with cmucam
 */

#ifndef CAM_TRACK_H
#define CAM_TRACK_H

#include <inttypes.h>
#include "math/pprz_algebra_float.h"

extern struct FloatVect3 target_pos_ned;
extern struct FloatVect3 target_speed_ned;
extern struct FloatVect3 target_accel_ned;

extern void track_init(void);
extern void track_periodic_task(void);
extern void track_event(void);

extern volatile uint8_t cam_msg_received;
extern void parse_cam_msg( void );
extern void parse_cam_buffer( uint8_t );

#include "mcu_periph/uart.h"

#define __CamLink(dev, _x) dev##_x
#define _CamLink(dev, _x)  __CamLink(dev, _x)
#define CamLink(_x) _CamLink(CAM_LINK, _x)

#define CamBuffer() CamLink(ChAvailable())
#define ReadCamBuffer() { while (CamLink(ChAvailable())&&!cam_msg_received) parse_cam_buffer(CamLink(Getch())); }
#define CamUartSend1(c) CamLink(Transmit(c))
#define CamUartSetBaudrate(_b) CamLink(SetBaudrate(_b))
#define CamUartRunning CamLink(TxRunning)

#define CamEventCheckAndHandle() { \
  if (CamBuffer()) {				  \
    ReadCamBuffer();				  \
  }						                \
  if (cam_msg_received) {			\
    parse_cam_msg();          \
    track_event();            \
    cam_msg_received = FALSE; \
  }						                \
}


#endif

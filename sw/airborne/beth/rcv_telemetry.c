/*
 * $Id: datalink.c 5626 2010-08-23 18:10:19Z flixr $
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file rcv_telemetry.c
 *  \brief Handling of messages coming from other A/Cs
 *
 */
#define DATALINK_C

#define MODULES_DATALINK_C

#include <inttypes.h>
#include <string.h>
#include "datalink.h"

#include "modules.h"

#include <stdio.h>
#include "common_nav.h"
#include "settings.h"
#include "latlong.h"


#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "uart.h"
#include "downlink.h"


#define MOfCm(_x) (((float)(_x))/100.)

#define SenderIdOfMsg(x) (x[0])
#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {
  datalink_time = 0;
  uint8_t msg_id = IdOfMsg(dl_buffer);
  printf("Tiny rx id: %d\n",msg_id);

if (msg_id == DL_ATTITUDE) {
      float phi = DL_ATTITUDE_phi(dl_buffer);
      float psi = DL_ATTITUDE_psi(dl_buffer);
      float theta = DL_ATTITUDE_theta(dl_buffer);
      printf("Attitude: %f %f %f\n", phi, psi, theta);
  } else {
    printf("Unknown msg id %d",msg_id);
  }

}

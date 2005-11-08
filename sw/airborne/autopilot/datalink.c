/*
 * $Id$
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
/** \file datalink.c
 *  \brief Hadling of messages coming from ground and other A/Cs
 *
 */
#define DATALINK_C

#include <inttypes.h>
#include "dl_protocol.h"
#include "traffic_info.h"
#include "nav.h"
#include "datalink.h"
#include "flight_plan.h"
#include "autopilot.h"

/***/
#include "uart.h"

#define MOfCm(_x) (((float)_x)/100.)


void dl_parse_msg(void) {
  uint8_t msg_id = dl_buffer[0];
  if (msg_id == DL_ACINFO_ID) {
    uint8_t id = DL_ACINFO_ac_id(dl_buffer);
    float ux = MOfCm(DL_ACINFO_utm_east(dl_buffer));
    float uy = MOfCm(DL_ACINFO_utm_north(dl_buffer));
    float a = MOfCm(DL_ACINFO_alt(dl_buffer));
    float c = RadOfDeg(((float)DL_ACINFO_course(dl_buffer))/ 10.);
    float s = MOfCm(DL_ACINFO_speed(dl_buffer));
    SetAcInfo(id, ux, uy, c, a, s);
  } else if (msg_id == DL_MOVE_WP_ID) {
    uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
    float ux = MOfCm(DL_MOVE_WP_utm_east(dl_buffer));
    float uy = MOfCm(DL_MOVE_WP_utm_north(dl_buffer));
    float a = MOfCm(DL_MOVE_WP_alt(dl_buffer));
    MoveWaypoint(wp_id, ux, uy, a);
  } else if (msg_id == DL_EVENT_ID) {
    uint8_t event = DL_EVENT_event(dl_buffer);
    switch (event) {
    case 1 : rc_event_1 = TRUE; break; // FIXME !
    case 2 : rc_event_2 = TRUE; break;
    default: ;
    }
  }
}

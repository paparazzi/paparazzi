/*
 * Copyright (C) 2014 Freek van Tienen
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

/** @file follow.c
 *  @brief Follow a certain AC ID.
 * Only for rotorcraft firmware.
 */

#include "multi/follow.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "subsystems/navigation/waypoints.h"

#include "state.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"

#ifndef FOLLOW_OFFSET_X
#define FOLLOW_OFFSET_X 0.0
#endif

#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0
#endif

#ifndef FOLLOW_OFFSET_Z
#define FOLLOW_OFFSET_Z 0.0
#endif

void follow_init(void)
{

}

void follow_change_wp(unsigned char *buffer)
{
  struct EcefCoor_i new_pos;
  struct EnuCoor_i enu;
  new_pos.x = DL_REMOTE_GPS_ecef_x(buffer);
  new_pos.y = DL_REMOTE_GPS_ecef_y(buffer);
  new_pos.z = DL_REMOTE_GPS_ecef_z(buffer);

  // Translate to ENU
  enu_of_ecef_point_i(&enu, &state.ned_origin_i, &new_pos);
  INT32_VECT3_SCALE_2(enu, enu, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);

  // TODO: Add the angle to the north

  // Update the offsets
  enu.x += POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  enu.y += POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  enu.z += POS_BFP_OF_REAL(FOLLOW_OFFSET_Z);

  // TODO: Remove the angle to the north

  // Move the waypoint
  waypoint_set_enu_i(FOLLOW_WAYPOINT_ID, &enu);
}

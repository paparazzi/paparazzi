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

#ifndef FOLLOW_OFFSET_X
#define FOLLOW_OFFSET_X 0.0
#endif

#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0
#endif

#ifndef FOLLOW_OFFSET_Z
#define FOLLOW_OFFSET_Z 0.0
#endif

#ifndef FOLLOW_AC_ID
#error "Please define FOLLOW_AC_ID"
#endif

#ifndef FOLLOW_WAYPOINT_ID
#error "Please define FOLLOW_WAYPOINT_ID"
#endif

void follow_init(void){}

/*
 * follow_wp(void)
 * updates the FOLLOW_WAYPOINT_ID to a fixed offset from the last received location
 * of other aircraft with id FOLLOW_AC_ID
 */
void follow_wp(void)
{
  struct ac_info_ * ac = get_ac_info(FOLLOW_AC_ID);

  struct UtmCoor_f my_pos = utm_float_from_gps(&gps, 0);
  my_pos.alt = gps.hmsl / 1000.;

  struct EnuCoor_i enu = stateGetPositionEnu_i();
  enu.x += POS_BFP_OF_REAL(my_pos.east - ac->utm.east);
  enu.y += POS_BFP_OF_REAL(my_pos.north - ac->utm.north);
  enu.z += POS_BFP_OF_REAL(my_pos.alt - ac->utm.alt);

  // TODO: Add the angle to the north

  // Update the offsets
  enu.x += POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  enu.y += POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  enu.z += POS_BFP_OF_REAL(FOLLOW_OFFSET_Z);

  // TODO: Remove the angle to the north

  // Move the waypoint
  waypoint_set_enu_i(FOLLOW_WAYPOINT_ID, &enu);
}

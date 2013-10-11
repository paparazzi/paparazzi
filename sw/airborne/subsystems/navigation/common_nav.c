/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
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
 */

/**
 * @file subsystems/navigation/common_nav.c
 *
 */

#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#include "subsystems/ins.h"
#include "subsystems/gps.h"
#include "math/pprz_geodetic_float.h"

float dist2_to_home;
float dist2_to_wp;

bool_t too_far_from_home;

const uint8_t nb_waypoint = NB_WAYPOINT;
struct point waypoints[NB_WAYPOINT] = WAYPOINTS;

float ground_alt;

int32_t nav_utm_east0 = NAV_UTM_EAST0;
int32_t nav_utm_north0 = NAV_UTM_NORTH0;
uint8_t nav_utm_zone0 = NAV_UTM_ZONE0;
float max_dist_from_home = MAX_DIST_FROM_HOME;

/** \brief Computes square distance to the HOME waypoint potentially sets
 * \a too_far_from_home
 */
void compute_dist2_to_home(void) {
  struct EnuCoor_f* pos = stateGetPositionEnu_f();
  float ph_x = waypoints[WP_HOME].x - pos->x;
  float ph_y = waypoints[WP_HOME].y - pos->y;
  dist2_to_home = ph_x*ph_x + ph_y *ph_y;
  too_far_from_home = dist2_to_home > (MAX_DIST_FROM_HOME*MAX_DIST_FROM_HOME);
#if defined InAirspace
  too_far_from_home = too_far_from_home || !(InAirspace(pos_x, pos_y));
#endif
}


static float previous_ground_alt;

/** Reset the UTM zone to current GPS fix */
unit_t nav_reset_utm_zone(void) {

  struct UtmCoor_f utm0_old;
  utm0_old.zone = nav_utm_zone0;
  utm0_old.north = nav_utm_north0;
  utm0_old.east = nav_utm_east0;
  utm0_old.alt = ground_alt;
  struct LlaCoor_f lla0;
  lla_of_utm_f(&lla0, &utm0_old);

#ifdef GPS_USE_LATLONG
  /* Set the real UTM zone */
  nav_utm_zone0 = (DegOfRad(gps.lla_pos.lon/1e7)+180) / 6 + 1;
#else
  nav_utm_zone0 = gps.utm_pos.zone;
#endif

  struct UtmCoor_f utm0;
  utm0.zone = nav_utm_zone0;
  utm_of_lla_f(&utm0, &lla0);

  nav_utm_east0 = utm0.east;
  nav_utm_north0 = utm0.north;

  stateSetLocalUtmOrigin_f(&utm0);

  return 0;
}

/** Reset the geographic reference to the current GPS fix */
unit_t nav_reset_reference( void ) {
#ifdef GPS_USE_LATLONG
  /* Set the real UTM zone */
  nav_utm_zone0 = (DegOfRad(gps.lla_pos.lon/1e7)+180) / 6 + 1;

  /* Recompute UTM coordinates in this zone */
  struct LlaCoor_f lla;
  lla.lat = gps.lla_pos.lat/1e7;
  lla.lon = gps.lla_pos.lon/1e7;
  struct UtmCoor_f utm;
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, &lla);
  nav_utm_east0 = utm.east;
  nav_utm_north0 = utm.north;
#else
  nav_utm_zone0 = gps.utm_pos.zone;
  nav_utm_east0 = gps.utm_pos.east/100;
  nav_utm_north0 = gps.utm_pos.north/100;
#endif

  // reset state UTM ref
  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);

  // realign INS if needed
  ins.hf_realign = TRUE;
  ins.vf_realign = TRUE;

  previous_ground_alt = ground_alt;
  ground_alt = gps.hmsl/1000.;
  return 0;
}

/** Shift altitude of the waypoint according to a new ground altitude */
unit_t nav_update_waypoints_alt( void ) {
  uint8_t i;
  for(i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].a += ground_alt - previous_ground_alt;
  }
  return 0;
}

void common_nav_periodic_task_4Hz() {
  RunOnceEvery(4, { stage_time++;  block_time++; });
}

void nav_move_waypoint(uint8_t wp_id, float ux, float uy, float alt) {
  if (wp_id < nb_waypoint) {
    float dx, dy;
    dx = ux - nav_utm_east0 - waypoints[WP_HOME].x;
    dy = uy - nav_utm_north0 - waypoints[WP_HOME].y;
    BoundAbs(dx, max_dist_from_home);
    BoundAbs(dy, max_dist_from_home);
    waypoints[wp_id].x = waypoints[WP_HOME].x + dx;
    waypoints[wp_id].y = waypoints[WP_HOME].y + dy;
    waypoints[wp_id].a = alt;
  }
}

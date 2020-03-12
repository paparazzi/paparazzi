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
#include "math/pprz_geodetic_float.h"

float dist2_to_home;
float dist2_to_wp;

bool too_far_from_home;

const uint8_t nb_waypoint = NB_WAYPOINT;
struct point waypoints[NB_WAYPOINT] = WAYPOINTS_UTM;

float ground_alt;

int32_t nav_utm_east0 = NAV_UTM_EAST0;
int32_t nav_utm_north0 = NAV_UTM_NORTH0;
uint8_t nav_utm_zone0 = NAV_UTM_ZONE0;
float max_dist_from_home = MAX_DIST_FROM_HOME;

/** Computes squared distance to the HOME waypoint.
 * Updates #dist2_to_home and potentially sets #too_far_from_home
 */
void compute_dist2_to_home(void)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  float ph_x = waypoints[WP_HOME].x - pos->x;
  float ph_y = waypoints[WP_HOME].y - pos->y;
  dist2_to_home = ph_x * ph_x + ph_y * ph_y;
  too_far_from_home = dist2_to_home > (MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME);
#ifdef InGeofenceSector
  too_far_from_home = too_far_from_home || !(InGeofenceSector(pos->x, pos->y));
#endif
}

/** Compute time to home
 * use wind and airspeed when available
 */
float get_time_to_home(void)
{
  struct FloatVect2 vect_to_home;
  vect_to_home.x = waypoints[WP_HOME].x - stateGetPositionEnu_f()->x;
  vect_to_home.y = waypoints[WP_HOME].y - stateGetPositionEnu_f()->y;
  // get distance to home
  float dist_to_home = float_vect2_norm(&vect_to_home);
  if (dist_to_home > 1.f) {
    // get windspeed or assume no wind
    struct FloatVect2 wind = { 0.f, 0.f };
    if (stateIsWindspeedValid()) {
      wind = *stateGetHorizontalWindspeed_f();
    }
    // compute effective windspeed when flying to home point
    float wind_to_home = (wind.x * vect_to_home.x + wind.y * vect_to_home.y) / dist_to_home;
    // get airspeed or assume constant nominal airspeed
    float airspeed = NOMINAL_AIRSPEED;
    if (stateIsAirspeedValid()) {
      airspeed = stateGetAirspeed_f();
    }
    // get estimated ground speed to home
    float gspeed_to_home = wind_to_home + airspeed;
    if (gspeed_to_home > 1.) {
      return dist_to_home / gspeed_to_home; // estimated time to home in seconds
    }
    else {
      return 999999.f; // this might take a long time to go back home
    }
  }
  return 0.f; // too close to home point
}


static float previous_ground_alt;

/** Reset the UTM zone to current GPS fix */
void nav_reset_utm_zone(void)
{

  struct UtmCoor_f utm0;
  utm0.zone = nav_utm_zone0;
  utm0.north = nav_utm_north0;
  utm0.east = nav_utm_east0;
  utm0.alt = ground_alt;
  ins_reset_utm_zone(&utm0);

  /* Set the real UTM ref */
  nav_utm_zone0 = utm0.zone;
  nav_utm_east0 = utm0.east;
  nav_utm_north0 = utm0.north;
}

/** Reset the geographic reference to the current GPS fix */
void nav_reset_reference(void)
{
  /* realign INS */
  ins_reset_local_origin();

  /* Set nav UTM ref */
  nav_utm_east0 = state.utm_origin_f.east;
  nav_utm_north0 = state.utm_origin_f.north;
  nav_utm_zone0 = state.utm_origin_f.zone;

  /* Ground alt */
  previous_ground_alt = ground_alt;
  ground_alt = state.utm_origin_f.alt;
}

/** Reset the altitude reference to the current GPS alt */
void nav_reset_alt(void)
{
  ins_reset_altitude_ref();

  /* Ground alt */
  previous_ground_alt = ground_alt;
  ground_alt = state.utm_origin_f.alt;
}

/** Shift altitude of the waypoint according to a new ground altitude */
void nav_update_waypoints_alt(void)
{
  uint8_t i;
  for (i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].a += ground_alt - previous_ground_alt;
  }
}

void common_nav_periodic_task_4Hz()
{
  RunOnceEvery(4, { stage_time++;  block_time++; });
}

/** Move a waypoint to given UTM coordinates.
 * @param[in] wp_id Waypoint ID
 * @param[in] ux    UTM x (east) coordinate
 * @param[in] uy    UTM y (north) coordinate
 * @param[in] alt   Altitude above MSL.
 */
void nav_move_waypoint(uint8_t wp_id, float ux, float uy, float alt)
{
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

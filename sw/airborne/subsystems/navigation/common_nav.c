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

float ground_alt = NAV_MSL0;
static float previous_ground_alt = NAV_MSL0;

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

/** Reset the UTM zone to current GPS fix
 */
bool nav_reset_utm_zone(uint8_t zone)
{
  struct UtmCoor_f previous_origin;
  UTM_COPY(previous_origin, state.utm_origin_f);

  ins_reset_utm_zone(zone);

  /* zone extend waypoints if nessesary */
  if (state.utm_origin_f.zone != previous_origin.zone)
  {
    nav_zone_extend_waypoints(&previous_origin, state.utm_origin_f.zone);
  }

  return 0;
}

/** Reset the geographic reference to the current GPS fix */
bool nav_reset_reference(void)
{
  struct UtmCoor_f previous_origin;
  UTM_COPY(previous_origin, state.utm_origin_f);

  /* realign INS */
  ins_reset_local_origin();

  /* zone extend waypoints if nessesary */
  if (state.utm_origin_f.zone != previous_origin.zone)
  {
    nav_zone_extend_waypoints(&previous_origin, state.utm_origin_f.zone);
  }

  /* Ground alt */
  previous_ground_alt = ground_alt;
  struct LlaCoor_f lla;
  lla_of_utm(&lla, state.utm_origin_f);
  ground_alt = wgs84_ellipsoid_to_geoid_f(lla.lat, lla.lon);

  return TRUE;
}

/** Reset the altitude reference to the current GPS alt */
bool nav_reset_alt(void)
{
  ins_reset_altitude_ref();

  /* Ground alt */
  previous_ground_alt = ground_alt;
  struct LlaCoor_f lla;
  lla_of_utm(&lla, state.utm_origin_f);
  ground_alt = wgs84_ellipsoid_to_geoid_f(lla.lat, lla.lon);

  return TRUE;
}

/** Shift relative position of the waypoint according to a new zone
 * global positions are not updated
 */
bool nav_zone_extend_waypoints(struct UtmCoor_f *prev_origin_utm, uint8_t zone)
{
  /* recompute locaiton of home waypoint in new zone */
  struct LlaCoor_f prev_origin_lla;
  lla_of_utm_f(&prev_origin_lla, prev_origin_utm);

  struct UtmCoor_f new_origin_utm;
  new_origin_utm.zone = zone;
  utm_of_lla_f(&new_origin_utm, &prev_origin_lla);

  struct EnuCoor_f origin_diff;
  ENU_OF_UTM_DIFF(origin_diff, new_origin_utm, *prev_origin_utm);

  uint8_t i;
  for (i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].x += origin_diff.x;
    waypoints[i].y += origin_diff.y;
  }
  return TRUE;
}

/** Shift altitude of the waypoint according to a new ground altitude */
bool nav_update_waypoints_alt(void)
{
  uint8_t i;
  for (i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].a += ground_alt - previous_ground_alt;
  }
  return TRUE;
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
    dx = ux - state.utm_origin_f.east - waypoints[WP_HOME].x;
    dy = uy - state.utm_origin_f.north - waypoints[WP_HOME].y;
    BoundAbs(dx, max_dist_from_home);
    BoundAbs(dy, max_dist_from_home);
    waypoints[wp_id].x = waypoints[WP_HOME].x + dx;
    waypoints[wp_id].y = waypoints[WP_HOME].y + dy;
    waypoints[wp_id].a = alt;
  }
}

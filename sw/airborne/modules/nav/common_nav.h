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
 * @file modules/nav/common_nav.h
 *
 */

#ifndef COMMON_NAV_H
#define COMMON_NAV_H

#include "std.h"
#include "state.h"
#include "modules/nav/common_flight_plan.h"

extern float max_dist_from_home;
extern float dist2_to_home;
extern float dist2_to_wp;
extern bool too_far_from_home;

struct point {
  float x;
  float y;
  float a;
};

#define WaypointX(_wp) (waypoints[_wp].x)
#define WaypointY(_wp) (waypoints[_wp].y)
/** waypoint altitude in m above MSL */
#define WaypointAlt(_wp) (waypoints[_wp].a)
#define Height(_h) (_h + ground_alt)

extern void nav_move_waypoint(uint8_t wp_id, float utm_east, float utm_north, float alt);
extern void nav_move_waypoint_enu(uint8_t wp_id, float x, float y, float alt);
extern void nav_move_waypoint_point(uint8_t wp_id, struct point *p);
extern void nav_send_waypoint(uint8_t wp_id);

extern const uint8_t nb_waypoint;
extern struct point waypoints[];
/** size == nb_waypoint, waypoint 0 is a dummy waypoint */

/** altitude of the ground in m above MSL */
extern float ground_alt;

extern int32_t nav_utm_east0;  /* m */
extern int32_t nav_utm_north0; /* m */
extern uint8_t nav_utm_zone0;


extern void compute_dist2_to_home(void);
extern void nav_reset_utm_zone(void);
extern void nav_reset_reference(void) __attribute__((unused));
extern void nav_reset_alt(void) __attribute__((unused));
extern void nav_update_waypoints_alt(void) __attribute__((unused));
extern void common_nav_periodic_task(void);
extern float get_time_to_home(void); /* estimated time to home point in seconds */


#define NavSetGroundReferenceHere() ({ nav_reset_reference(); nav_update_waypoints_alt(); false; })

#define NavSetAltitudeReferenceHere() ({ nav_reset_alt(); nav_update_waypoints_alt(); false; })

#define NavSetWaypointHere(_wp) ({ \
    waypoints[_wp].x = stateGetPositionEnu_f()->x; \
    waypoints[_wp].y = stateGetPositionEnu_f()->y; \
    false; \
  })

#define NavSetWaypointHereSend(_wp) ({ \
    waypoints[_wp].x = stateGetPositionEnu_f()->x; \
    waypoints[_wp].y = stateGetPositionEnu_f()->y; \
    RunOnceEvery(NAVIGATION_FREQUENCY/1, {nav_send_waypoint(_wp);}) \
    false; \
  })

#define NavSetWaypointPosAndAltHere(_wp) ({ \
    waypoints[_wp].x = stateGetPositionEnu_f()->x; \
    waypoints[_wp].y = stateGetPositionEnu_f()->y; \
    waypoints[_wp].a = stateGetPositionEnu_f()->z + ground_alt; \
    false; \
  })

#define NavSetWaypointDistBehind(_wp, _ref, dist) ({ \
  waypoints[_wp].x = waypoints[_ref].x - dist*sin(stateGetNedToBodyEulers_f()->psi); \
  waypoints[_wp].y = waypoints[_ref].y - dist*cos(stateGetNedToBodyEulers_f()->psi); \
  false; \
})

#endif /* COMMON_NAV_H */

/*
 * Copyright (C) 2003-2009  ENAC, Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/nav.h
 *
 * Fixedwing Navigation library.
 *
 * This collection of macros and functions is used by the C code generated
 * from the XML flight plan.
 */

#ifndef NAV_H
#define NAV_H

#include "std.h"
#include "paparazzi.h"
#include "state.h"
#ifdef CTRL_TYPE_H
#include CTRL_TYPE_H
#endif
#include "subsystems/navigation/nav_survey_rectangle.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"

#define NAV_GRAVITY 9.806
#define Square(_x) ((_x)*(_x))
#define DistanceSquare(p1_x, p1_y, p2_x, p2_y) (Square(p1_x-p2_x)+Square(p1_y-p2_y))

#define PowerVoltage() (ap_electrical.vsupply)
#define RcRoll(travel) (imcu_get_radio(RADIO_ROLL) * (float)travel /(float)MAX_PPRZ)


enum oval_status { OR12, OC2, OR21, OC1 };

extern float cur_pos_x;
extern float cur_pos_y;
extern float last_x, last_y;

extern float desired_x, desired_y, nav_altitude, flight_altitude, nav_glide_pitch_trim;

extern pprz_t nav_throttle_setpoint;
extern float nav_pitch; /* Rad */
extern float rc_pitch;
extern float fp_pitch; /* Degrees */
extern float fp_throttle; /* [0-1] */
extern float fp_climb; /* m/s */

extern float carrot_x, carrot_y;

extern float nav_circle_radians; /* Cumulated */
extern float nav_circle_radians_no_rewind; /* Cumulated */
extern bool nav_in_circle;
extern bool nav_in_segment;
extern float nav_circle_x, nav_circle_y, nav_circle_radius; /* m */
extern float nav_segment_x_1, nav_segment_y_1, nav_segment_x_2, nav_segment_y_2; /* m */

extern uint8_t last_wp __attribute__((unused));

extern int nav_mode;
#define NAV_MODE_ROLL 1
#define NAV_MODE_COURSE 2

extern uint8_t horizontal_mode;

#define HORIZONTAL_MODE_WAYPOINT 0
#define HORIZONTAL_MODE_ROUTE 1
#define HORIZONTAL_MODE_CIRCLE 2

extern void fly_to_xy(float x, float y);

#define NavGotoWaypoint(_wp) { \
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
    fly_to_xy(waypoints[_wp].x, waypoints[_wp].y); \
  }


extern void nav_eight_init(void);
extern void nav_eight(uint8_t, uint8_t, float);
#define Eight(a, b, c) nav_eight((a), (b), (c))

extern void nav_oval_init(void);
extern void nav_oval(uint8_t, uint8_t, float);
extern uint8_t nav_oval_count;
#define Oval(a, b, c) nav_oval((b), (a), (c))

extern float nav_radius; /* m */
extern float nav_course; /* degrees, clockwise, 0.0 = N */
extern float nav_climb; /* m/s */
extern float nav_shift; /* Lateral shift along a route. In meters */
extern float sweep_var; //added to allow dynamic sweep width in surveys

extern float nav_ground_speed_pgain, nav_ground_speed_setpoint;


extern float nav_survey_shift;
extern float nav_survey_west, nav_survey_east, nav_survey_north, nav_survey_south;
extern bool nav_survey_active;

extern void nav_periodic_task(void);
extern void nav_home(void);
extern void nav_init(void);
extern void nav_without_gps(void);

extern float nav_circle_trigo_qdr; /** Angle from center to mobile */
extern void nav_circle_XY(float x, float y, float radius);

extern float baseleg_out_qdr;
extern void nav_compute_baseleg(uint8_t wp_af, uint8_t wp_td, uint8_t wp_baseleg, float radius);
extern void nav_compute_final_from_glide(uint8_t wp_af, uint8_t wp_td, float glide);

#define RCLost() bit_is_set(imcu_get_status(), STATUS_RADIO_REALLY_LOST)

extern void nav_follow(uint8_t _ac_id, float _distance, float _height);
#define NavFollow(_ac_id, _distance, _height) nav_follow(_ac_id, _distance, _height)

extern void nav_glide(uint8_t start_wp, uint8_t wp);
#define NavGlide(_start_wp, _wp) nav_glide(_start_wp, _wp)

#define NavCircleWaypoint(wp, radius) \
  nav_circle_XY(waypoints[wp].x, waypoints[wp].y, radius)

/** Normalize a degree angle between 0 and 359 */
#define NormCourse(x) { \
    uint8_t dont_loop_forever = 0;  \
    while (x < 0 && ++dont_loop_forever) x += 360; \
    while (x >= 360 && ++dont_loop_forever) x -= 360; \
  }

#define NavCircleCountNoRewind() (nav_circle_radians_no_rewind / (2*M_PI))
#define NavCircleCount() (fabs(nav_circle_radians) / (2*M_PI))
#define NavCircleQdr() ({ float qdr = DegOfRad(M_PI_2 - nav_circle_trigo_qdr); NormCourse(qdr); qdr; })

#define CloseDegAngles(_c1, _c2) ({ float _diff = _c1 - _c2; NormCourse(_diff); 350 < _diff || _diff < 10; })

/** True if x (in degrees) is close to the current QDR (less than 10 degrees)*/
#define NavQdrCloseTo(x) CloseDegAngles(x, NavCircleQdr())

#define NavCourseCloseTo(x) CloseDegAngles(x, DegOfRad(stateGetHorizontalSpeedDir_f()))

/*********** Navigation along a line *************************************/
extern void nav_route_xy(float last_wp_x, float last_wp_y, float wp_x, float wp_y);
#define NavSegment(_start, _end) \
  nav_route_xy(waypoints[_start].x, waypoints[_start].y, waypoints[_end].x, waypoints[_end].y)

bool nav_approaching_xy(float x, float y, float from_x, float from_y, float approaching_time);
#define NavApproaching(wp, time) nav_approaching_xy(waypoints[wp].x, waypoints[wp].y, last_x, last_y, time)
#define NavApproachingFrom(wp, from, time) nav_approaching_xy(waypoints[wp].x, waypoints[wp].y, waypoints[from].x, waypoints[from].y, time)


/** Set the climb control to auto-throttle with the specified pitch
    pre-command */
#define NavVerticalAutoThrottleMode(_pitch) { \
    v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE; \
    nav_pitch = _pitch; \
  }

/** Set the climb control to auto-pitch with the specified throttle
    pre-command */
#define NavVerticalAutoPitchMode(_throttle) { \
    v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_PITCH; \
    nav_throttle_setpoint = _throttle; \
  }

/** Set the vertical mode to altitude control with the specified altitude
 setpoint and climb pre-command. */
#define NavVerticalAltitudeMode(_alt, _pre_climb) { \
    v_ctl_mode = V_CTL_MODE_AUTO_ALT; \
    nav_altitude = _alt; \
    v_ctl_altitude_pre_climb = _pre_climb; \
  }

/** Set the vertical mode to climb control with the specified climb setpoint */
#define NavVerticalClimbMode(_climb) { \
    v_ctl_mode = V_CTL_MODE_AUTO_CLIMB; \
    v_ctl_climb_setpoint = _climb; \
  }

/** Set the vertical mode to fixed throttle with the specified setpoint */
#define NavVerticalThrottleMode(_throttle) { \
    v_ctl_mode = V_CTL_MODE_AUTO_THROTTLE; \
    nav_throttle_setpoint = _throttle; \
  }

#define NavHeading(_course) { \
    lateral_mode = LATERAL_MODE_COURSE; \
    h_ctl_course_setpoint = _course; \
  }

#define NavAttitude(_roll) { \
    lateral_mode = LATERAL_MODE_ROLL; \
    if(autopilot_get_mode() != AP_MODE_AUTO1)  \
    {h_ctl_roll_setpoint = _roll;} \
  }

#define NavSetManual(_roll, _pitch, _yaw) _Pragma("GCC error \"Manual mode in flight plan for fixedwing is not available\"")


#define nav_IncreaseShift(x) { if (x==0) nav_shift = 0; else nav_shift += x; }

#define nav_SetNavRadius(x) { if (x==1) nav_radius = DEFAULT_CIRCLE_RADIUS; else if (x==-1) nav_radius = -DEFAULT_CIRCLE_RADIUS; else nav_radius = x; }

#define NavKillThrottle() { autopilot_set_kill_throttle(true); }

/// Get current x (east) position in local coordinates
#define GetPosX() (stateGetPositionEnu_f()->x)
/// Get current y (north) position in local coordinates
#define GetPosY() (stateGetPositionEnu_f()->y)
/// Get current altitude above MSL
#define GetPosAlt() (stateGetPositionUtm_f()->alt)
/**
 * Get current altitude reference for local coordinates.
 * This is the ground_alt from the flight plan at first,
 * but might be updated later through a call to NavSetGroundReferenceHere() or
 * NavSetAltitudeReferenceHere(), e.g. in the GeoInit flight plan block.
 */
#define GetAltRef() (ground_alt)

#if DOWNLINK
#define SEND_NAVIGATION(_trans, _dev) { \
    uint8_t _circle_count = NavCircleCount(); \
    struct EnuCoor_f* pos = stateGetPositionEnu_f(); \
    float dist_wp = sqrtf(dist2_to_wp); \
    float dist_home = sqrtf(dist2_to_home); \
    uint8_t kill = (uint8_t)autopilot.kill_throttle; \
    pprz_msg_send_NAVIGATION(_trans, _dev, AC_ID, &nav_block, &nav_stage, &(pos->x), &(pos->y), &dist_wp, &dist_home, &autopilot.flight_time, &block_time, &stage_time, &kill, &_circle_count, &nav_oval_count); \
  }

extern void DownlinkSendWpNr(uint8_t _wp);

#endif /* DOWNLINK */

#endif /* NAV_H */

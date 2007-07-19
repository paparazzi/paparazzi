/*
 * $Id$
 *  
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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
/** \file nav.c
 *  \brief Regroup functions to compute navigation
 *
 */

#define NAV_C

#include <math.h>

#include "nav.h"
#include "gps.h"
#include "estimator.h"
#include "fw_h_ctl.h"
#include "fw_v_ctl.h"
#include "autopilot.h"
#include "inter_mcu.h"
#include "cam.h"
#include "traffic_info.h"
#include "latlong.h"

uint8_t nav_stage, nav_block;

/** To save the current block/stage to enable return */
static uint8_t last_block, last_stage;

float last_x, last_y;

/** Index of last waypoint. Used only in "go" stage in "route" horiz mode */
static uint8_t last_wp __attribute__ ((unused));

float rc_pitch;
uint16_t stage_time, block_time;
float stage_time_ds;
float carrot_x, carrot_y;

/** Status on the current circle */
float nav_circle_radians; /* Cumulated */
float nav_circle_trigo_qdr; /* Angle from center to mobile */
float nav_radius, nav_course;


/** Status on the current leg (percentage, 0. < < 1.) in route mode */
static float nav_leg_progress;

/** length of the current leg (m) */
static float nav_leg_length;

bool_t nav_in_circle = FALSE;
bool_t nav_in_segment = FALSE;
int16_t nav_circle_x, nav_circle_y, nav_circle_radius;
int16_t nav_segment_x_1, nav_segment_y_1, nav_segment_x_2, nav_segment_y_2;
uint8_t horizontal_mode;
float circle_bank = 0;

/** Dynamically adjustable, reset to nav_altitude when it is changing */
float flight_altitude;

float nav_glide_pitch_trim;

float nav_ground_speed_setpoint, nav_ground_speed_pgain;

/* Used in nav_survey_rectangle. Defined here for downlink and uplink */
float nav_survey_shift;
float nav_survey_west, nav_survey_east, nav_survey_north, nav_survey_south;

void nav_init_stage( void ) {
  last_x = estimator_x; last_y = estimator_y;
  stage_time = 0;
  stage_time_ds = 0;
  nav_circle_radians = 0;
  nav_in_circle = FALSE;
  nav_in_segment = FALSE;
}

#define PowerVoltage() (vsupply/10.)
#define RcRoll(travel) (fbw_state->channels[RADIO_ROLL]* (float)travel /(float)MAX_PPRZ)

#define RcEvent1() CheckEvent(rc_event_1)
#define RcEvent2() CheckEvent(rc_event_2)
#define Block(x) case x: nav_block=x;
#define InitBlock() { nav_stage = 0; block_time = 0; InitStage(); }
#define NextBlock() { nav_block++; InitBlock(); }
#define GotoBlock(b) { nav_block=b; InitBlock(); }
#define Return() { nav_block=last_block; nav_stage=last_stage; block_time=0; return;}

#define Stage(s) case s: nav_stage=s;
#define NextStage() { nav_stage++; InitStage() }
#define NextStageFrom(wp) { last_wp = wp; NextStage() }
#define GotoStage(s) { nav_stage = s; InitStage() }

#define Label(x) label_ ## x:
#define Goto(x) { goto label_ ## x; }

static inline void fly_to_xy(float x, float y);

#define MIN_DX ((int16_t)(MAX_PPRZ * 0.05))

void nav_circle_XY(float x, float y, float radius) {
  float last_trigo_qdr = nav_circle_trigo_qdr;
  nav_circle_trigo_qdr = atan2(estimator_y - y, estimator_x - x);

  if (nav_in_circle) {
    float trigo_diff = nav_circle_trigo_qdr - last_trigo_qdr;
    NormRadAngle(trigo_diff);
    nav_circle_radians += trigo_diff;
  }

  float dist2_center = DistanceSquare(estimator_x, estimator_y, x, y);
  float dist_carrot = CARROT*NOMINAL_AIRSPEED;
  float abs_radius = fabs(radius);
  float sign_radius = radius > 0 ? 1 : -1;
  
  /** Computes a prebank. Go straight if inside or outside the circle */
  circle_bank =
    (dist2_center > Square(abs_radius + dist_carrot)
      || dist2_center < Square(abs_radius - dist_carrot)) ?
    0 :
    atan(estimator_hspeed_mod*estimator_hspeed_mod / (G*radius));

  float carrot_angle = dist_carrot / abs_radius;
  carrot_angle = Min(carrot_angle, M_PI/4);
  carrot_angle = Max(carrot_angle, M_PI/16);
  float alpha_carrot = nav_circle_trigo_qdr - sign_radius * carrot_angle;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
  float radius_carrot = abs_radius + (abs_radius / cos(carrot_angle) - abs_radius);
  fly_to_xy(x+cos(alpha_carrot)*radius_carrot,
	    y+sin(alpha_carrot)*radius_carrot);
  nav_in_circle = TRUE;
  nav_circle_x = x;
  nav_circle_y = y;
  nav_circle_radius = radius;
}


#define NavGotoWaypoint(_wp) { \
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
  fly_to_xy(waypoints[_wp].x, waypoints[_wp].y); \
}

#define NavGlide(_last_wp, _wp) { \
  float start_alt = waypoints[_last_wp].a; \
  float diff_alt = waypoints[_wp].a - start_alt; \
  float alt = start_alt + nav_leg_progress * diff_alt; \
  float pre_climb = estimator_hspeed_mod * diff_alt / nav_leg_length; \
  NavVerticalAltitudeMode(alt, pre_climb); \
}




#define MAX_DIST_CARROT 250.
#define MIN_HEIGHT_CARROT 50.
#define MAX_HEIGHT_CARROT 150.

#define Goto3D(radius) { \
  if (pprz_mode == PPRZ_MODE_AUTO2) { \
    int16_t yaw = fbw_state->channels[RADIO_YAW]; \
    if (yaw > MIN_DX || yaw < -MIN_DX) { \
      carrot_x += FLOAT_OF_PPRZ(yaw, 0, -20.); \
      carrot_x = Min(carrot_x, MAX_DIST_CARROT); \
      carrot_x = Max(carrot_x, -MAX_DIST_CARROT); \
    } \
    int16_t pitch = fbw_state->channels[RADIO_PITCH]; \
    if (pitch > MIN_DX || pitch < -MIN_DX) { \
      carrot_y += FLOAT_OF_PPRZ(pitch, 0, -20.); \
      carrot_y = Min(carrot_y, MAX_DIST_CARROT); \
      carrot_y = Max(carrot_y, -MAX_DIST_CARROT); \
    } \
    v_ctl_mode = V_CTL_MODE_AUTO_ALT; \
    int16_t roll =  fbw_state->channels[RADIO_ROLL]; \
    if (roll > MIN_DX || roll < -MIN_DX) { \
      nav_altitude += FLOAT_OF_PPRZ(roll, 0, -1.0);	\
      nav_altitude = Max(nav_altitude, MIN_HEIGHT_CARROT+ground_alt); \
      nav_altitude = Min(nav_altitude, MAX_HEIGHT_CARROT+ground_alt); \
    } \
  } \
  nav_circle_XY(carrot_x, carrot_y, radius); \
}

#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define NavBlockTime() (block_time)
#define LessThan(_x, _y) ((_x) < (_y))

#define NavFollow(_ac_id, _distance, _height) \
  nav_follow(_ac_id, _distance, _height);


#define NavSetGroundReferenceHere() ({ nav_reset_reference(); nav_update_waypoints_alt(); FALSE; })


void nav_goto_block(uint8_t b) {
  if (b != nav_block) { /* To avoid a loop in a the current block */
    last_block = nav_block;
    last_stage = nav_stage;
  }
  GotoBlock(b);
}


static unit_t unit __attribute__ ((unused));

static unit_t nav_reset_reference( void ) __attribute__ ((unused));

static unit_t nav_update_waypoints_alt( void ) __attribute__ ((unused));
static inline void nav_follow(uint8_t _ac_id, float _distance, float _height);

#ifdef NAV_GROUND_SPEED_PGAIN
/** \brief Computes cruise throttle from ground speed setpoint
 */
static int nav_ground_speed_loop( void ) {
  float err = estimator_hspeed_mod - nav_ground_speed_setpoint;
  v_ctl_auto_throttle_cruise_throttle += nav_ground_speed_pgain*err;
  Bound(v_ctl_auto_throttle_cruise_throttle, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
  return FALSE;
}
#endif

static float baseleg_out_qdr;
static inline bool_t nav_compute_baseleg(uint8_t wp_af, uint8_t wp_td, uint8_t wp_baseleg ) {
  nav_radius = DEFAULT_CIRCLE_RADIUS;

  float x_0 = waypoints[wp_td].x - waypoints[wp_af].x;
  float y_0 = waypoints[wp_td].y - waypoints[wp_af].y;

  /* Unit vector from AF to TD */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[wp_baseleg].x = waypoints[wp_af].x + y_1 * nav_radius;
  waypoints[wp_baseleg].y = waypoints[wp_af].y - x_1 * nav_radius;
  waypoints[wp_baseleg].a = waypoints[wp_af].a;
  baseleg_out_qdr = M_PI - atan2(-y_1, -x_1);

  return FALSE;
}


#include "flight_plan.h"

float ground_alt;

static float previous_ground_alt;
	     
static inline void nav_follow(uint8_t _ac_id, float _distance, float _height) { 
  struct ac_info_ * ac = get_ac_info(_ac_id);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(Max(ac->alt + _height, ground_alt+SECURITY_HEIGHT), 0.);
  float alpha = M_PI/2 - ac->course;
  float ca = cos(alpha), sa = sin(alpha);
  float x = ac->east - _distance*ca;
  float y = ac->north - _distance*sa;
  fly_to_xy(x, y);
#ifdef NAV_FOLLOW_PGAIN
  float s = (estimator_x-x)*ca+(estimator_y-y)*sa;
  nav_ground_speed_setpoint = ac->gspeed + NAV_FOLLOW_PGAIN*s;
  nav_ground_speed_loop();
#endif
}



/** Reset the geographic reference to the current GPS fix */
static unit_t nav_reset_reference( void ) {
#ifdef GPS_USE_LATLONG
  /* Set the real UTM zone */
  nav_utm_zone0 = (gps_lon/10000000+180) / 6 + 1;

  /* Recompute UTM coordinates in this zone */
  latlong_utm_of(RadOfDeg(gps_lat/1e7), RadOfDeg(gps_lon/1e7), nav_utm_zone0);
  nav_utm_east0 = latlong_utm_x;
  nav_utm_north0 = latlong_utm_y;
#else
  nav_utm_zone0 = gps_utm_zone;
  nav_utm_east0 = gps_utm_east/100;
  nav_utm_north0 = gps_utm_north/100;
#endif

  previous_ground_alt = ground_alt;
  ground_alt = gps_alt/100;
  return 0;
}

/** Shift altitude of the waypoint according to a new ground altitude */
static unit_t nav_update_waypoints_alt( void ) {
  uint8_t i;
  for(i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].a += ground_alt - previous_ground_alt;
  }
  return 0;
}



int32_t nav_utm_east0 = NAV_UTM_EAST0;
int32_t nav_utm_north0 = NAV_UTM_NORTH0;
uint8_t nav_utm_zone0 = NAV_UTM_ZONE0;

float nav_altitude = GROUND_ALT + MIN_HEIGHT_CARROT;
float altitude_shift = 0;
float desired_x, desired_y;
pprz_t nav_throttle_setpoint;
float nav_pitch = NAV_PITCH;

float dist2_to_wp, dist2_to_home;
bool_t too_far_from_home;
const uint8_t nb_waypoint = NB_WAYPOINT;

struct point waypoints[NB_WAYPOINT] = WAYPOINTS;


/** \brief Decide if uav is approaching of current waypoint.
 *  Computes \a dist2_to_wp and compare it to square \a carrot.
 *  Return true if it is smaller. Else computes by scalar products if 
 *  uav has not gone past waypoint.
 *  Return true if it is the case.
 */
bool_t nav_approaching_xy(float x, float y, float from_x, float from_y, float approaching_time) {
  /** distance to waypoint in x */
  float pw_x = x - estimator_x;
  /** distance to waypoint in y */
  float pw_y = y - estimator_y;

  dist2_to_wp = pw_x*pw_x + pw_y *pw_y;
  float min_dist = approaching_time * estimator_hspeed_mod;
  if (dist2_to_wp < min_dist*min_dist)
    return TRUE;

  float scal_prod = (x - from_x) * pw_x + (y - from_y) * pw_y;
  
  return (scal_prod < 0.);
}


/**
 *  \brief Computes \a desired_x, \a desired_y and \a desired_course.
 */
static inline void fly_to_xy(float x, float y) { 
  desired_x = x;
  desired_y = y;
  h_ctl_course_setpoint = M_PI/2.-atan2(y - estimator_y, x - estimator_x);
  lateral_mode = LATERAL_MODE_COURSE;
}


/**
 *  \brief Computes the carrot position along the desired segment.
 */
void nav_route_xy(float last_wp_x, float last_wp_y, float wp_x, float wp_y) {
  float leg_x = wp_x - last_wp_x;
  float leg_y = wp_y - last_wp_y;
  float leg2 = Max(leg_x * leg_x + leg_y * leg_y, 1.);
  nav_leg_progress = ((estimator_x - last_wp_x) * leg_x + (estimator_y - last_wp_y) * leg_y) / leg2;
  // nav_leg_progress = Max(nav_leg_progress, 0.);
  nav_leg_length = sqrt(leg2);

  /** distance of carrot (in meter) */
  float carrot = CARROT * NOMINAL_AIRSPEED;

  nav_leg_progress += Max(carrot / nav_leg_length, 0.);
  nav_in_segment = TRUE;
  nav_segment_x_1 = last_wp_x;
  nav_segment_y_1 = last_wp_y;
  nav_segment_x_2 = wp_x;
  nav_segment_y_2 = wp_y;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;

  fly_to_xy(last_wp_x + nav_leg_progress*leg_x, last_wp_y + nav_leg_progress*leg_y);
}


/** \brief Computes square distance to the HOME waypoint potentially sets
 * \a too_far_from_home
 */
static inline void compute_dist2_to_home(void) {
  float ph_x = waypoints[WP_HOME].x - estimator_x;
  float ph_y = waypoints[WP_HOME].y - estimator_y;
  dist2_to_home = ph_x*ph_x + ph_y *ph_y;
  too_far_from_home = dist2_to_home > (MAX_DIST_FROM_HOME*MAX_DIST_FROM_HOME);
#if defined InAirspace
  too_far_from_home = too_far_from_home || !(InAirspace(estimator_x, estimator_y));
#endif
}


#ifndef FAILSAFE_HOME_RADIUS
#define FAILSAFE_HOME_RADIUS 50
#endif

/** \brief Home mode navigation (circle around HOME) */
void nav_home(void) {
  NavCircleWaypoint(WP_HOME, FAILSAFE_HOME_RADIUS);
  /** Nominal speed */ 
  nav_pitch = 0.;
  v_ctl_mode = V_CTL_MODE_AUTO_ALT;
  nav_altitude = ground_alt+SECURITY_HEIGHT;
  compute_dist2_to_home();
  dist2_to_wp = dist2_to_home;
}

/** 
 *  \brief Navigation main: call to the code generated from the XML flight
 * plan
 */
void nav_update(void) {
  compute_dist2_to_home();

  auto_nav(); /* From flight_plan.h */

  h_ctl_course_pre_bank = nav_in_circle ? circle_bank : 0;

#ifdef AGR_CLIMB
  if ( v_ctl_mode == V_CTL_MODE_AUTO_CLIMB)
    v_ctl_auto_throttle_submode =  V_CTL_AUTO_THROTTLE_STANDARD;
#endif

  static float last_nav_altitude;
  if (fabs(nav_altitude - last_nav_altitude) > 1.) {
    flight_altitude = nav_altitude;
    last_nav_altitude = nav_altitude;
  }
  v_ctl_altitude_setpoint = flight_altitude + altitude_shift;
}


/**
 *  \brief Navigation Initialisation
 */
void nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
  ground_alt = GROUND_ALT;
  nav_glide_pitch_trim = NAV_GLIDE_PITCH_TRIM;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_survey_shift = 2*DEFAULT_CIRCLE_RADIUS;

#ifdef NAV_GROUND_SPEED_PGAIN
  nav_ground_speed_pgain = NAV_GROUND_SPEED_PGAIN; 
  nav_ground_speed_setpoint = NOMINAL_AIRSPEED;
#endif
}

/** 
 *  \brief Failsafe navigation without position estimation
 * 
 * Just set attitude and throttle to FAILSAFE values
 * to prevent the plane from crashing.
 */
void nav_without_gps(void) {
  lateral_mode = LATERAL_MODE_ROLL;
  v_ctl_mode = V_CTL_MODE_AUTO_THROTTLE;

#ifdef SECTION_FAILSAFE
  h_ctl_roll_setpoint = FAILSAFE_DEFAULT_ROLL;
  nav_pitch = FAILSAFE_DEFAULT_PITCH;
  nav_throttle_setpoint = TRIM_UPPRZ((FAILSAFE_DEFAULT_THROTTLE)*MAX_PPRZ);
#else
  h_ctl_roll_setpoint = 0;
  nav_pitch = 0;
  nav_throttle_setpoint = TRIM_UPPRZ((CRUISE_THROTTLE)*MAX_PPRZ);
#endif
}


/**************** 8 Navigation **********************************************/


enum eight_status { R12, C2, R21, C1 };

static enum eight_status eight_status;
void nav_eight_init( void ) {
  eight_status = C1;
}

/** Navigation along a figure 8. The cross center is defined by the waypoint
    [target], the center of one of the circles is defined by [c1]. Altitude is
    given by [target].
    The navigation goes through 4 states: C1 (circle around [c1]), R12 (route
    from circle 1 to circle 2 over [target], C2 and R21.
    If necessary, the [c1] waypoint is moved in the direction of [target]
    to be not far than [2*radius].
*/
void nav_eight(uint8_t target, uint8_t c1, float radius) {
  float aradius = fabs(radius);
  float alt = waypoints[target].a;
  waypoints[c1].a = alt;

  float target_c1_x = waypoints[c1].x - waypoints[target].x;
  float target_c1_y = waypoints[c1].y - waypoints[target].y;
  float d = sqrt(target_c1_x*target_c1_x+target_c1_y*target_c1_y);
  d = Max(d, 1.); /* To prevent a division by zero */

  /* Unit vector from target to c1 */
  float u_x = target_c1_x / d;
  float u_y = target_c1_y / d;

  /* Move [c1] closer if needed */
  if (d > 2 * aradius) {
    d = 2*aradius;
    waypoints[c1].x = waypoints[target].x + d*u_x;
    waypoints[c1].y = waypoints[target].y + d*u_y;
  }

  /* The other center */
  struct point c2 = { 
    waypoints[target].x - d*u_x,
    waypoints[target].y - d*u_y,
    alt };

  struct point c1_in = {
    waypoints[c1].x + radius * -u_y,
    waypoints[c1].y + radius * u_x,
    alt };
  struct point c1_out = {
    waypoints[c1].x - radius * -u_y,
    waypoints[c1].y - radius * u_x,
    alt };
  
  struct point c2_in = {
    c2.x + radius * -u_y,
    c2.y + radius * u_x,
    alt };
  struct point c2_out = {
    c2.x - radius * -u_y,
    c2.y - radius * u_x,
    alt };
  
  float qdr_out = M_PI - atan2(u_y, u_x);
  if (radius < 0)
    qdr_out += M_PI;
  
  switch (eight_status) {
  case C1 :
    NavCircleWaypoint(c1, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out)-10)) {
      eight_status = R12;
      InitStage();
    }
    return;

  case R12:
    nav_route_xy(c1_out.x, c1_out.y, c2_in.x, c2_in.y);
    if (nav_approaching_xy(c2_in.x, c2_in.y, c1_out.x, c1_out.y, CARROT)) { 
      eight_status = C2;
      InitStage();
    }
    return;

  case C2 :
    nav_circle_XY(c2.x, c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out)+10)) {
      eight_status = R21;
      InitStage();
    }
    return;
    
  case R21:
    nav_route_xy(c2_out.x, c2_out.y, c1_in.x, c1_in.y);
    if (nav_approaching_xy(c1_in.x, c1_in.y, c2_out.x, c2_out.y, CARROT)) { 
      eight_status = C1;
      InitStage();
    }
    return;
  }
}

/************** Oval Navigation **********************************************/

/** Navigation along a figure O. One side leg is defined by waypoints [p1] and
    [p2]. 
    The navigation goes through 4 states: OC1 (half circle next to [p1]),
    OR21 (route [p2] to [p1], OC2 (half circle next to [p2]) and OR12 
    (opposite leg).

    Initial state is the route along the desired segment (OC2).
*/

enum oval_status { OR12, OC2, OR21, OC1 };

static enum oval_status oval_status;
void nav_oval_init( void ) {
  oval_status = OC2;
}

void nav_oval(uint8_t p1, uint8_t p2, float radius) {
  float alt = waypoints[p1].a;
  waypoints[p2].a = alt;

  float p2_p1_x = waypoints[p1].x - waypoints[p2].x;
  float p2_p1_y = waypoints[p1].y - waypoints[p2].y;
  float d = sqrt(p2_p1_x*p2_p1_x+p2_p1_y*p2_p1_y);

  /* Unit vector from p1 to p2 */
  float u_x = p2_p1_x / d;
  float u_y = p2_p1_y / d;

  /* The half circle centers and the other leg */
  struct point p1_center = { waypoints[p1].x + radius * -u_y,
			     waypoints[p1].y + radius * u_x,
			     alt  };
  struct point p1_out = { waypoints[p1].x + 2*radius * -u_y,
			  waypoints[p1].y + 2*radius * u_x,
			  alt  };
  
  struct point p2_in = { waypoints[p2].x + 2*radius * -u_y,
			 waypoints[p2].y + 2*radius * u_x,
			 alt  };
  struct point p2_center = { waypoints[p2].x + radius * -u_y,
			     waypoints[p2].y + radius * u_x,
			     alt  };
 
  float qdr_out_2 = M_PI - atan2(u_y, u_x);
  float qdr_out_1 = qdr_out_2 + M_PI;
  
  switch (oval_status) {
  case OC1 :
    nav_circle_XY(p1_center.x,p1_center.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_1)-10)) {
      oval_status = OR12;
      InitStage();
    }
    return;

  case OR12:
    nav_route_xy(p1_out.x, p1_out.y, p2_in.x, p2_in.y);
    if (nav_approaching_xy(p2_in.x, p2_in.y, p1_out.x, p1_out.y, CARROT)) { 
      oval_status = OC2;
      InitStage();
    }
    return;

  case OC2 :
    nav_circle_XY(p2_center.x, p2_center.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2)-10)) {
      oval_status = OR21;
      InitStage();
    }
   return;

  case OR21:
    nav_route_xy(waypoints[p2].x, waypoints[p2].y, waypoints[p1].x, waypoints[p1].y);
    if (nav_approaching_xy(waypoints[p1].x, waypoints[p1].y, waypoints[p2].x, waypoints[p2].y, CARROT)) { 
      oval_status = OC1;
      InitStage();
    }
    return;
  }
}

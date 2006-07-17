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
#include "pid.h"
#include "autopilot.h"
#include "inter_mcu.h"
#include "airframe.h"
#include "cam.h"
#include "traffic_info.h"

#define Distance2(p1_x, p1_y, p2_x, p2_y) ((p1_x-p2_x)*(p1_x-p2_x)+(p1_y-p2_y)*(p1_y-p2_y))
#define G 9.81

uint8_t nav_stage, nav_block;
/** To save the current stage when an exception is raised */
uint8_t excpt_stage;
static float last_x, last_y;

/** Index of last waypoint. Used only in "go" stage in "route" horiz mode */
static uint8_t last_wp __attribute__ ((unused));

float rc_pitch;
uint16_t stage_time, block_time;
float stage_time_ds;
float carrot_x, carrot_y;
static float sum_alpha, last_alpha;
/** represents in percent the stage where the uav is on a route
 *  or the qdr when uav circles.
 */
static float alpha;
/** Number of circle done */
float circle_count;
static bool_t new_circle;
bool_t in_circle = FALSE;
bool_t in_segment = FALSE;
int16_t circle_x, circle_y, circle_radius;
int16_t segment_x_1, segment_y_1, segment_x_2, segment_y_2;
uint8_t horizontal_mode;
float circle_bank = 0;


#define RcRoll(travel) (fbw_state->channels[RADIO_ROLL]* (float)travel /(float)MAX_PPRZ)

#define RcEvent1() CheckEvent(rc_event_1)
#define RcEvent2() CheckEvent(rc_event_2)
#define Block(x) case x: nav_block=x;
#define InitBlock() { nav_stage = 0; block_time = 0; InitStage(); }
#define NextBlock() { nav_block++; InitBlock(); }
#define GotoBlock(b) { nav_block=b; InitBlock(); }

#define Stage(s) case s: nav_stage=s;
#define InitStage() { last_x = estimator_x; last_y = estimator_y; stage_time = 0; stage_time_ds = 0; sum_alpha = 0; new_circle = TRUE; in_circle = FALSE; in_segment = FALSE; return; }
#define NextStage() { nav_stage++; InitStage() }
#define NextStageFrom(wp) { last_wp = wp; NextStage() }
#define GotoStage(s) { nav_stage = s; InitStage() }

#define Label(x) label_ ## x:
#define Goto(x) { goto label_ ## x; }

#define Exception(x) { excpt_stage = nav_stage; goto label_ ## x; }
#define ReturnFromException(_) { GotoStage(excpt_stage) }

static bool_t approaching(uint8_t, float) __attribute__ ((unused));
static inline void fly_to_xy(float x, float y);
static void fly_to(uint8_t wp) __attribute__ ((unused));
static void route_to(uint8_t _last_wp, uint8_t _wp) __attribute__ ((unused));
static void glide_to(uint8_t _last_wp, uint8_t _wp) __attribute__ ((unused));
static void route_to_xy(float last_wp_x, float last_wp_y, float wp_x, float wp_y);

#define MIN_DX ((int16_t)(MAX_PPRZ * 0.05))

#define DegOfRad(x) ((x) / M_PI * 180.)
#define RadOfDeg(x) ((x)/180. * M_PI)
#define NormCourse(x) { \
  while (x < 0) x += 360; \
  while (x >= 360) x -= 360; \
}

/** Degrees from 0 to 360 */
static float qdr;

#define CircleXY(x, y, radius) { \
  last_alpha = alpha; \
  alpha = atan2(estimator_y - y, estimator_x - x); \
  if (! new_circle) { \
    float alpha_diff = alpha - last_alpha; \
    NormRadAngle(alpha_diff); \
    sum_alpha += alpha_diff; \
  } else \
    new_circle = FALSE; \
  float dist2_center = Distance2(estimator_x, estimator_y, x, y); \
  float dist_carrot = CARROT*estimator_hspeed_mod; \
  float abs_radius = fabs(radius); \
  float sign_radius = radius > 0 ? 1 : -1; \
  circle_bank = \
    (dist2_center > Square(abs_radius + dist_carrot) \
      || dist2_center < Square(abs_radius - dist_carrot)) ? \
    0 : \
     sign_radius * atan2(estimator_hspeed_mod*estimator_hspeed_mod, \
                         G*abs_radius); \
  circle_count = fabs(sum_alpha) / (2*M_PI); \
  float carrot_angle = CARROT / abs_radius * estimator_hspeed_mod; \
  carrot_angle = Min(carrot_angle, M_PI/4); \
  carrot_angle = Max(carrot_angle, M_PI/16); \
  float alpha_carrot = alpha - sign_radius * carrot_angle; \
  horizontal_mode = HORIZONTAL_MODE_CIRCLE; \
  float radius_carrot = abs_radius + (abs_radius / cos(carrot_angle) - abs_radius); \
  fly_to_xy(x+cos(alpha_carrot)*radius_carrot, \
	    y+sin(alpha_carrot)*radius_carrot); \
  qdr = DegOfRad(M_PI/2 - alpha); \
  NormCourse(qdr); \
  in_circle = TRUE; \
  circle_x = x; \
  circle_y = y; \
  circle_radius = radius; \
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
    vertical_mode = VERTICAL_MODE_AUTO_ALT; \
    int16_t roll =  fbw_state->channels[RADIO_ROLL]; \
    if (roll > MIN_DX || roll < -MIN_DX) { \
      nav_altitude += FLOAT_OF_PPRZ(roll, 0, -1.0);	\
      nav_altitude = Max(nav_altitude, MIN_HEIGHT_CARROT+ground_alt); \
      nav_altitude = Min(nav_altitude, MAX_HEIGHT_CARROT+ground_alt); \
    } \
  } \
  CircleXY(carrot_x, carrot_y, radius); \
}
#define Circle(wp, radius) \
  CircleXY(waypoints[wp].x, waypoints[wp].y, radius);

#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define Qdr(x) (Min(x, 350) < qdr && qdr < x+10)

#define Follow(_ac_id, _distance, _height) { \
  struct ac_info_ * ac = get_ac_info(_ac_id); \
  vertical_mode = VERTICAL_MODE_AUTO_ALT; \
  nav_altitude = Max(ac->alt + _height, SECURITY_ALT); \
  float alpha = M_PI/2 - ac->course; \
  fly_to_xy(ac->east - _distance*cos(alpha), ac->north - _distance*sin(alpha)); \
}

/** Automatic survey of a sector (south-north sweep) */
static struct point survey_from;
static struct point survey_to;
static float shift;
static bool_t survey_uturn __attribute__ ((unused)) = FALSE;
float survey_west, survey_east, survey_north, survey_south;
static float survey_radius __attribute__ ((unused));

#include <stdio.h>

#define Survey(_inside_sector, x_east, x_west, y_south, y_north) { \
  if (!_inside_sector(estimator_x, estimator_y)) { \
    if (! survey_uturn) { \
      survey_uturn = TRUE; \
      float x0 = survey_from.x; \
      if (x0+shift > x_west || x0+shift < x_east) { \
        shift = -shift; \
      } \
      x0 = x0 + shift; \
      survey_from.x = survey_to.x = x0; \
      float tmp = survey_from.y; \
      survey_from.y = survey_to.y; \
      survey_to.y = tmp; \
    } \
  } else \
    survey_uturn = FALSE; \
  route_to_xy(survey_from.x, survey_from.y, survey_to.x, survey_to.y); \
}

#define SurveyGoingNorth() (survey_to.y > survey_from.y)
#define SurveyGoingSouth() (survey_to.y < survey_from.y)


#define SurveyRectangle(wp1, wp2) { \
  survey_west = Min(waypoints[wp1].x, waypoints[wp2].x); \
  survey_east = Max(waypoints[wp1].x, waypoints[wp2].x); \
  survey_south = Min(waypoints[wp1].y, waypoints[wp2].y); \
  survey_north = Max(waypoints[wp1].y, waypoints[wp2].y); \
  if (survey_from.y < survey_to.y) { \
    survey_to.y = survey_north; \
    survey_from.y = survey_south; \
  } \
  if (survey_to.y < survey_from.y) { \
    survey_to.y = survey_south; \
    survey_from.y = survey_north; \
  } \
  if (! survey_uturn) { \
    if ((estimator_y < survey_north && SurveyGoingNorth()) || \
        (estimator_y > survey_south && SurveyGoingSouth())) { \
      route_to_xy(survey_from.x, survey_from.y, survey_to.x, survey_to.y); \
    } else { \
      survey_uturn = TRUE; \
      /** Prepare the next leg */ \
      float x0 = survey_from.x; \
      if (x0+shift < survey_west || x0+shift > survey_east) { \
        shift = -shift; \
      } \
      x0 = x0 + shift; \
      survey_from.x = survey_to.x = x0; \
      float tmp = survey_from.y; \
      survey_from.y = survey_to.y; \
      survey_to.y = tmp; \
      /** Do half a circle */ \
      waypoints[0].x = x0 - shift/2.; \
      waypoints[0].y = survey_from.y; \
      survey_radius = shift / 2.; \
      if (survey_to.y > survey_from.y) \
        survey_radius = -survey_radius; \
      sum_alpha = 0.; circle_count = 0.; new_circle = TRUE; in_segment = FALSE; \
    } \
  } else { \
    if (circle_count < 0.5) { \
      Circle(0, survey_radius); \
    } else { \
      survey_uturn = FALSE; \
      in_circle = FALSE; \
    } \
  } \
  vertical_mode = VERTICAL_MODE_AUTO_ALT; \
  nav_altitude = waypoints[wp1].a; \
}


void nav_goto_block(uint8_t b) {
  GotoBlock(b);
}

static inline void survey_init(float y_south, float y_north, float grid) {
  survey_from.x = survey_to.x = estimator_x;
  survey_from.y = y_south;
  survey_to.y = y_north;
  shift = grid;
}

static inline void survey_rectangle_init(uint8_t wp1, uint8_t wp2, float grid) {
  survey_west = Min(waypoints[wp1].x, waypoints[wp2].x);
  survey_east = Max(waypoints[wp1].x, waypoints[wp2].x);
  survey_south = Min(waypoints[wp1].y, waypoints[wp2].y);
  survey_north = Max(waypoints[wp1].y, waypoints[wp2].y);
  survey_from.x = survey_to.x = Min(Max(estimator_x, survey_west+grid/2.), survey_east-grid/2.);
  if (estimator_y > survey_north || (estimator_y > survey_south && estimator_hspeed_dir > M_PI/2. && estimator_hspeed_dir < 3*M_PI/2)) {
    survey_to.y = survey_south;
    survey_from.y = survey_north;
  } else {
    survey_from.y = survey_south;
    survey_to.y = survey_north;
  }
  shift = grid;
  survey_uturn = FALSE;
}

typedef uint8_t unit_;
static unit_ unit __attribute__ ((unused));

static unit_ reset_nav_reference( void ) __attribute__ ((unused));

static unit_ reset_waypoints( void ) __attribute__ ((unused));

extern float nav_altitude;

#include "flight_plan.h"

float ground_alt = GROUND_ALT;

static unit_ reset_nav_reference( void ) {
  nav_utm_east0 = gps_utm_east/100;
  nav_utm_north0 = gps_utm_north/100;
  nav_utm_zone0 = gps_utm_zone;
  ground_alt = gps_alt/100;
  return 0;
}

static unit_ reset_waypoints( void ) {
  uint8_t i;
  for(i = 0; i <= NB_WAYPOINT; i++) {
    waypoints[i].a = waypoints[i].a + ground_alt - GROUND_ALT;
    moved_waypoints[i] = TRUE;
  }
  return 0;
}



#define MIN_DIST2_WP (15.*15.)

// #define DISTANCE2(p1_x, p1_y, p2) ((p1_x-p2.x)*(p1_x-p2.x)+(p1_y-p2.y)*(p1_y-p2.y))

int32_t nav_utm_east0 = NAV_UTM_EAST0;
int32_t nav_utm_north0 = NAV_UTM_NORTH0;
uint8_t nav_utm_zone0 = NAV_UTM_ZONE0;

float nav_altitude = GROUND_ALT + MIN_HEIGHT_CARROT;
float desired_altitude, altitude_shift = 0;
float desired_x, desired_y;
uint16_t nav_desired_gaz;
float nav_pitch = NAV_PITCH;
float nav_desired_roll;

float dist2_to_wp, dist2_to_home;
bool_t too_far_from_home;
const uint8_t nb_waypoint = NB_WAYPOINT;

struct point waypoints[NB_WAYPOINT+1] = WAYPOINTS;
bool_t moved_waypoints[NB_WAYPOINT+1];



/** \brief Decide if uav is approaching of current waypoint.
 *  Computes \a dist2_to_wp and compare it to square \a carrot.
 *  Return true if it is smaller. Else computes by scalar products if 
 *  uav has not gone past waypoint.
 *  Return true if it is the case.
 */
static bool_t approaching(uint8_t wp, float approaching_time) {
  /** distance to waypoint in x */
  float pw_x = waypoints[wp].x - estimator_x;
  /** distance to waypoint in y */
  float pw_y = waypoints[wp].y - estimator_y;

  dist2_to_wp = pw_x*pw_x + pw_y *pw_y;
  float min_dist = approaching_time * estimator_hspeed_mod;
  if (dist2_to_wp < min_dist*min_dist)
    return TRUE;

  float scal_prod = (waypoints[wp].x - last_x) * pw_x + (waypoints[wp].y - last_y) * pw_y;
  
  return (scal_prod < 0.);
}

/** static inline void fly_to_xy(float x, float y)
 *  \brief Computes \a desired_x, \a desired_y and \a desired_course.
 */
static inline void fly_to_xy(float x, float y) { 
  desired_x = x;
  desired_y = y;
  desired_course = M_PI/2.-atan2(y - estimator_y, x - estimator_x);
}

/** static void fly_to(uint8_t wp)
 *  \brief Just call \a fly_to_xy with x and y of current waypoint.
 */
static void fly_to(uint8_t wp) { 
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  fly_to_xy(waypoints[wp].x, waypoints[wp].y);
}

/** length of the leg */
static float leg;
/** static void route_to(uint8_t _last_wp, uint8_t wp)
 *  \brief Computes where the uav is on a route and call
 *  \a fly_to_xy to follow it.
 */
static void route_to_xy(float last_wp_x, float last_wp_y, float wp_x, float wp_y) {
  float leg_x = wp_x - last_wp_x;
  float leg_y = wp_y - last_wp_y;
  float leg2 = leg_x * leg_x + leg_y * leg_y;
  alpha = ((estimator_x - last_wp_x) * leg_x + (estimator_y - last_wp_y) * leg_y) / leg2;
  alpha = Max(alpha, 0.);
  leg = sqrt(leg2);

  /** distance of carrot (in meter) */
  float carrot = CARROT * estimator_hspeed_mod;

  alpha += Max(carrot / leg, 0.);
  alpha = Min(1., alpha);
  in_segment = TRUE;
  segment_x_1 = last_wp_x;
  segment_y_1 = last_wp_y;
  segment_x_2 = wp_x;
  segment_y_2 = wp_y;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;

  fly_to_xy(last_wp_x + alpha*leg_x, last_wp_y + alpha*leg_y);
}

static void route_to(uint8_t _last_wp, uint8_t wp) {
  route_to_xy(waypoints[_last_wp].x, waypoints[_last_wp].y, 
	      waypoints[wp].x, waypoints[wp].y);
}


/** static void glide_to(uint8_t _last_wp, uint8_t wp)
 *  \brief Computes \a nav_altitude and \a pre_climb to stay on a glide.
 */
static void glide_to(uint8_t _last_wp, uint8_t wp) {
  float last_alt = waypoints[_last_wp].a;
  nav_altitude = last_alt + alpha * (waypoints[wp].a - last_alt);
  pre_climb = NOMINAL_AIRSPEED * (waypoints[wp].a - last_alt) / leg;
}

/** \brief Compute square distance to waypoint Home and
 *  verify uav is not too far away from Home.
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

/** \brief Occurs when it switchs in Home mode.
 */
void nav_home(void) {
  Circle(WP_HOME, FAILSAFE_HOME_RADIUS);
  /** Nominal speed */ 
  nav_pitch = 0.;
  vertical_mode = VERTICAL_MODE_AUTO_ALT;
  nav_altitude = ground_alt+50;
  compute_dist2_to_home();
  dist2_to_wp = dist2_to_home;
}

/** void nav_update(void)
 *  \brief Update navigation
 */
void nav_update(void) {
  compute_dist2_to_home();
  auto_nav();
}


/** void nav_init(void)
 *  \brief Initialisation of navigation
 */
void nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
}

/** void nav_wihtout_gps(void)
 *  \brief 
 */
/** Don't navigate anymore. It's impossible without GPS.
 *	Just set attitude and gaz to failsafe values
 *	to prevent the plane from crashing.
 */
void nav_without_gps(void) {
#ifdef SECTION_FAILSAFE
  lateral_mode = LATERAL_MODE_ROLL;
  nav_desired_roll = FAILSAFE_DEFAULT_ROLL;
  nav_pitch = FAILSAFE_DEFAULT_PITCH;
  vertical_mode = VERTICAL_MODE_AUTO_GAZ;
  nav_desired_gaz = TRIM_UPPRZ((FAILSAFE_DEFAULT_GAZ)*MAX_PPRZ);
#else
  lateral_mode = LATERAL_MODE_ROLL;
  nav_desired_roll = 0;
  nav_pitch = 0;
  vertical_mode = VERTICAL_MODE_AUTO_GAZ;
  nav_desired_gaz = TRIM_UPPRZ((CLIMB_LEVEL_GAZ)*MAX_PPRZ);
#endif
}

float course_pgain = COURSE_PGAIN;
float desired_course = 0.;
float max_roll = MAX_ROLL;
float altitude_error;

#define CLIMB_MODE_GAZ 0 
#define CLIMB_MODE_PITCH 1
#define CLIMB_MODE_AGRESSIVE 2
#define CLIMB_MODE_BLENDED 3

uint8_t climb_mode = CLIMB_MODE_GAZ;


/** \brief Computes ::nav_desired_roll from course estimation and expected
 course.
*/
void course_pid_run( void ) {
  float err = estimator_hspeed_dir - desired_course;
  NormRadAngle(err);
  float speed_depend_nav = estimator_hspeed_mod/NOMINAL_AIRSPEED; 
  speed_depend_nav = Max(speed_depend_nav,0.66);
  speed_depend_nav = Min(speed_depend_nav,1.5);
  float roll_from_err = course_pgain * speed_depend_nav * err;
#if defined  AGR_CLIMB_GAZ
  if (climb_mode == CLIMB_MODE_AGRESSIVE) {
    if (altitude_error < 0)
      roll_from_err *= AGR_CLIMB_NAV_RATIO;
    else
      roll_from_err *= AGR_DESCENT_NAV_RATIO;
  }
#endif
  nav_desired_roll = (in_circle ? circle_bank : 0) + roll_from_err;
  if (nav_desired_roll > max_roll)
    nav_desired_roll = max_roll;
  else if (nav_desired_roll < -max_roll)
    nav_desired_roll = -max_roll;
}

#define MAX_CLIMB_SUM_ERR 150
#define MAX_PITCH_CLIMB_SUM_ERR 100

float climb_pgain   = CLIMB_PGAIN;
float climb_igain   =  CLIMB_IGAIN;
float desired_climb = 0., pre_climb = 0.;
float climb_sum_err  = 0;

float climb_pitch_pgain = CLIMB_PITCH_PGAIN;
float climb_pitch_igain = CLIMB_PITCH_IGAIN;
float climb_pitch_sum_err = 0.;
float climb_level_gaz = CLIMB_LEVEL_GAZ;

/** \brief Computes desired_gaz and updates nav_pitch from desired_climb */
void 
climb_pid_run ( void ) {
  float fgaz = 0;
  float err  = estimator_z_dot - desired_climb;
  float controlled_gaz = climb_pgain * (err + climb_igain * climb_sum_err) + climb_level_gaz + CLIMB_GAZ_OF_CLIMB*desired_climb;
  /* pitch offset for climb */
  pitch_of_vz = desired_climb * pitch_of_vz_pgain;
  switch (climb_mode) {
  case CLIMB_MODE_GAZ:
    fgaz = controlled_gaz;
    climb_sum_err += err;
    Bound(climb_sum_err, -MAX_CLIMB_SUM_ERR, MAX_CLIMB_SUM_ERR);
    nav_pitch += pitch_of_vz;
    break;
    
  case CLIMB_MODE_PITCH:
    fgaz = nav_desired_gaz;
    nav_pitch = climb_pitch_pgain * (err + climb_pitch_igain * climb_pitch_sum_err);
    Bound(nav_pitch, MIN_PITCH, MAX_PITCH);
    climb_pitch_sum_err += err;
    BoundAbs(climb_pitch_sum_err, MAX_PITCH_CLIMB_SUM_ERR);
    break;

#if defined AGR_CLIMB_GAZ
  case CLIMB_MODE_AGRESSIVE:
    if (altitude_error < 0) {
      fgaz =  AGR_CLIMB_GAZ;
      nav_pitch = AGR_CLIMB_PITCH;
    } else {
      fgaz =  AGR_DESCENT_GAZ;
      nav_pitch = AGR_DESCENT_PITCH;
    }
    break;
    
  case CLIMB_MODE_BLENDED: {
    float ratio = (fabs(altitude_error) - AGR_BLEND_END) / (AGR_BLEND_START-AGR_BLEND_END);
    if (altitude_error < 0) {
      fgaz =  ratio*AGR_CLIMB_GAZ + (1-ratio)*controlled_gaz;
      nav_pitch = ratio*AGR_CLIMB_PITCH + (1-ratio)*pitch_of_vz;
    } else {
      fgaz =  ratio*AGR_DESCENT_GAZ + (1-ratio)*controlled_gaz;
      nav_pitch = ratio*AGR_DESCENT_PITCH + (1-ratio)*pitch_of_vz;
    }
    break;
  }
#endif
  }
  desired_gaz = TRIM_UPPRZ(fgaz * MAX_PPRZ);
}

float altitude_pgain = ALTITUDE_PGAIN;


void altitude_pid_run(void) {
  desired_altitude = nav_altitude + altitude_shift;
  altitude_error = estimator_z - desired_altitude;
  
#ifdef AGR_CLIMB_GAZ
  float dist = fabs(altitude_error);
  if (climb_mode == CLIMB_MODE_PITCH || dist < AGR_BLEND_END) {
#endif
    desired_climb = pre_climb + altitude_pgain * altitude_error;
    Bound(desired_climb, -CLIMB_MAX, CLIMB_MAX);
#ifdef AGR_CLIMB_GAZ
  } else if (dist > AGR_BLEND_START)
    climb_mode = CLIMB_MODE_AGRESSIVE;
  else
    climb_mode = CLIMB_MODE_BLENDED;
#endif
}

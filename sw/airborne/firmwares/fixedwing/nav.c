/*
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
 */

/**
 * @file firmwares/fixedwing/nav.c
 * Fixedwing functions to compute navigation.
 *
 */

#include <math.h>
#include "std.h"

static unit_t unit __attribute__((unused));

#define NAV_C
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "autopilot.h"
#include "inter_mcu.h"
#include "modules/gps/gps.h"

#include "generated/flight_plan.h"


enum oval_status oval_status;

float last_x, last_y;

/** Index of last waypoint. Used only in "go" stage in "route" horiz mode */
uint8_t last_wp __attribute__((unused));

float rc_pitch;
float carrot_x, carrot_y;

/** Status on the current circle */
float nav_circle_radians; /* Cumulated */
float nav_circle_radians_no_rewind; /* Cumulated */
float nav_circle_trigo_qdr; /* Angle from center to mobile */
float nav_radius, nav_course, nav_climb, nav_shift;


/** Status on the current leg (percentage, 0. < < 1.) in route mode */
static float nav_leg_progress;
static float nav_carrot_leg_progress;

/** length of the current leg (m) */
static float nav_leg_length;

bool nav_in_circle = false;
bool nav_in_segment = false;
float nav_circle_x, nav_circle_y, nav_circle_radius;
float nav_segment_x_1, nav_segment_y_1, nav_segment_x_2, nav_segment_y_2;
uint8_t horizontal_mode;
float circle_bank = 0;

/** Dynamically adjustable, reset to nav_altitude when it is changing */
float flight_altitude;

float nav_glide_pitch_trim;
#ifndef NAV_GLIDE_PITCH_TRIM
#define NAV_GLIDE_PITCH_TRIM 0.
#endif



float nav_ground_speed_setpoint, nav_ground_speed_pgain;

/* Used in nav_survey_rectangle. Defined here for downlink and uplink */
float nav_survey_shift;
float nav_survey_west, nav_survey_east, nav_survey_north, nav_survey_south;
bool nav_survey_active;

int nav_mode;

void nav_init_stage(void)
{
  last_x = stateGetPositionEnu_f()->x;
  last_y = stateGetPositionEnu_f()->y;
  stage_time = 0;
  nav_circle_radians = 0;
  nav_circle_radians_no_rewind = 0;
  nav_in_circle = false;
  nav_in_segment = false;
  nav_shift = 0;
}

#define MIN_DX ((int16_t)(MAX_PPRZ * 0.05))


/** Navigates around (x, y). Clockwise iff radius > 0 */
void nav_circle_XY(float x, float y, float radius)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  float last_trigo_qdr = nav_circle_trigo_qdr;
  nav_circle_trigo_qdr = atan2f(pos->y - y, pos->x - x);
  float sign_radius = radius > 0 ? 1 : -1;

  if (nav_in_circle) {
    float trigo_diff = nav_circle_trigo_qdr - last_trigo_qdr;
    NormRadAngle(trigo_diff);
    nav_circle_radians += trigo_diff;
    trigo_diff *= - sign_radius;
    if (trigo_diff > 0) { // do not rewind if the change in angle is in the opposite sense than nav_radius
      nav_circle_radians_no_rewind += trigo_diff;
    }
  }

  float dist2_center = DistanceSquare(pos->x, pos->y, x, y);
  float dist_carrot = CARROT * NOMINAL_AIRSPEED;

  radius += -nav_shift;

  float abs_radius = fabs(radius);

  /** Computes a prebank. Go straight if inside or outside the circle */
  circle_bank =
    (dist2_center > Square(abs_radius + dist_carrot)
     || dist2_center < Square(abs_radius - dist_carrot)) ?
    0 :
    atanf(stateGetHorizontalSpeedNorm_f() * stateGetHorizontalSpeedNorm_f() / (NAV_GRAVITY * radius));

  float carrot_angle = dist_carrot / abs_radius;
  carrot_angle = Min(carrot_angle, M_PI / 4);
  carrot_angle = Max(carrot_angle, M_PI / 16);
  float alpha_carrot = nav_circle_trigo_qdr - sign_radius * carrot_angle;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
  float radius_carrot = abs_radius;
  if (nav_mode == NAV_MODE_COURSE) {
    radius_carrot += (abs_radius / cosf(carrot_angle) - abs_radius);
  }
  fly_to_xy(x + cosf(alpha_carrot)*radius_carrot,
            y + sinf(alpha_carrot)*radius_carrot);
  nav_in_circle = true;
  nav_circle_x = x;
  nav_circle_y = y;
  nav_circle_radius = radius;
}


void nav_glide(uint8_t start_wp, uint8_t wp)
{
  float start_alt = waypoints[start_wp].a;
  float diff_alt = waypoints[wp].a - start_alt;
  float alt = start_alt + nav_leg_progress * diff_alt;
  float pre_climb = stateGetHorizontalSpeedNorm_f() * diff_alt / nav_leg_length;
  NavVerticalAltitudeMode(alt, pre_climb);
}


#define MAX_DIST_CARROT 250.
#define MIN_HEIGHT_CARROT 50.
#define MAX_HEIGHT_CARROT 150.

#define Goto3D(radius) {                                                \
    if (autopilot_get_mode() == AP_MODE_AUTO2) {                                 \
      int16_t yaw = imcu_get_radio(RADIO_YAW);                          \
      if (yaw > MIN_DX || yaw < -MIN_DX) {                              \
        carrot_x += FLOAT_OF_PPRZ(yaw, 0, -20.);                        \
        carrot_x = Min(carrot_x, MAX_DIST_CARROT);                      \
        carrot_x = Max(carrot_x, -MAX_DIST_CARROT);                     \
      }                                                                 \
      int16_t pitch = imcu_get_radio(RADIO_PITCH);                      \
      if (pitch > MIN_DX || pitch < -MIN_DX) {                          \
        carrot_y += FLOAT_OF_PPRZ(pitch, 0, -20.);                      \
        carrot_y = Min(carrot_y, MAX_DIST_CARROT);                      \
        carrot_y = Max(carrot_y, -MAX_DIST_CARROT);                     \
      }                                                                 \
      v_ctl_mode = V_CTL_MODE_AUTO_ALT;                                 \
      int16_t roll =  imcu_get_radio(RADIO_ROLL);                       \
      if (roll > MIN_DX || roll < -MIN_DX) {                            \
        nav_altitude += FLOAT_OF_PPRZ(roll, 0, -1.0);                   \
        nav_altitude = Max(nav_altitude, MIN_HEIGHT_CARROT+ground_alt); \
        nav_altitude = Min(nav_altitude, MAX_HEIGHT_CARROT+ground_alt); \
      }                                                                 \
    }                                                                   \
    nav_circle_XY(carrot_x, carrot_y, radius);                          \
  }



#ifdef NAV_GROUND_SPEED_PGAIN
/** \brief Computes cruise throttle from ground speed setpoint
 */
static void nav_ground_speed_loop(void)
{
  if (MINIMUM_AIRSPEED < nav_ground_speed_setpoint
      && nav_ground_speed_setpoint < MAXIMUM_AIRSPEED) {
    float err = nav_ground_speed_setpoint - stateGetHorizontalSpeedNorm_f();
    v_ctl_auto_throttle_cruise_throttle += nav_ground_speed_pgain * err;
    Bound(v_ctl_auto_throttle_cruise_throttle, v_ctl_auto_throttle_min_cruise_throttle,
          v_ctl_auto_throttle_max_cruise_throttle);
  } else {
    /* Reset cruise throttle to nominal value */
    v_ctl_auto_throttle_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  }
}
#endif

float baseleg_out_qdr;
void nav_compute_baseleg(uint8_t wp_af, uint8_t wp_td, uint8_t wp_baseleg, float radius)
{
  nav_radius = radius;

  float x_0 = waypoints[wp_td].x - waypoints[wp_af].x;
  float y_0 = waypoints[wp_td].y - waypoints[wp_af].y;

  /* Unit vector from AF to TD */
  float d = sqrtf(x_0 * x_0 + y_0 * y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[wp_baseleg].x = waypoints[wp_af].x + y_1 * nav_radius;
  waypoints[wp_baseleg].y = waypoints[wp_af].y - x_1 * nav_radius;
  waypoints[wp_baseleg].a = waypoints[wp_af].a;
  baseleg_out_qdr = M_PI - atan2f(-y_1, -x_1);
  if (nav_radius < 0) {
    baseleg_out_qdr += M_PI;
  }
}

void nav_compute_final_from_glide(uint8_t wp_af, uint8_t wp_td, float glide)
{

  float x_0 = waypoints[wp_td].x - waypoints[wp_af].x;
  float y_0 = waypoints[wp_td].y - waypoints[wp_af].y;
  float h_0 = waypoints[wp_td].a - waypoints[wp_af].a;

  /* Unit vector from AF to TD */
  float d = sqrtf(x_0 * x_0 + y_0 * y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[wp_af].x = waypoints[wp_td].x + x_1 * h_0 * glide;
  waypoints[wp_af].y = waypoints[wp_td].y + y_1 * h_0 * glide;
}


/* For a landing UPWIND.
   Computes Top Of Descent waypoint from Touch Down and Approach Fix
   waypoints, using glide airspeed, glide vertical speed and wind */
static inline void compute_TOD(uint8_t _af, uint8_t _td, uint8_t _tod, float glide_airspeed, float glide_vspeed)
{
  struct FloatVect2 *wind = stateGetHorizontalWindspeed_f();
  float td_af_x = WaypointX(_af) - WaypointX(_td);
  float td_af_y = WaypointY(_af) - WaypointY(_td);
  float td_af = sqrtf(td_af_x * td_af_x + td_af_y * td_af_y);
  float td_tod = (WaypointAlt(_af) - WaypointAlt(_td)) / glide_vspeed * (glide_airspeed - sqrtf(
                   wind->x * wind->x + wind->y * wind->y));
  WaypointX(_tod) = WaypointX(_td) + td_af_x / td_af * td_tod;
  WaypointY(_tod) = WaypointY(_td) + td_af_y / td_af * td_tod;
  WaypointAlt(_tod) = WaypointAlt(_af);
}


#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

#ifdef TRAFFIC_INFO
#include "modules/multi/traffic_info.h"

void nav_follow(uint8_t ac_id, float distance, float height)
{
  struct EnuCoor_f *ac = acInfoGetPositionEnu_f(ac_id);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(Max(ac->z + height, ground_alt + SECURITY_HEIGHT), 0.);
  float alpha = M_PI / 2 - acInfoGetCourse(ac_id);
  float ca = cosf(alpha), sa = sinf(alpha);
  float x = ac->x - distance * ca;
  float y = ac->y - distance * sa;
  fly_to_xy(x, y);
#ifdef NAV_FOLLOW_PGAIN
  float s = (stateGetPositionEnu_f()->x - x) * ca + (stateGetPositionEnu_f()->y - y) * sa;
  nav_ground_speed_setpoint = acInfoGetGspeed(ac_id) + NAV_FOLLOW_PGAIN * s;
  nav_ground_speed_loop();
#endif
}
#else
void nav_follow(uint8_t  __attribute__((unused)) _ac_id, float  __attribute__((unused)) distance,
                float  __attribute__((unused)) height) {}
#endif // TRAFFIC_INFO


float nav_altitude = GROUND_ALT + MIN_HEIGHT_CARROT;
float desired_x, desired_y;
pprz_t nav_throttle_setpoint;
float nav_pitch; /* Rad */
float fp_pitch; /* deg */
float fp_throttle; /* [0-1] */
float fp_climb; /* m/s */


/** \brief Decide if the UAV is approaching the current waypoint.
 *  Computes \a dist2_to_wp and compare it to square \a carrot.
 *  Return true if it is smaller. Else computes by scalar products if
 *  uav has not gone past waypoint.
 *  \a approaching_time can be negative and in this case, the UAV will
 *  fly after the waypoint for the given number of seconds.
 *
 *  @return true if the position (x, y) is reached
 */
bool nav_approaching_xy(float x, float y, float from_x, float from_y, float approaching_time)
{
  /** distance to waypoint in x */
  float pw_x = x - stateGetPositionEnu_f()->x;
  /** distance to waypoint in y */
  float pw_y = y - stateGetPositionEnu_f()->y;

  if (approaching_time < 0.) {
    // fly after the destination waypoint
    float leg_x = x - from_x;
    float leg_y = y - from_y;
    float leg = sqrtf(Max(leg_x * leg_x + leg_y * leg_y, 1.));
    float exceed_dist = approaching_time * stateGetHorizontalSpeedNorm_f(); // negative value
    float scal_prod = (leg_x * pw_x + leg_y * pw_y) / leg;
    return (scal_prod < exceed_dist);
  } else {
    // fly close enough of the waypoint or cross it
    dist2_to_wp = pw_x * pw_x + pw_y * pw_y;
    float min_dist = approaching_time * stateGetHorizontalSpeedNorm_f();
    if (dist2_to_wp < min_dist * min_dist) {
      return true;
    }
    float scal_prod = (x - from_x) * pw_x + (y - from_y) * pw_y;
    return (scal_prod < 0.);
  }
}


/**
 *  \brief Computes \a desired_x, \a desired_y and \a desired_course.
 */
//static inline void fly_to_xy(float x, float y) {
void fly_to_xy(float x, float y)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  desired_x = x;
  desired_y = y;
  if (nav_mode == NAV_MODE_COURSE) {
    h_ctl_course_setpoint = atan2f(x - pos->x, y - pos->y);
    if (h_ctl_course_setpoint < 0.) {
      h_ctl_course_setpoint += 2 * M_PI;
    }
    lateral_mode = LATERAL_MODE_COURSE;
  } else {
    float diff = atan2f(x - pos->x, y - pos->y) - stateGetHorizontalSpeedDir_f();
    NormRadAngle(diff);
    BoundAbs(diff, M_PI / 2.);
    float s = sinf(diff);
    float speed = stateGetHorizontalSpeedNorm_f();
    h_ctl_roll_setpoint = atanf(2 * speed * speed * s * h_ctl_course_pgain / (CARROT * NOMINAL_AIRSPEED * 9.81));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
    lateral_mode = LATERAL_MODE_ROLL;
  }
}

/**
 *  \brief Computes the carrot position along the desired segment.
 */
void nav_route_xy(float last_wp_x, float last_wp_y, float wp_x, float wp_y)
{
  float leg_x = wp_x - last_wp_x;
  float leg_y = wp_y - last_wp_y;
  float leg2 = Max(leg_x * leg_x + leg_y * leg_y, 1.);
  nav_leg_progress = ((stateGetPositionEnu_f()->x - last_wp_x) * leg_x + (stateGetPositionEnu_f()->y - last_wp_y) *
                      leg_y) / leg2;
  nav_leg_length = sqrtf(leg2);

  /** distance of carrot (in meter) */
  float carrot = CARROT * NOMINAL_AIRSPEED;

  nav_carrot_leg_progress = nav_leg_progress + Max(carrot / nav_leg_length, 0.);
  nav_in_segment = true;
  nav_segment_x_1 = last_wp_x;
  nav_segment_y_1 = last_wp_y;
  nav_segment_x_2 = wp_x;
  nav_segment_y_2 = wp_y;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;

  fly_to_xy(last_wp_x + nav_carrot_leg_progress * leg_x + nav_shift * leg_y / nav_leg_length,
            last_wp_y + nav_carrot_leg_progress * leg_y - nav_shift * leg_x / nav_leg_length);
}

#include "subsystems/navigation/common_nav.c"

#ifndef FAILSAFE_HOME_RADIUS
#define FAILSAFE_HOME_RADIUS DEFAULT_CIRCLE_RADIUS
#endif

static void nav_set_altitude(void)
{
  static float last_nav_altitude;
  if (fabs(nav_altitude - last_nav_altitude) > 1.) {
    flight_altitude = nav_altitude;
    last_nav_altitude = nav_altitude;
  }
  v_ctl_altitude_setpoint = flight_altitude;
}

/** \brief Home mode navigation (circle around HOME) */
void nav_home(void)
{
  NavCircleWaypoint(WP_HOME, FAILSAFE_HOME_RADIUS);
  /** Nominal speed */
  nav_pitch = 0.;
  if (autopilot.launch) {
    v_ctl_mode = V_CTL_MODE_AUTO_ALT;
  } else {
    v_ctl_mode = V_CTL_MODE_AUTO_THROTTLE;
    v_ctl_throttle_setpoint = 0;
  }
  nav_altitude = ground_alt + HOME_MODE_HEIGHT;
  compute_dist2_to_home();
  dist2_to_wp = dist2_to_home;
  nav_set_altitude();
}

/**
 *  \brief Navigation main: call to the code generated from the XML flight
 * plan
 */
void nav_periodic_task(void)
{
  nav_survey_active = false;

  compute_dist2_to_home();
  dist2_to_wp = 0.;

  auto_nav(); /* From flight_plan.h */

  h_ctl_course_pre_bank = nav_in_circle ? circle_bank : 0;

#ifdef AGR_CLIMB
  if (v_ctl_mode == V_CTL_MODE_AUTO_CLIMB) {
    v_ctl_auto_throttle_submode =  V_CTL_AUTO_THROTTLE_STANDARD;
  }
#endif

  nav_set_altitude();
}

/**
 * \brief Periodic telemetry
 */
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_nav_ref(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_NAVIGATION_REF(trans, dev, AC_ID,
                               &nav_utm_east0, &nav_utm_north0, &nav_utm_zone0, &ground_alt);
}

static void send_nav(struct transport_tx *trans, struct link_device *dev)
{
  SEND_NAVIGATION(trans, dev);
}

static void DownlinkSendWp(struct transport_tx *trans, struct link_device *dev, uint8_t _wp)
{
  float x = nav_utm_east0 +  waypoints[_wp].x;
  float y = nav_utm_north0 + waypoints[_wp].y;
  pprz_msg_send_WP_MOVED(trans, dev, AC_ID, &_wp, &x, &y, &(waypoints[_wp].a), &nav_utm_zone0);
}

static void send_wp_moved(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i;
  i++;
  if (i >= nb_waypoint) { i = 0; }
  DownlinkSendWp(trans, dev, i);
}

void DownlinkSendWpNr(uint8_t _wp)
{
  if (_wp >= nb_waypoint) return;
  DownlinkSendWp(&(DefaultChannel).trans_tx, &(DefaultDevice).device, _wp);
}


static void send_circle(struct transport_tx *trans, struct link_device *dev)
{
  if (nav_in_circle) {
    pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                         &nav_circle_x, &nav_circle_y, &nav_circle_radius);
  }
}

static void send_segment(struct transport_tx *trans, struct link_device *dev)
{
  if (nav_in_segment) {
    pprz_msg_send_SEGMENT(trans, dev, AC_ID,
                          &nav_segment_x_1, &nav_segment_y_1, &nav_segment_x_2, &nav_segment_y_2);
  }
}

static void send_survey(struct transport_tx *trans, struct link_device *dev)
{
  if (nav_survey_active) {
    pprz_msg_send_SURVEY(trans, dev, AC_ID,
                         &nav_survey_east, &nav_survey_north, &nav_survey_west, &nav_survey_south);
  }
}
#endif

/**
 *  \brief Navigation Initialisation
 */
void nav_init(void)
{
  nav_block = 0;
  nav_stage = 0;
  ground_alt = GROUND_ALT;
  nav_glide_pitch_trim = NAV_GLIDE_PITCH_TRIM;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_survey_shift = 2 * DEFAULT_CIRCLE_RADIUS;
  nav_mode = NAV_MODE_COURSE;

  fp_pitch = 0.f;
  fp_throttle = 0.f;
  fp_climb = 0.f;

#ifdef NAV_GROUND_SPEED_PGAIN
  nav_ground_speed_pgain = ABS(NAV_GROUND_SPEED_PGAIN);
  nav_ground_speed_setpoint = NOMINAL_AIRSPEED;
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NAVIGATION_REF, send_nav_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NAVIGATION, send_nav);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WP_MOVED, send_wp_moved);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRCLE, send_circle);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SEGMENT, send_segment);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SURVEY, send_survey);
#endif

  // generated init function
  auto_nav_init();
}

/**
 *  \brief Failsafe navigation without position estimation
 *
 * Just set attitude and throttle to FAILSAFE values
 * to prevent the plane from crashing.
 */
void nav_without_gps(void)
{
  lateral_mode = LATERAL_MODE_ROLL;
  v_ctl_mode = V_CTL_MODE_AUTO_THROTTLE;

#ifdef SECTION_FAILSAFE
  h_ctl_roll_setpoint = FAILSAFE_DEFAULT_ROLL;
  nav_pitch = FAILSAFE_DEFAULT_PITCH;
  nav_throttle_setpoint = TRIM_UPPRZ((FAILSAFE_DEFAULT_THROTTLE) * MAX_PPRZ);
#else
  h_ctl_roll_setpoint = 0;
  nav_pitch = 0;
  nav_throttle_setpoint = TRIM_UPPRZ((V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE) * MAX_PPRZ);
#endif
}


/**************** 8 Navigation **********************************************/


enum eight_status { R1T, RT2, C2, R2T, RT1, C1 };

static enum eight_status eight_status;
void nav_eight_init(void)
{
  eight_status = C1;
}

/** Navigation along a figure 8. The cross center is defined by the waypoint
    [target], the center of one of the circles is defined by [c1]. Altitude is
    given by [target].
    The navigation goes through 6 states: C1 (circle around [c1]), R1T, RT2
    (route from circle 1 to circle 2 over [target]), C2 and R2T, RT1.
    If necessary, the [c1] waypoint is moved in the direction of [target]
    to be not far than [2*radius].
*/
void nav_eight(uint8_t target, uint8_t c1, float radius)
{
  float aradius = fabs(radius);
  float alt = waypoints[target].a;
  waypoints[c1].a = alt;

  float target_c1_x = waypoints[c1].x - waypoints[target].x;
  float target_c1_y = waypoints[c1].y - waypoints[target].y;
  float d = sqrtf(target_c1_x * target_c1_x + target_c1_y * target_c1_y);
  d = Max(d, 1.); /* To prevent a division by zero */

  /* Unit vector from target to c1 */
  float u_x = target_c1_x / d;
  float u_y = target_c1_y / d;

  /* Move [c1] closer if needed */
  if (d > 2 * aradius) {
    d = 2 * aradius;
    waypoints[c1].x = waypoints[target].x + d * u_x;
    waypoints[c1].y = waypoints[target].y + d * u_y;
  }

  /* The other center */
  struct point c2 = {
    waypoints[target].x - d * u_x,
    waypoints[target].y - d * u_y,
    alt
  };

  struct point c1_in = {
    waypoints[c1].x + radius * -u_y,
    waypoints[c1].y + radius * u_x,
    alt
  };
  struct point c1_out = {
    waypoints[c1].x - radius * -u_y,
    waypoints[c1].y - radius * u_x,
    alt
  };

  struct point c2_in = {
    c2.x + radius * -u_y,
    c2.y + radius * u_x,
    alt
  };
  struct point c2_out = {
    c2.x - radius * -u_y,
    c2.y - radius * u_x,
    alt
  };

  float qdr_out = M_PI - atan2f(u_y, u_x);
  if (radius < 0) {
    qdr_out += M_PI;
  }

  switch (eight_status) {
    case C1 :
      NavCircleWaypoint(c1, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out) - 10)) {
        eight_status = R1T;
        InitStage();
      }
      return;

    case R1T:
      nav_route_xy(c1_out.x, c1_out.y, c2_in.x, c2_in.y);
      if (nav_approaching_xy(waypoints[target].x, waypoints[target].y, c1_out.x, c1_out.y, 0)) {
        eight_status = RT2;
        InitStage();
      }
      return;

    case RT2:
      nav_route_xy(c1_out.x, c1_out.y, c2_in.x, c2_in.y);
      if (nav_approaching_xy(c2_in.x, c2_in.y, c1_out.x, c1_out.y, CARROT)) {
        eight_status = C2;
        InitStage();
      }
      return;

    case C2 :
      nav_circle_XY(c2.x, c2.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out) + 10)) {
        eight_status = R2T;
        InitStage();
      }
      return;

    case R2T:
      nav_route_xy(c2_out.x, c2_out.y, c1_in.x, c1_in.y);
      if (nav_approaching_xy(waypoints[target].x, waypoints[target].y, c2_out.x, c2_out.y, 0)) {
        eight_status = RT1;
        InitStage();
      }
      return;

    case RT1:
      nav_route_xy(c2_out.x, c2_out.y, c1_in.x, c1_in.y);
      if (nav_approaching_xy(c1_in.x, c1_in.y, c2_out.x, c2_out.y, CARROT)) {
        eight_status = C1;
        InitStage();
      }
      return;

    default:/* Should not occur !!! Doing nothing */
      return;
  } /* switch */
}

/************** Oval Navigation **********************************************/

/** Navigation along a figure O. One side leg is defined by waypoints [p1] and
    [p2].
    The navigation goes through 4 states: OC1 (half circle next to [p1]),
    OR21 (route [p2] to [p1], OC2 (half circle next to [p2]) and OR12
    (opposite leg).

    Initial state is the route along the desired segment (OC2).
*/

uint8_t nav_oval_count;

void nav_oval_init(void)
{
  oval_status = OC2;
  nav_oval_count = 0;
}

void nav_oval(uint8_t p1, uint8_t p2, float radius)
{
  radius = - radius; /* Historical error ? */

  float alt = waypoints[p1].a;
  waypoints[p2].a = alt;

  float p2_p1_x = waypoints[p1].x - waypoints[p2].x;
  float p2_p1_y = waypoints[p1].y - waypoints[p2].y;
  float d = sqrtf(p2_p1_x * p2_p1_x + p2_p1_y * p2_p1_y);

  /* Unit vector from p1 to p2 */
  float u_x = p2_p1_x / d;
  float u_y = p2_p1_y / d;

  /* The half circle centers and the other leg */
  struct point p1_center = { waypoints[p1].x + radius * -u_y,
           waypoints[p1].y + radius * u_x,
           alt
  };
  struct point p1_out = { waypoints[p1].x + 2 * radius * -u_y,
           waypoints[p1].y + 2 * radius * u_x,
           alt
  };

  struct point p2_in = { waypoints[p2].x + 2 * radius * -u_y,
           waypoints[p2].y + 2 * radius * u_x,
           alt
  };
  struct point p2_center = { waypoints[p2].x + radius * -u_y,
           waypoints[p2].y + radius * u_x,
           alt
  };

  float qdr_out_2 = M_PI - atan2f(u_y, u_x);
  float qdr_out_1 = qdr_out_2 + M_PI;
  if (radius < 0) {
    qdr_out_2 += M_PI;
    qdr_out_1 += M_PI;
  }
  float qdr_anticipation = (radius > 0 ? -15 : 15);

  switch (oval_status) {
    case OC1 :
      nav_circle_XY(p1_center.x, p1_center.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_1) - qdr_anticipation)) {
        oval_status = OR12;
        InitStage();
        LINE_START_FUNCTION;
      }
      return;

    case OR12:
      nav_route_xy(p1_out.x, p1_out.y, p2_in.x, p2_in.y);
      if (nav_approaching_xy(p2_in.x, p2_in.y, p1_out.x, p1_out.y, CARROT)) {
        oval_status = OC2;
        nav_oval_count++;
        InitStage();
        LINE_STOP_FUNCTION;
      }
      return;

    case OC2 :
      nav_circle_XY(p2_center.x, p2_center.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2) - qdr_anticipation)) {
        oval_status = OR21;
        InitStage();
        LINE_START_FUNCTION;
      }
      return;

    case OR21:
      nav_route_xy(waypoints[p2].x, waypoints[p2].y, waypoints[p1].x, waypoints[p1].y);
      if (nav_approaching_xy(waypoints[p1].x, waypoints[p1].y, waypoints[p2].x, waypoints[p2].y, CARROT)) {
        oval_status = OC1;
        InitStage();
        LINE_STOP_FUNCTION;
      }
      return;

    default: /* Should not occur !!! Doing nothing */
      return;
  }
}

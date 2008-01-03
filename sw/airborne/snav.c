/* Smooth navigation to wp_a along an arc (around wp_cd), a
   segment (from wp_rd to wp_ta) and a second arc (around wp_ca) */

#include <math.h>
#include "snav.h"
#include "estimator.h"
#include "nav.h"
#include "gps.h"

#define Sign(_x) ((_x) > 0 ? 1 : (-1))

static struct point wp_cd, wp_td, wp_ca, wp_ta;
static float d_radius, a_radius;
static float qdr_td, qdr_a;
static uint8_t wp_a;
float snav_desired_tow; /* time of week, s */
static float u_a_ca_x, u_a_ca_y;

/* D is the current position */
bool_t snav_init(uint8_t a, float desired_course_rad, float radius) {
  wp_a = a;
  radius = fabs(radius);

  float da_x = WaypointX(wp_a) - estimator_x;
  float da_y = WaypointY(wp_a) - estimator_y;

  /* D_CD orthogonal to current course, CD on the side of A */
  float u_x = cos(M_PI_2 - estimator_hspeed_dir);
  float u_y = sin(M_PI_2 - estimator_hspeed_dir);
  d_radius = - Sign(u_x*da_y - u_y*da_x) * radius;
  wp_cd.x = estimator_x + d_radius * u_y;
  wp_cd.y = estimator_y - d_radius * u_x;
  wp_cd.a = WaypointAlt(wp_a);

  /* A_CA orthogonal to desired course, CA on the side of D */
  float desired_u_x = cos(M_PI_2 - desired_course_rad);
  float desired_u_y = sin(M_PI_2 - desired_course_rad);
  a_radius = Sign(desired_u_x*da_y - desired_u_y*da_x) * radius;
  u_a_ca_x = desired_u_y;
  u_a_ca_y = - desired_u_x;
  wp_ca.x = WaypointX(wp_a) + a_radius * u_a_ca_x;
  wp_ca.y = WaypointY(wp_a) + a_radius * u_a_ca_y;
  wp_ca.a = WaypointAlt(wp_a);

  /* Unit vector along CD-CA */
  u_x = wp_ca.x - wp_cd.x;
  u_y = wp_ca.y - wp_cd.y;
  float cd_ca = sqrt(u_x*u_x+u_y*u_y);

  /* If it is too close in reverse direction, set CA on the other side */
  if (a_radius * d_radius < 0 && cd_ca < 2 * radius) {
    a_radius = -a_radius;
    wp_ca.x = WaypointX(wp_a) + a_radius * u_a_ca_x;
    wp_ca.y = WaypointY(wp_a) + a_radius * u_a_ca_y;
    u_x = wp_ca.x - wp_cd.x;
    u_y = wp_ca.y - wp_cd.y;
    cd_ca = sqrt(u_x*u_x+u_y*u_y);
  }
  
  u_x /= cd_ca;
  u_y /= cd_ca;
  
  if (a_radius * d_radius > 0) {
    /* Both arcs are in the same direction */
    /* CD_TD orthogonal to CD_CA */
    wp_td.x = wp_cd.x - d_radius * u_y;
    wp_td.y = wp_cd.y + d_radius * u_x;
    
    /* CA_TA also orthogonal to CD_CA */
    wp_ta.x = wp_ca.x - a_radius * u_y;
    wp_ta.y = wp_ca.y + a_radius * u_x;
  } else {
    /* Arcs are in reverse directions: trigonemetric puzzle :-) */
    float alpha = atan2(u_y, u_x) + acos(d_radius/(cd_ca/2));
    wp_td.x = wp_cd.x + d_radius * cos(alpha);
    wp_td.y = wp_cd.y + d_radius * sin(alpha);
    
    wp_ta.x = wp_ca.x + a_radius * cos(alpha);
    wp_ta.y = wp_ca.y + a_radius * sin(alpha);
  }
  qdr_td = M_PI_2 - atan2(wp_td.y-wp_cd.y, wp_td.x-wp_cd.x);
  qdr_a = M_PI_2 - atan2(WaypointY(wp_a)-wp_ca.y, WaypointX(wp_a)-wp_ca.x);
  wp_td.a = wp_cd.a;
  wp_ta.a = wp_ca.a;
  
  return FALSE;
}


bool_t snav_circle1(void) {
  /* circle around CD until QDR_TD */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);
  nav_circle_XY(wp_cd.x, wp_cd.y, d_radius);
  return(! NavQdrCloseTo(DegOfRad(qdr_td)));
}

bool_t snav_route(void) {
  /* Straight route from TD to TA */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);
  nav_route_xy(wp_td.x, wp_td.y, wp_ta.x, wp_ta.y);
  
  return (! nav_approaching_xy(wp_ta.x, wp_ta.y, wp_td.x, wp_td.y, CARROT));
}

bool_t snav_circle2(void) {
  /* circle around CA until QDR_A */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);
  nav_circle_XY(wp_ca.x, wp_ca.y, a_radius);

  return(! NavQdrCloseTo(DegOfRad(qdr_a)));
}

/* Adjusting a circle around CA, tangent in A, to end at snav_desired_tow */
bool_t snav_on_time(float nominal_radius) {
  nominal_radius = fabs(nominal_radius);

  float current_qdr = M_PI_2 - atan2(estimator_y-wp_ca.y, estimator_x-wp_ca.x);
  float remaining_angle = Sign(a_radius)*(qdr_a - current_qdr);
  if (remaining_angle <= 0.)
    remaining_angle += 2 * M_PI;
  float remaining_time = snav_desired_tow - gps_itow / 1000.;

  /* Use the nominal airspeed if the estimated on is not realistic */
  float airspeed = estimator_airspeed;
  if (estimator_airspeed < NOMINAL_AIRSPEED / 2. ||
      estimator_airspeed > 2.* NOMINAL_AIRSPEED)
    estimator_airspeed = NOMINAL_AIRSPEED;

  /* Radius size to finish in one single circle */
  float radius = (remaining_time * airspeed) / remaining_angle;
  if (radius > 2. * nominal_radius)
    radius = nominal_radius;
  
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);

  radius *= Sign(a_radius);
  wp_ca.x = WaypointX(wp_a) + radius * u_a_ca_x;
  wp_ca.y = WaypointY(wp_a) + radius * u_a_ca_y;
  nav_circle_XY(wp_ca.x, wp_ca.y, radius);
  
  /* Stay in this mode until the end of time */
  return(remaining_time > 0);
}

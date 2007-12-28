/* Smooth navigation to wp_a along an arc (around wp_cd), a
   segment (from wp_rd to wp_ta) and a second arc (around wp_ca) */

#include <math.h>
#include "snav.h"
#include "estimator.h"
#include "nav.h"

#define Sign(_x) ((_x) > 0 ? -1 : 1)

static struct point wp_cd, wp_td, wp_ca, wp_ta;
float d_radius, a_radius;
float qdr_td, qdr_a;

/* D is the current position */
bool_t snav_init(uint8_t wp_a, float desired_course_rad) {
  float da_x = WaypointX(wp_a) - estimator_x;
  float da_y = WaypointY(wp_a) - estimator_y;

  /* D_CD orthogonal to current course, CD on the side of A */
  float u_x = cos(M_PI_2 - estimator_hspeed_dir);
  float u_y = sin(M_PI_2 - estimator_hspeed_dir);
  d_radius = Sign(u_x*da_y - u_y*da_x) * DEFAULT_CIRCLE_RADIUS;
  wp_cd.x = estimator_x + d_radius * u_y;
  wp_cd.y = estimator_y - d_radius * u_x;
  wp_cd.a = WaypointAlt(wp_a);

  /* A_CA orthogonal to desired course, CA on the side of D */
  u_x = cos(M_PI_2 - desired_course_rad);
  u_y = sin(M_PI_2 - desired_course_rad);
  a_radius = - Sign(u_x*da_y - u_y*da_x) * DEFAULT_CIRCLE_RADIUS;
  wp_ca.x = WaypointX(wp_a) + a_radius * u_y;
  wp_ca.y = WaypointY(wp_a) - a_radius * u_x;
  wp_ca.a = WaypointAlt(wp_a);

  /* Unit vector along CD-CA */
  u_x = wp_ca.x - wp_cd.x;
  u_y = wp_ca.y - wp_cd.y;
  float cd_da = sqrt(u_x*u_x+u_y*u_y);
  u_x /= cd_da;
  u_y /= cd_da;

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
    float alpha = atan2(u_y, u_x) + acos(d_radius/(cd_da/2));
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

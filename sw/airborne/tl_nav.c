#include <math.h>

#include "tl_nav.h"
#include "tl_estimator.h"
#include "tl_control.h"
#include "gps.h"

float tl_nav_goto_h_pgain;
float tl_nav_goto_h_dgain;

#define TL_NAV_GOTO_MAX_PHI_COMMAND   RadOfDeg(30.)
#define TL_NAV_GOTO_MAX_THETA_COMMAND RadOfDeg(30.)

static void nav_goto_waypoint(uint8_t wp);

#define NavGotoWaypoint(_wp) { nav_goto_waypoint(_wp); }
#define NavVerticalAutoThrottleMode(_foo) {}
#define NavVerticalAltitudeMode(_foo, _bar) {}

static void nav_home(void) {
  kill_throttle = TRUE;
}

#define NAV_C
#include "flight_plan.h"

void tl_nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
  tl_nav_goto_h_pgain = NAV_GOTO_H_PGAIN;
  tl_nav_goto_h_dgain = NAV_GOTO_H_DGAIN;
}

void tl_nav_periodic_task(void) {
  compute_dist2_to_home();
  auto_nav();
}

void nav_init_stage( void ) {
  stage_time = 0;
}

static void fly_to_xy(float x_sp, float y_sp) {
  /* get a position error vector              */
  float x_error = estimator_x - x_sp;
  float y_error = estimator_y - y_sp;

  /* Normalize the error */
  float norm_error = sqrt(x_error*x_error+ y_error*y_error);
  x_error /= norm_error;
  y_error /= norm_error;

  /* convert to body frame */
  float y_unit_err_body, x_unit_err_body;
  tl_estimator_to_body_frame(x_error, y_error, &x_unit_err_body, &y_unit_err_body);

  /* Compute command */
  float tl_nav_goto_phi_command = - tl_nav_goto_h_pgain * y_unit_err_body * norm_error +                            - tl_nav_goto_h_dgain *  tl_estimator_v;
  float tl_nav_goto_theta_command =    tl_nav_goto_h_pgain * x_unit_err_body * norm_error + tl_nav_goto_h_dgain *  tl_estimator_u;
  
  Bound(tl_nav_goto_phi_command, -TL_NAV_GOTO_MAX_PHI_COMMAND, TL_NAV_GOTO_MAX_PHI_COMMAND);
  Bound(tl_nav_goto_theta_command, -TL_NAV_GOTO_MAX_THETA_COMMAND, TL_NAV_GOTO_MAX_THETA_COMMAND);

  tl_control_attitude_phi_sp = tl_nav_goto_phi_command;
  tl_control_attitude_theta_sp = tl_nav_goto_theta_command;
}

static void nav_goto_waypoint(uint8_t wp) {
  fly_to_xy(waypoints[wp].x, waypoints[wp].y);
}

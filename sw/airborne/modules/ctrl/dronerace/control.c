
#include "std.h"


#include "control.h"
#include "filter.h"
#include "flightplan.h"
#include "ransac.h"
#include <math.h>

// Variables
struct dronerace_control_struct dr_control;

// Settings


// Slow speed
#define CTRL_MAX_SPEED  3.5             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(17)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(15)    // rad
#define CTRL_MAX_R      RadOfDeg(70)    // rad/sec

/*
// Max speed for bebop
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(35)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(35)    // rad
#define CTRL_MAX_R      RadOfDeg(45)    // rad/sec
*/

/*
// Race drone
#define CTRL_MAX_SPEED  10              // m/s
#define CTRL_MAX_PITCH  RadOfDeg(45)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(45)    // rad
#define CTRL_MAX_R      RadOfDeg(180)   // rad/sec
*/


void control_reset(void)
{
  // Reset flight plan logic
  flightplan_reset();

  // Reset own variables
  dr_control.psi_ref = 0;
  dr_control.psi_cmd = 0;
}

static float angle180(float r)
{
  if (r < RadOfDeg(-180))
  {
    r += RadOfDeg(360.0f);
  }
  else if (r > RadOfDeg(180.0f))
  {
    r -= RadOfDeg(360.0f);
  }

  return r;
}

void control_run(float dt)
{
  float psi, vxcmd, vycmd, r_cmd, ax, ay;
  // Propagate the flightplan
  flightplan_run();

  // Variables
  psi = dr_state.psi;

  // Heading controller
  r_cmd = dr_fp.psi_set - dr_control.psi_ref;

  // Find shortest turn
  r_cmd = angle180(r_cmd);

  // Apply rate limit
  Bound(r_cmd, -CTRL_MAX_R, CTRL_MAX_R);
  dr_control.psi_ref += r_cmd * dt;

  // Position error to Speed
  // TODO: interestingly, we don't use the velocity correction for control: t_fit * dr_ransac.corr_vx
  if(gates[dr_fp.gate_nr].psi == RadOfDeg(0.0) || gates[dr_fp.gate_nr].psi == RadOfDeg(180))
  {
    vxcmd = (dr_fp.x_set - (dr_state.x + dr_ransac.corr_x)) * P_FORWARD - dr_state.vx * D_FORWARD;
    vycmd = (dr_fp.y_set - (dr_state.y+dr_ransac.corr_y)) * P_LATERAL - dr_state.vy * D_LATERAL;
  }
  else
  {
    vxcmd = (dr_fp.x_set - (dr_state.x + dr_ransac.corr_x)) * P_LATERAL- dr_state.vx * D_LATERAL;
    vycmd = (dr_fp.y_set - (dr_state.y+dr_ransac.corr_y)) * P_FORWARD- dr_state.vy * D_FORWARD;
  }

  if(!waypoints_dr[dr_fp.gate_nr].brake) {
      vxcmd += 10.0f * cosf(waypoints_dr[dr_fp.gate_nr].psi);
      vycmd += 10.0f * sinf(waypoints_dr[dr_fp.gate_nr].psi);
  }

  Bound(vxcmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);
  Bound(vycmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);

  vxcmd *= dr_fp.gate_speed;
  vycmd *= dr_fp.gate_speed;

  // Speed to Attitude
  ax = (vxcmd - dr_state.vx) * 1.0f + vxcmd * RadOfDeg(10.0f) / 3.0f;
  ay = (vycmd - dr_state.vy) * 1.0f + vycmd * RadOfDeg(10.0f) / 3.0f;

  Bound(ax, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);
  Bound(ay, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);

  dr_control.phi_cmd   = - sinf(psi) * ax + cosf(psi) * ay;
  dr_control.theta_cmd = - cosf(psi) * ax - sinf(psi) * ay;
  dr_control.psi_cmd   = dr_control.psi_ref;
  dr_control.z_cmd   = dr_fp.z_set;

}


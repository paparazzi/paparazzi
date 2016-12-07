/*
 * Copyright (C) 2016 Hector Garcia de Marina
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

#include <math.h>
#include "std.h"

#include "modules/guidance/gvf/gvf.h"
#include "modules/guidance/gvf/trajectories/gvf_ellipse.h"
#include "modules/guidance/gvf/trajectories/gvf_line.h"
#include "modules/guidance/gvf/trajectories/gvf_sin.h"

#include "subsystems/navigation/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/autopilot.h"

// Control
gvf_con gvf_control;

// Trajectory
gvf_tra gvf_trajectory;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{
  // Do not know whether is a good idea to do this check here or to include
  // this plen in gvf_trajectory
  int plen;

  switch (gvf_trajectory.type) {
    case LINE:
      plen = 3;
      break;
    case ELLIPSE:
      plen = 5;
      break;
    case SIN:
      plen = 6;
      break;
    default:
      plen = 0;
      break;
  }

  uint8_t traj_type = (uint8_t)gvf_trajectory.type;

  pprz_msg_send_GVF(trans, dev, AC_ID, &gvf_control.error, &traj_type,
                    &gvf_control.s, plen, gvf_trajectory.p);
}

static void send_circle(struct transport_tx *trans, struct link_device *dev)
{
  if (gvf_trajectory.type == ELLIPSE &&
      (gvf_trajectory.p[2] == gvf_trajectory.p[3])) {
    pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                         &gvf_trajectory.p[0], &gvf_trajectory.p[1],
                         &gvf_trajectory.p[2]);
  }
}

#endif

void gvf_init(void)
{
  gvf_control.ke = 1;
  gvf_control.kn = 1;
  gvf_control.s = 1;
  gvf_trajectory.type = NONE;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRCLE,
                              send_circle);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_control_2D(float ke, float kn, float e,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float ground_speed = stateGetHorizontalSpeedNorm_f();
  float course = stateGetHorizontalSpeedDir_f();
  float px_dot = ground_speed * sinf(course);
  float py_dot = ground_speed * cosf(course);
  int s = gvf_control.s;

  // gradient Phi
  float nx = grad->nx;
  float ny = grad->ny;

  // tangent to Phi
  float tx = s * grad->ny;
  float ty = -s * grad->nx;

  // Hessian
  float H11 = hess->H11;
  float H12 = hess->H12;
  float H21 = hess->H21;
  float H22 = hess->H22;

  // Calculation of the desired angular velocity in the vector field
  float pdx_dot = tx - ke * e * nx;
  float pdy_dot = ty - ke * e * ny;

  float norm_pd_dot = sqrtf(pdx_dot * pdx_dot + pdy_dot * pdy_dot);
  float md_x = pdx_dot / norm_pd_dot;
  float md_y = pdy_dot / norm_pd_dot;

  float Apd_dot_dot_x = -ke * (nx * px_dot + ny * py_dot) * nx;
  float Apd_dot_dot_y = -ke * (nx * px_dot + ny * py_dot) * ny;

  float Bpd_dot_dot_x = ((-ke * e * H11) + s * H21) * px_dot
                        + ((-ke * e * H12) + s * H22) * py_dot;
  float Bpd_dot_dot_y = -(s * H11 + (ke * e * H21)) * px_dot
                        - (s * H12 + (ke * e * H22)) * py_dot;

  float pd_dot_dot_x = Apd_dot_dot_x + Bpd_dot_dot_x;
  float pd_dot_dot_y = Apd_dot_dot_y + Bpd_dot_dot_y;

  float md_dot_const = -(md_x * pd_dot_dot_y - md_y * pd_dot_dot_x)
                       / norm_pd_dot;

  float md_dot_x =  md_y * md_dot_const;
  float md_dot_y = -md_x * md_dot_const;

  float omega_d = -(md_dot_x * md_y - md_dot_y * md_x);

  float mr_x = sinf(course);
  float mr_y = cosf(course);

  float omega = omega_d + kn * (mr_x * md_y - mr_y * md_x);

  // Coordinated turn
  h_ctl_roll_setpoint =
    -atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

  lateral_mode = LATERAL_MODE_ROLL;
}

void gvf_set_gains(float ke, float kn)
{
  gvf_control.ke = ke;
  gvf_control.kn = kn;
}

void gvf_set_direction(int8_t s)
{
  gvf_control.s = s;
}

// STRAIGHT LINE

void gvf_line(float a, float b, float alpha)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 0;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = alpha;

  gvf_line_info(&e, &grad_line, &Hess_line);
  gvf_control_2D(1e-2 * gvf_control.ke, gvf_control.kn, e, &grad_line, &Hess_line);

  gvf_control.error = e;
}

bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  float x1 = waypoints[wp1].x;
  float y1 = waypoints[wp1].y;
  float x2 = waypoints[wp2].x;
  float y2 = waypoints[wp2].y;

  float zx = x1 - x2;
  float zy = y1 - y2;

  float alpha = atanf(zy / zx);

  gvf_line(x1, y1, alpha);

  return true;
}

bool gvf_line_wp_heading(uint8_t wp, float alpha)
{
  alpha = alpha * M_PI / 180;

  float a = waypoints[wp].x;
  float b = waypoints[wp].y;

  gvf_line(a, b, alpha);

  return true;
}

// ELLIPSE
bool gvf_ellipse(uint8_t wp, float a, float b, float alpha)
{
  float e;
  struct gvf_grad grad_ellipse;
  struct gvf_Hess Hess_ellipse;

  gvf_trajectory.type = 1;
  gvf_trajectory.p[0] = waypoints[wp].x;
  gvf_trajectory.p[1] = waypoints[wp].y;
  gvf_trajectory.p[2] = a;
  gvf_trajectory.p[3] = b;
  gvf_trajectory.p[4] = alpha;

  // SAFE MODE
  if (a < 1 || b < 1) {
    gvf_trajectory.p[2] = 60;
    gvf_trajectory.p[3] = 60;
  }

  gvf_ellipse_info(&e, &grad_ellipse, &Hess_ellipse);
  gvf_control_2D(gvf_control.ke, gvf_control.kn, e, &grad_ellipse, &Hess_ellipse);

  gvf_control.error = e;

  return true;
}

// SINUSOIDAL (if w = 0 and off = 0, then we just have the straight line case)

void gvf_sin(float a, float b, float alpha, float w, float off, float A)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 2;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = alpha;
  gvf_trajectory.p[3] = w;
  gvf_trajectory.p[4] = off;
  gvf_trajectory.p[5] = A;

  gvf_sin_info(&e, &grad_line, &Hess_line);
  gvf_control_2D(1e-2 * gvf_control.ke, gvf_control.kn, e, &grad_line, &Hess_line);

  gvf_control.error = e;
}

bool gvf_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off, float A)
{
  w = 2 * M_PI * w;

  float x1 = waypoints[wp1].x;
  float y1 = waypoints[wp1].y;
  float x2 = waypoints[wp2].x;
  float y2 = waypoints[wp2].y;

  float zx = x1 - x2;
  float zy = y1 - y2;

  float alpha = atanf(zy / zx);

  gvf_sin(x1, y1, alpha, w, off, A);

  return true;
}

bool gvf_sin_wp_heading(uint8_t wp, float alpha, float w, float off, float A)
{
  w = 2 * M_PI * w;
  alpha = alpha * M_PI / 180;

  float x = waypoints[wp].x;
  float y = waypoints[wp].y;

  gvf_sin(x, y, alpha, w, off, A);

  return true;
}


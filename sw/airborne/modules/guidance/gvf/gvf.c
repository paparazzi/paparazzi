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

#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "autopilot.h"

// Control
gvf_con gvf_control;

// Trajectory
gvf_tra gvf_trajectory;
gvf_seg gvf_segment;

// Time variables to check if GVF is active
uint32_t gvf_t0 = 0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
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
      plen = 1;
      break;
  }

  uint8_t traj_type = (uint8_t)gvf_trajectory.type;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_t0;

  if (delta_T < 200)
    pprz_msg_send_GVF(trans, dev, AC_ID, &gvf_control.error, &traj_type,
                      &gvf_control.s, &gvf_control.ke, plen, gvf_trajectory.p);
}

static void send_circle(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_t0;

  if (delta_T < 200)
    if (gvf_trajectory.type == ELLIPSE &&
        (gvf_trajectory.p[2] == gvf_trajectory.p[3])) {
      pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                           &gvf_trajectory.p[0], &gvf_trajectory.p[1],
                           &gvf_trajectory.p[2]);
    }
}

static void send_segment(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_t0;

  if (delta_T < 200)
    if (gvf_trajectory.type == LINE && gvf_segment.seg == 1) {
      pprz_msg_send_SEGMENT(trans, dev, AC_ID,
                            &gvf_segment.x1, &gvf_segment.y1,
                            &gvf_segment.x2, &gvf_segment.y2);
    }
}

#endif

static int out_of_segment_area(float x1, float y1, float x2, float y2, float d1, float d2)
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atan2f(zy, zx);

  float cosa = cosf(-alpha);
  float sina = sinf(-alpha);

  float pxr = px * cosa - py * sina;
  float zxr = zx * cosa - zy * sina;

  int s = 0;

  if (pxr < -d1) {
    s = 1;
  } else if (pxr > (zxr + d2)) {
    s = -1;
  }

  if (zy < 0) {
    s *= -1;
  }

  return s;
}

void gvf_init(void)
{
  gvf_control.ke = 1;
  gvf_control.kn = 1;
  gvf_control.s = 1;
  gvf_trajectory.type = NONE;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRCLE, send_circle);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SEGMENT, send_segment);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_control_2D(float ke, float kn, float e,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{
  gvf_t0 = get_sys_time_msec();

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
  if (autopilot_get_mode() == AP_MODE_AUTO2) {
    h_ctl_roll_setpoint =
      -atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

    lateral_mode = LATERAL_MODE_ROLL;
  }
}

void gvf_set_direction(int8_t s)
{
  gvf_control.s = s;
}

// STRAIGHT LINE

static void gvf_line(float a, float b, float heading)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 0;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = heading;

  gvf_line_info(&e, &grad_line, &Hess_line);
  gvf_control.ke = gvf_line_par.ke;
  gvf_control_2D(1e-2 * gvf_line_par.ke, gvf_line_par.kn, e, &grad_line, &Hess_line);

  gvf_control.error = e;

  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  gvf_segment.seg = 0;
}

bool gvf_line_XY_heading(float a, float b, float heading)
{
  gvf_set_direction(1);
  gvf_line(a, b, heading);
  return true;
}

bool gvf_line_XY1_XY2(float x1, float y1, float x2, float y2)
{
  float zx = x2 - x1;
  float zy = y2 - y1;

  gvf_line_XY_heading(x1, y1, atan2f(zx, zy));

  horizontal_mode = HORIZONTAL_MODE_ROUTE;
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  float x1 = waypoints[wp1].x;
  float y1 = waypoints[wp1].y;
  float x2 = waypoints[wp2].x;
  float y2 = waypoints[wp2].y;

  return gvf_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2)
{
  int s = out_of_segment_area(x1, y1, x2, y2, d1, d2);
  if (s != 0) {
    gvf_control.s = s;
  }

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atanf(zx / zy);

  gvf_line(x1, y1, alpha);

  horizontal_mode = HORIZONTAL_MODE_ROUTE;
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2)
{
  float x1 = waypoints[wp1].x;
  float y1 = waypoints[wp1].y;
  float x2 = waypoints[wp2].x;
  float y2 = waypoints[wp2].y;

  return gvf_segment_loop_XY1_XY2(x1, y1, x2, y2, d1, d2);
}

bool gvf_segment_XY1_XY2(float x1, float y1, float x2, float y2)
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;

  float beta = atan2f(zy, zx);
  float cosb = cosf(-beta);
  float sinb = sinf(-beta);
  float zxr = zx * cosb - zy * sinb;
  float pxr = px * cosb - py * sinb;

  if ((zxr > 0 && pxr > zxr) || (zxr < 0 && pxr < zxr)) {
    return false;
  }

  return gvf_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_segment_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  float x1 = waypoints[wp1].x;
  float y1 = waypoints[wp1].y;
  float x2 = waypoints[wp2].x;
  float y2 = waypoints[wp2].y;

  return gvf_segment_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_line_wp_heading(uint8_t wp, float heading)
{
  heading = heading * M_PI / 180;

  float a = waypoints[wp].x;
  float b = waypoints[wp].y;

  return gvf_line_XY_heading(a, b, heading);
}

// ELLIPSE

bool gvf_ellipse_XY(float x, float y, float a, float b, float alpha)
{
  float e;
  struct gvf_grad grad_ellipse;
  struct gvf_Hess Hess_ellipse;

  gvf_trajectory.type = 1;
  gvf_trajectory.p[0] = x;
  gvf_trajectory.p[1] = y;
  gvf_trajectory.p[2] = a;
  gvf_trajectory.p[3] = b;
  gvf_trajectory.p[4] = alpha;

  // SAFE MODE
  if (a < 1 || b < 1) {
    gvf_trajectory.p[2] = 60;
    gvf_trajectory.p[3] = 60;
  }

  if (gvf_trajectory.p[2] == gvf_trajectory.p[3]) {
    horizontal_mode = HORIZONTAL_MODE_CIRCLE;
  } else {
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  }

  gvf_ellipse_info(&e, &grad_ellipse, &Hess_ellipse);
  gvf_control.ke = gvf_ellipse_par.ke;
  gvf_control_2D(gvf_ellipse_par.ke, gvf_ellipse_par.kn,
                 e, &grad_ellipse, &Hess_ellipse);

  gvf_control.error = e;

  return true;
}


bool gvf_ellipse_wp(uint8_t wp, float a, float b, float alpha)
{
  gvf_ellipse_XY(waypoints[wp].x,  waypoints[wp].y, a, b, alpha);
  return true;
}

// SINUSOIDAL (if w = 0 and off = 0, then we just have the straight line case)

bool gvf_sin_XY_alpha(float a, float b, float alpha, float w, float off, float A)
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
  gvf_control.ke = gvf_sin_par.ke;
  gvf_control_2D(1e-2 * gvf_sin_par.ke, gvf_sin_par.kn, e, &grad_line, &Hess_line);

  gvf_control.error = e;

  return true;
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

  gvf_sin_XY_alpha(x1, y1, alpha, w, off, A);

  return true;
}

bool gvf_sin_wp_alpha(uint8_t wp, float alpha, float w, float off, float A)
{
  w = 2 * M_PI * w;
  alpha = alpha * M_PI / 180;

  float x = waypoints[wp].x;
  float y = waypoints[wp].y;

  gvf_sin_XY_alpha(x, y, alpha, w, off, A);

  return true;
}


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

#include "gvf.h"
#include "autopilot.h"

// Control
gvf_con gvf_control;

// Telemetry
gvf_tel gvf_telemetry = {0};

// Time variables to check if GVF is active
static uint32_t gvf_t0 = 0;

/** TELEMETRY -------------------------------------------------------------- **/

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t traj_type = (uint8_t)gvf_trajectory.type;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_t0;

  if (delta_T < 200) {
    pprz_msg_send_GVF(
      trans, dev, AC_ID, 
      &gvf_control.error,
      &traj_type, 
      &gvf_control.s, 
      &gvf_control.ke, 
      gvf_trajectory.p_len, gvf_trajectory.p,
      &gvf_control.error_n,
      &gvf_telemetry.n_norm,
      &gvf_telemetry.t_norm,
      &gvf_telemetry.omega_d,
      &gvf_telemetry.omega);

#if GVF_OCAML_GCS
    if (gvf_trajectory.type == ELLIPSE &&
        ((int)gvf_trajectory.p[2] == (int)gvf_trajectory.p[3])) {
      pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                           &gvf_trajectory.p[0], &gvf_trajectory.p[1],
                           &gvf_trajectory.p[2]);
    }

    if (gvf_trajectory.type == LINE && gvf_segment.seg == 1) {
      pprz_msg_send_SEGMENT(trans, dev, AC_ID,
                            &gvf_segment.x1, &gvf_segment.y1,
                            &gvf_segment.x2, &gvf_segment.y2);
    }
#endif // GVF_OCAML_GCS

  }
}

#endif // PERIODIC_TELEMETRY

/** ------------------------------------------------------------------------ **/

void gvf_init(void)
{
  gvf_control.ke = 1;
  gvf_control.kn = 1;
  gvf_control.s = 1;
  gvf_control.speed = 1; // Rotorcraft only (for now)
  gvf_control.align = false; // Rotorcraft only
  gvf_trajectory.type = NONE;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_control_2D(float ke, float kn __attribute__((unused)), float e,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{
  gvf_t0 = get_sys_time_msec();
  
  gvf_low_level_getState();
  float course __attribute__((unused)) = gvf_c_state.course;
  float px_dot __attribute__((unused)) = gvf_c_state.px_dot;
  float py_dot __attribute__((unused)) = gvf_c_state.py_dot;

  int s = gvf_control.s;

  // gradient Phi
  float nx = grad->nx;
  float ny = grad->ny;

  // tangent to Phi
  float tx = s * grad->ny;
  float ty = -s * grad->nx;

  // Hessian
  float H11 __attribute__((unused)) = hess->H11;
  float H12 __attribute__((unused)) = hess->H12;
  float H21 __attribute__((unused)) = hess->H21;
  float H22 __attribute__((unused)) = hess->H22;

  // Calculation of the desired angular velocity in the vector field
  float pdx_dot = tx - ke * e * nx;
  float pdy_dot = ty - ke * e * ny;

  float norm_pd_dot = sqrtf(pdx_dot * pdx_dot + pdy_dot * pdy_dot);
  float md_x = pdx_dot / norm_pd_dot;
  float md_y = pdy_dot / norm_pd_dot;

  #if defined(ROTORCRAFT_FIRMWARE) // TODO: new accel GVF module just for rotorcrafts

  (void)(course);

  // Use accel based control. Not recommended as of current implementation
  #if defined(GVF_ROTORCRAFT_USE_ACCEL)

  // Set nav for command
  // Use parameter kn as the speed command
  nav.speed.x = md_x * kn;
  nav.speed.y = md_y * kn;

  // Acceleration induced by the field with speed set to kn (!WIP!)
  #warning "Using GVF for rotorcraft is still experimental, proceed with caution"
  float n_norm = sqrtf(nx*nx+ny*ny);
  float hess_px_dot = px_dot * H11 + py_dot * H12;
  float hess_py_dot = px_dot * H21 + py_dot * H22;

  float hess_pdx_dot = pdx_dot * H11 + pdy_dot * H12;
  float hess_pdy_dot = pdx_dot * H21 + pdy_dot * H22;

  float curvature_correction = tx * hess_px_dot + ty * hess_py_dot / (n_norm * n_norm);
  float accel_correction_x = kn * hess_py_dot / n_norm;
  float accel_correction_y = - kn * hess_px_dot / n_norm;
  float accel_cmd_x = accel_correction_x + px_dot * curvature_correction;
  float accel_cmd_y = accel_correction_y + py_dot * curvature_correction;

  float speed_cmd_x = kn*tx / n_norm - ke * e * nx / (n_norm);
  float speed_cmd_y = kn*ty / n_norm - ke * e * ny / (n_norm);

  // TODO: don't change nav struct directly
  nav.accel.x = accel_cmd_x + (speed_cmd_x - px_dot);
  nav.accel.y = accel_cmd_y + (speed_cmd_y - py_dot);
  nav.heading = atan2f(md_x,md_y);

  #else // SPEED_BASED_GVF // TODO: move to low level control??

  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;

  // Speed-based control, acceleration based control not implemented yet
  nav.speed.x = gvf_control.speed * md_x;
  nav.speed.y = gvf_control.speed * md_y;

  // Optionally align heading with trajectory
  if (gvf_control.align) 
  {
    nav.heading = atan2f(md_x, md_y);
  }
  
  #endif

  #else // FIXEDWING & ROVER FIRMWARE

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

  float e_n = (mr_x * md_y - mr_y * md_x);
  float omega = omega_d + kn * e_n;
  
  gvf_control.error_n = e_n;
  
  // ---------------------------------------------------------------------------

  // Telemetry data
  gvf_telemetry.n_norm = ke*e*sqrtf(nx*nx + ny*ny);
  gvf_telemetry.t_norm = sqrtf(tx*tx + ty*ty);
  gvf_telemetry.omega_d = omega_d;
  gvf_telemetry.omega = omega;

  // Set GVF common info
  gvf_c_info.kappa   = (nx*(H12*ny - nx*H22) + ny*(H21*nx - H11*ny))/powf(nx*nx + ny*ny,1.5);
  gvf_c_info.ori_err = e_n;

  // Call the low level control
  gvf_low_level_control_2D(omega);

  #endif
}

/** FLIGHT_PLAN FUNCTIONS -------------------------------------------------- **/

// BEGIN ROTORCRAFT

void gvf_set_speed(float speed)
{
  if (speed < 0.0) speed = 0.0;
  gvf_control.speed = speed;
}

void gvf_set_align(bool align)
{
  gvf_control.align = align;
}

// END ROTORCRAFT

void gvf_set_direction(int8_t s)
{
  gvf_control.s = s;
}

/** ------------------------------------------------------------------------ **/

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

#include "gvf_ik.h"
#include "autopilot.h"

/* TODO:
 * - Reset gamma_t0 whenever any gamma setting is updated.
 * - The GVF drawn in the GCS is not the IK-GVF, so it needs to be fixed.
 */

// Control
gvf_ik_con gvf_ik_control = {0};
gvf_ik_tel gvf_ik_telemetry = {0};

// Time variables to check if GVF is active
static uint32_t gamma_t0 = 0;
static uint32_t last_gvf_t0 = 0;

/** TELEMETRY -------------------------------------------------------------- **/

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{

  pprz_msg_send_GVF_IK(
  trans, dev, AC_ID,
  &gvf_ik_control.error,
  &gvf_ik_control.gamma,
  &gvf_ik_control.gamma_dot,
  &gvf_ik_control.gamma_amplitude,
  &gvf_ik_control.gamma_omega
  );

  uint32_t delta_T = get_sys_time_msec() - last_gvf_t0;
  if (delta_T < 200) {
    uint8_t traj_type = (uint8_t)gvf_trajectory.type;
    
    pprz_msg_send_GVF(
      trans, dev, AC_ID, 
      &gvf_ik_control.phi,
      &traj_type, 
      &gvf_ik_control.s, 
      &gvf_ik_control.ke, 
      gvf_trajectory.p_len, gvf_trajectory.p,
      &gvf_ik_control.error_n, 
      &gvf_ik_telemetry.n_norm,
      &gvf_ik_telemetry.t_norm,
      &gvf_ik_telemetry.omega_d,
      &gvf_ik_telemetry.omega);

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

/** STATIC FUNCTIONS ------------------------------------------------------- **/

static void reset_gamma_t0(void)
{
  gamma_t0 = get_sys_time_msec();
}

static bool check_alpha(float ke, float J1, float J2, float phi, float speed)
{ 
  float J_Jt = (J1*J1 + J2*J2);

  float e       = phi - gvf_ik_control.gamma;
  float e_tdot  = - gvf_ik_control.gamma_dot;

  float u = - ke * e;

  float nx = J1 / J_Jt * (u - e_tdot);
  float ny = J2 / J_Jt * (u - e_tdot);

  float un_norm2 = nx*nx + ny*ny;

  return un_norm2 < speed*speed;
}

/** ------------------------------------------------------------------------ **/

void gvf_ik_init(void)
{
  gvf_ik_control.ke = 1;
  gvf_ik_control.kn = 1;
  gvf_ik_control.s = 1;
  gvf_trajectory.type = NONE;

  gvf_ik_control.gamma_amplitude = GVF_IK_GAMMA_AMPLITUDE;
  gvf_ik_control.gamma_omega = GVF_IK_GAMMA_OMEGA;

  reset_gamma_t0();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
#endif // PERIODIC_TELEMETRY
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_ik_control_2D(float ke, float kn, float phi,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{

  // compute gamma period
  float gamma_period = 2 * M_PI / gvf_ik_control.gamma_omega;

  // compute gamma t
  last_gvf_t0 = get_sys_time_msec();
  float t = (last_gvf_t0 - gamma_t0) / 1000;

  if (t > gamma_period) {
    reset_gamma_t0();
  }
  
  // get state
  gvf_low_level_getState();
  float course = gvf_c_state.course;
  float px_dot = gvf_c_state.px_dot;
  float py_dot = gvf_c_state.py_dot;

  float speed = sqrtf(px_dot*px_dot + py_dot*py_dot);
  float speed2 = speed * speed;

  int s = gvf_ik_control.s;

  // gradient Phi
  float J1 = grad->nx;
  float J2 = grad->ny;
  float J_Jt = (J1*J1 + J2*J2);

  // Hessian
  float H11 = hess->H11;
  float H12 = hess->H12;
  float H21 = hess->H21;
  float H22 = hess->H22;

  // -- Calculation of the desired angular velocity in the vector field --------
  gvf_ik_control.gamma = gvf_ik_control.gamma_amplitude  * sinf(gvf_ik_control.gamma_omega * t);
  gvf_ik_control.gamma_dot = gvf_ik_control.gamma_omega * gvf_ik_control.gamma_amplitude  * cosf(gvf_ik_control.gamma_omega * t);

  bool cond_flag = check_alpha(ke, J1, J2, phi, speed);

  float e = phi;
  float e_tdot = 0;
  float e_tddot = 0;
  if (cond_flag) {
    e       = e - gvf_ik_control.gamma;
    e_tdot  = - gvf_ik_control.gamma_dot;
    e_tddot = (gvf_ik_control.gamma_omega * gvf_ik_control.gamma_omega) * gvf_ik_control.gamma;
  }

  // control law
  float u = - ke * e;

  // normal to Phi in p_dot (IK)
  float nx = J1 / J_Jt * (u - e_tdot);
  float ny = J2 / J_Jt * (u - e_tdot);

  float n_norm2 = nx*nx + ny*ny;
  float n_norm = sqrtf(n_norm2);
  
  // tangent to Phi in p_dot
  float tx = s * J2;
  float ty = -s * J1;

  float t_norm = sqrtf(tx*tx + ty*ty);
  float tx_hat = tx / t_norm;
  float ty_hat = ty / t_norm;
  
  // compute alpha and p_dot
  float alpha = 0;
  float pdx_dot = 0;
  float pdy_dot = 0;
  
  if (cond_flag){
    alpha = sqrtf(speed2 - n_norm2);
    pdx_dot = alpha * tx_hat + nx;
    pdy_dot = alpha * ty_hat + ny;
  } else {
    pdx_dot = speed * nx / n_norm;
    pdy_dot = speed * ny / n_norm;
  }

  // compute n_dot
  float u_dot = - ke * (J1*pdx_dot + J2*pdy_dot + e_tdot);

  float un_dot_A_x = (pdx_dot*H11 + pdy_dot*H21);
  float un_dot_A_y = (pdx_dot*H12 + pdy_dot*H22);

  float B_term = 2 * ((H11*pdx_dot + H21*pdy_dot)*J1 + (H21*pdx_dot + H22*pdy_dot)*J2) / J_Jt;
  float un_dot_B_x = - J1 * B_term;
  float un_dot_B_y = - J2 * B_term;

  float C_term = (u_dot - e_tddot) / J_Jt;
  float un_dot_C_x = J1 * C_term;
  float un_dot_C_y = J2 * C_term;

  float nx_dot = (un_dot_A_x + un_dot_B_x) * (u - e_tdot) / J_Jt + un_dot_C_x;
  float ny_dot = (un_dot_A_y + un_dot_B_y) * (u - e_tdot) / J_Jt + un_dot_C_y;

  // compute omega_d and omega
  float pd_ddot_x = 0;
  float pd_ddot_y = 0;
  
  if (cond_flag){
    float alpha_dot = - (nx*nx_dot + ny*ny_dot) / (alpha);

    float tx_dot = s * (H12 * pdx_dot + H22 * pdy_dot);
    float ty_dot = - s * (H11 * pdx_dot + H21 * pdy_dot);

    float t_norm3 = t_norm * t_norm * t_norm;
    float Bpd_ddot_x = alpha * (tx_dot / t_norm + (tx*tx*tx_dot - tx*ty*ty_dot) / t_norm3);
    float Bpd_ddot_y = alpha * (ty_dot / t_norm + (tx*ty*tx_dot - ty*ty*ty_dot) / t_norm3);

    pd_ddot_x = alpha_dot * tx_hat + Bpd_ddot_x + nx_dot;
    pd_ddot_y = alpha_dot * ty_hat + Bpd_ddot_y + ny_dot;
  } else {
    float n_norm3 = n_norm2 * n_norm;
    pd_ddot_x = speed * (nx_dot / n_norm + (nx*nx*nx_dot - nx*ny*ny_dot) / n_norm3);
    pd_ddot_y = speed * (ny_dot / n_norm + (nx*ny*nx_dot - ny*ny*ny_dot) / n_norm3);
  }
  
  
  float omega_d = - (- pdx_dot*pd_ddot_y + pdy_dot*pd_ddot_x) / speed2;

  float rx = speed * sinf(course);
  float ry = speed * cosf(course);

  float e_n = (pdx_dot*ry - pdy_dot*rx) / speed2;
  float omega = omega_d - kn * e_n;

  // Update external control-related and telemetry variables 
  gvf_ik_control.phi = phi;
  gvf_ik_control.error = e;
  gvf_ik_control.error_n = e_n;
  gvf_ik_telemetry.n_norm = n_norm;
  gvf_ik_telemetry.t_norm = alpha;
  gvf_ik_telemetry.omega_d = omega_d;
  gvf_ik_telemetry.omega = omega;

  //printf("t: %f, pdx_dot: %f, pdy_dot: %f, rx: %f , ry: %f, omega: %f \n", t, pdx_dot, pdy_dot, rx, ry, omega);
  
  // ---------------------------------------------------------------------------

  // Set GVF common info
  gvf_c_info.kappa   = (J1*(H12*J2 - J1*H22) + J2*(H21*J1 - H11*J2))/powf(J_Jt, 1.5);
  gvf_c_info.ori_err = e_n;

  // Send the computed omega to the low level control
  gvf_low_level_control_2D(omega);
}

void gvf_ik_set_direction(int8_t s)
{
  gvf_ik_control.s = s;
}

/** ------------------------------------------------------------------------ **/

/*
 * Copyright (C) 2023 Tomaso De Ponti <tmldeponti@tudelft.nl>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "firmwares/rotorcraft/oneloop/oneloop_andi.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) ANDI controller for rotorcrafts
 */
///// EXPLANATION OF HALFLOOP ///////////////////////////////////////////////////////////
/**
 * @param oneloop_andi_half_loop - A Boolean indicating the state of the oneloop controller.
 * @param oneloop_andi_half_loop = true - Control allocation is performed to accommodate desired Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk (ANDI).
 * @param oneloop_andi_half_loop = false - Control allocation is performed to accommodate desired Jerk North, Jerk East, Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk (ANDI).
 */
///// Enter functions change the state of the oneloop controller //////////////////////
/**
 * @fn stabilization_attitude_enter in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * @result oneloop_andi_half_loop = true
 */
/**
 * @fn guidance_h_run_enter in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * @result oneloop_andi_half_loop = false
 */
/**
 * @fn guidance_v_run_enter in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * @result nothing
 */
///// Example of execution of the oneloop controller for two different states in the state machine /////////////////////
/**
 * @file "sw/airborne/firmwares/rotorcraft/autopilot_static.c"
 * 
 * @if MODE_ATTITUDE_RC_DIRECT
 * 
 * - @fn stabilization_attitude_run() in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * 
 * - - @if half_loop
 * 
 * - - - @fn oneloop_andi_run(true) in @file "firmwares/rotorcraft/oneloop/oneloop_andi.c"
 * 
 * - - - @result: Control allocation is performed to accommodate desired Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk from stick inputs
 * - - @endif
 * 
 * @elseif MODE_NAV
 * 
 * - @fn guidance_h_run() in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * 
 * - @fn oneloop_andi_run(false) in @file "firmwares/rotorcraft/oneloop/oneloop_andi.c"
 * 
 * - @result: Control allocation is performed to accommodate desired Jerk North, Jerk East, Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk from navigation outputs
 * 
 * - @fn guidance_v_run() in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * 
 * - @result: nothing
 * 
 * - @fn stabilization_attitude_run() in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * 
 * - @result: nothing because of oneloop_andi_half_loop = false
 * @endif
 */


/* Include necessary header files */
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "filters/notch_filter_float.h"
#include "math/wls/wls_alloc.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "modules/core/commands.h"
#include "modules/ctrl/eff_scheduling_rotwing_V2.h"
#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif

#include "modules/gps/gps.h" // DELETE FIX
//#include "nps/nps_fdm.h"

// Number of real actuators (e.g. motors, servos)
#ifndef ONELOOP_ANDI_NUM_THRUSTERS
float num_thrusters_oneloop = 4.0; // Number of motors used for thrust
#else
float num_thrusters_oneloop = ONELOOP_ANDI_NUM_THRUSTERS;
#endif

#ifndef ONELOOP_ANDI_SCHEDULING
#define ONELOOP_ANDI_SCHEDULING FALSE
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF
float  oneloop_andi_filt_cutoff = ONELOOP_ANDI_FILT_CUTOFF;
#else
float  oneloop_andi_filt_cutoff = 2.0;
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_ACC
float  oneloop_andi_filt_cutoff_a = ONELOOP_ANDI_FILT_CUTOFF_ACC;
#else
float  oneloop_andi_filt_cutoff_a = 2.0;
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_VEL
float  oneloop_andi_filt_cutoff_v = ONELOOP_ANDI_FILT_CUTOFF_VEL;
#else
float  oneloop_andi_filt_cutoff_v = 2.0;
#endif
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_VEL)
#ifdef ONELOOP_ANDI_FILT_CUTOFF_POS
float  oneloop_andi_filt_cutoff_pos = ONELOOP_ANDI_FILT_CUTOFF_POS;
#else
float  oneloop_andi_filt_cutoff_pos = 2.0;
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_P
#define ONELOOP_ANDI_FILTER_ROLL_RATE TRUE
float oneloop_andi_filt_cutoff_p = ONELOOP_ANDI_FILT_CUTOFF_P;
#else
float oneloop_andi_filt_cutoff_p = 20.0;
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_Q
#define ONELOOP_ANDI_FILTER_PITCH_RATE TRUE
float oneloop_andi_filt_cutoff_q = ONELOOP_ANDI_FILT_CUTOFF_Q;
#else
float oneloop_andi_filt_cutoff_q = 20.0;
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_R
#define ONELOOP_ANDI_FILTER_YAW_RATE TRUE
float oneloop_andi_filt_cutoff_r = ONELOOP_ANDI_FILT_CUTOFF_R;
#else
float oneloop_andi_filt_cutoff_r = 20.0;
#endif

PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF);
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_Q);
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_P);
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_R);
#ifndef MAX_R 
float max_r = RadOfDeg(120.0);
#else
float max_r = RadOfDeg(MAX_R);
#endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float  act_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
#else
#error "You must specify the actuator dynamics"
float  act_dynamics[ANDI_NUM_ACT_TOT] = = {1};
#endif

#ifdef ONELOOP_ANDI_ACT_MAX
float  act_max[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX;
#else
float  act_max[ANDI_NUM_ACT_TOT] = = {MAX_PPRZ};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN
float  act_min[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN;
#else
float  act_min[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MAX_NORM
float  act_max_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX_NORM;
#else
float  act_max_norm[ANDI_NUM_ACT_TOT] = = {1.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN_NORM
float  act_min_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN_NORM;
#else
float  act_min_norm[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_NU_NORM_MAX
float  nu_norm_max = ONELOOP_ANDI_NU_NORM_MAX;
#else
float  nu_norm_max = 1.0;
#endif

#ifdef ONELOOP_ANDI_U_PREF
static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_U_PREF;
#else
static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
#endif

#ifndef ONELOOP_ANDI_DEBUG_MODE
#define ONELOOP_ANDI_DEBUG_MODE  FALSE
#endif

// Assume phi and theta are the first actuators after the real ones unless otherwise specified
#define ONELOOP_ANDI_MAX_BANK  act_max[COMMAND_ROLL] // assuming abs of max and min is the same
#define ONELOOP_ANDI_MAX_PHI   act_max[COMMAND_ROLL] // assuming abs of max and min is the same

#define ONELOOP_ANDI_MAX_THETA   act_max[COMMAND_PITCH] // assuming abs of max and min is the same

#ifndef ONELOOP_THETA_PREF_MAX
float theta_pref_max = RadOfDeg(20.0);
#else
float theta_pref_max = RadOfDeg(ONELOOP_THETA_PREF_MAX);
#endif

#if ANDI_NUM_ACT_TOT != WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX is not equal to the number of actuators: define WLS_N_U_MAX == ANDI_NUM_ACT_TOT in airframe file
#define WLS_N_U_MAX == ANDI_NUM_ACT_TOT
#endif
#if ANDI_OUTPUTS != WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX is not equal to the number of controlled axis: define WLS_N_V_MAX == ANDI_OUTPUTS in airframe file
#define WLS_N_V_MAX == ANDI_OUTPUTS
#endif

#ifndef ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD
#define ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD 10.0
#endif

/* Declaration of Navigation Variables*/
#ifdef NAV_HYBRID_MAX_DECELERATION
float max_a_nav = NAV_HYBRID_MAX_DECELERATION;
#else
float max_a_nav = 4.0;   // (35[N]/6.5[Kg]) = 5.38[m/s2]  [0.8 SF]
#endif

#ifdef ONELOOP_MAX_JERK
float max_j_nav = ONELOOP_MAX_JERK;
#else
float max_j_nav = 500.0; // Pusher Test shows erros above 2[Hz] ramp commands [0.6 SF]
#endif

#ifdef ONELOOP_MAX_ANGULAR_JERK
float max_j_ang = ONELOOP_MAX_ANGULAR_JERK;
#else
float max_j_ang = 1000.0; 
#endif

#ifdef NAV_HYBRID_MAX_AIRSPEED
float max_v_nav = NAV_HYBRID_MAX_AIRSPEED; // Consider implications of difference Ground speed and airspeed
#else
float max_v_nav = 5.0;
#endif
float max_as = 19.0f;
float min_as = 0.0f;

#ifdef NAV_HYBRID_MAX_SPEED_V
float max_v_nav_v = NAV_HYBRID_MAX_SPEED_V;
#else
float max_v_nav_v = 1.5;
#endif

#ifndef FWD_SIDESLIP_GAIN
float fwd_sideslip_gain = 0.2;
#else
float fwd_sideslip_gain = FWD_SIDESLIP_GAIN;
#endif

#ifndef ONELOOP_ANDI_WU_QUAD_MOTORS_FWD
float Wu_quad_motors_fwd = 6.0;
#else
float Wu_quad_motors_fwd = ONELOOP_ANDI_WU_QUAD_MOTORS_FWD;
#endif


/*  Define Section of the functions used in this module*/
void  init_poles(void);
void  init_poles_att(void);
void  init_poles_pos(void);
void  calc_normalization(void);
void  normalize_nu(void);
void  G1G2_oneloop(int ctrl_type);
void  get_act_state_oneloop(void);
void  oneloop_andi_propagate_filters(void);
void  init_filter(void);
void  init_controller(void);
void  float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3]);
void  float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3]);
void  err_nd(float err[], float a[], float b[], float k[], int n);
void  err_sum_nd(float err[], float a[], float b[], float k[], float c[], int n);
void  integrate_nd(float dt, float a[], float a_dot[], int n);
void  vect_bound_nd(float vect[], float bound, int n);
void  acc_body_bound(struct FloatVect2* vect, float bound);
float bound_v_from_a(float e_x[], float v_bound, float a_bound, int n);
void  rm_2nd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm);
void  rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3], float max_ang_jerk);
void  rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n);
void  rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n);
void  rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n);
void  ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3], float max_ang_jerk);
void  ec_3rd_pos(float y_4d[], float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x[], float x_d[], float x_2d[], float k1_e[], float k2_e[], float k3_e[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n);
void  calc_model(void);
float oneloop_andi_sideslip(void);
void  reshape_wind(void);
void  chirp_pos(float time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  chirp_call(bool* chirp_on, bool* chirp_first_call, float* t_0_chirp, float* time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  init_cf2(struct CF2_t *cf, float fc);
void  init_cf4(struct CF4_t *cf, float fc);
void  init_all_cf(void);
void  reinit_cf2(struct CF2_t *cf, bool reinit);
void  reinit_cf4(struct CF4_t *cf, bool reinit);
void  reinit_all_cf(bool reinit);

/*Define general struct of the Oneloop ANDI controller*/
struct OneloopGeneral oneloop_andi;

/* Oneloop Misc variables*/
static float use_increment = 0.0;
static float nav_target[3]; // Can be a position, speed or acceleration depending on the guidance H mode
static float nav_target_new[3];
static float dt_1l = 1./PERIODIC_FREQUENCY;
static float g   = 9.81; // [m/s^2] Gravitational Acceleration
float k_as = 2.0;
int16_t temp_pitch = 0;
float gi_unbounded_airspeed_sp = 0.0;

/* Oneloop Control Variables*/
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
static float andi_du_n[ANDI_NUM_ACT_TOT];
float nu[ANDI_OUTPUTS];
float nu_n[ANDI_OUTPUTS];
static float act_dynamics_d[ANDI_NUM_ACT_TOT];
float actuator_state_1l[ANDI_NUM_ACT];
static float a_thrust = 0.0;
static float g2_ff= 0.0;

/*Attitude related variables*/
struct Int32Eulers stab_att_sp_euler_1l;// here for now to correct warning, can be better eploited in the future 
struct Int32Quat   stab_att_sp_quat_1l; // here for now to correct warning, can be better eploited in the future
struct FloatEulers eulers_zxy_des;
struct FloatEulers eulers_zxy;
//static float  psi_des_rad = 0.0;
float  psi_des_rad = 0.0;
float  psi_des_deg = 0.0;
static float  psi_vec[4]  = {0.0, 0.0, 0.0, 0.0};

#if ONELOOP_ANDI_HEADING_MANUAL
bool heading_manual = true;
#else
bool heading_manual = false;
#endif
#if ONELOOP_ANDI_YAW_STICK_IN_AUTO
bool yaw_stick_in_auto = true;
#else
bool yaw_stick_in_auto = false;
#endif
bool ctrl_off = false;
/*WLS Settings*/

static float pitch_pref = 0;
struct WLS_t WLS_one_p = {
  .nu        = ANDI_NUM_ACT_TOT,
  .nv        = ANDI_OUTPUTS,
  .gamma_sq  = 100.0,
  .v         = {0.0},
#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}  
  .Wv        = ONELOOP_ANDI_WV,
#else
  .Wv        = {1.0},
#endif
#ifdef ONELOOP_ANDI_WU // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}  
  .Wu        = ONELOOP_ANDI_WU,
#else
  .Wu        = {1.0},
#endif
  .u_pref    = {0.0},
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};

#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wv_backup[ANDI_OUTPUTS] = ONELOOP_ANDI_WV;
#else
static float Wv_backup[ANDI_OUTPUTS] = {1.0};
#endif

#ifdef ONELOOP_ANDI_WU // {mF,mR,mB,mL,mP,de,dr,da,df,phi,theta}
static float Wu_backup[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_WU;
#else
static float Wu_backup[ANDI_NUM_ACT_TOT] = {1.0};
#endif

/*Complementary Filter Variables*/
struct Oneloop_CF_t cf;
static struct Oneloop_notch_t oneloop_notch; 

/*Chirp test Variables*/
bool  chirp_on            = false;
bool  chirp_first_call    = true;
float time_elapsed_chirp  = 0.0;
float t_0_chirp           = 0.0;
float f0_chirp            = 0.8 / (2.0 * M_PI);
float f1_chirp            = 0.8 / (2.0 * M_PI);
float t_chirp             = 45.0;
float A_chirp             = 1.0;
int8_t chirp_axis         = 0;
float p_ref_0[3]          = {0.0, 0.0, 0.0};

/*Declaration of Reference Model and Error Controller Gains*/
struct PolePlacement p_att_e;
struct PolePlacement p_att_rm;
/*Position Loop*/
struct PolePlacement p_pos_e;  
struct PolePlacement p_pos_rm;
/*Altitude Loop*/
struct PolePlacement p_alt_e;   
struct PolePlacement p_alt_rm; 
/*Heading Loop*/
struct PolePlacement p_head_e;
struct PolePlacement p_head_rm;
/*Gains of EC and RM ANDI*/
struct Gains3rdOrder k_att_e;
struct Gains3rdOrder k_att_rm;  
struct Gains3rdOrder k_pos_e;
struct Gains3rdOrder k_pos_rm; 
/*Gains of EC and RM INDI*/
struct Gains3rdOrder k_att_e_indi;
struct Gains3rdOrder k_pos_e_indi;

/* Effectiveness Matrix definition */
float *bwls_1l[ANDI_OUTPUTS];
float EFF_MAT_G[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float ratio_u_un[ANDI_NUM_ACT_TOT];
float ratio_vn_v[ANDI_OUTPUTS];

/*Filters Initialization*/
static Butterworth2LowPass filt_veloc_N;                 // Low pass filter for velocity NED - oneloop_andi_filt_cutoff_a (tau_a)       
static Butterworth2LowPass filt_veloc_E;
static Butterworth2LowPass filt_veloc_D;
static Butterworth2LowPass accely_filt;                       // Low pass filter for acceleration in y direction                - oneloop_andi_filt_cutoff (tau)
static Butterworth2LowPass airspeed_filt;                     // Low pass filter for airspeed                                - oneloop_andi_filt_cutoff (tau)
/* Define messages of the module*/
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
// static void send_cf_oneloop(struct transport_tx *trans, struct link_device *dev)
// {
//   float temp_cf_p[5] = {cf.p.model,cf.p.model_filt.o[0],cf.p.feedback,cf.p.feedback_filt.o[0],cf.p.out};
//   float temp_cf_q[5] = {cf.q.model,cf.q.model_filt.o[0],cf.q.feedback,cf.q.feedback_filt.o[0],cf.q.out};
//   float temp_cf_r[5] = {cf.r.model,cf.r.model_filt.o[0],cf.r.feedback,cf.r.feedback_filt.o[0],cf.r.out};
//   float temp_cf_p_dot[5] = {cf.p_dot.model,cf.p_dot.model_filt.lp2.o[0],cf.p_dot.feedback,cf.p_dot.feedback_filt.lp2.o[0],cf.p_dot.out};
//   float temp_cf_q_dot[5] = {cf.q_dot.model,cf.q_dot.model_filt.lp2.o[0],cf.q_dot.feedback,cf.q_dot.feedback_filt.lp2.o[0],cf.q_dot.out};
//   float temp_cf_r_dot[5] = {cf.r_dot.model,cf.r_dot.model_filt.o[0],cf.r_dot.feedback,cf.r_dot.feedback_filt.o[0],cf.r_dot.out};
//   //float temp_cf_ax[5] = {cf.ax.model,cf.ax.model_filt.o[0],cf.ax.feedback,cf.ax.feedback_filt.o[0],cf.ax.out};
//   //float temp_cf_ay[5] = {cf.ay.model,cf.ay.model_filt.o[0],cf.ay.feedback,cf.ay.feedback_filt.o[0],cf.ay.out};
//   float temp_cf_az[5] = {cf.az.model,cf.az.model_filt.o[0],cf.az.feedback,cf.az.feedback_filt.o[0],cf.az.out};
//   pprz_msg_send_COMPLEMENTARY_FILTER(trans, dev, AC_ID, 
//                 5, temp_cf_p,
//                 5, temp_cf_q,
//                 5, temp_cf_r,
//                 5, temp_cf_p_dot,
//                 5, temp_cf_q_dot,
//                 5, temp_cf_r_dot,
//                 5, temp_cf_az);
// }
static void send_wls_v_oneloop(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_v("one", &WLS_one_p, trans, dev); 
}
static void send_wls_u_oneloop(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_u("one", &WLS_one_p, trans, dev); 
}
static void send_eff_mat_stab_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                ANDI_NUM_ACT, EFF_MAT_RW[3],
                ANDI_NUM_ACT, EFF_MAT_RW[4],
                ANDI_NUM_ACT, EFF_MAT_RW[5], 
                                    1, &zero,
                                    1, &zero);
}

static void send_eff_mat_guid_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EFF_MAT_GUID(trans, dev, AC_ID, 
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[0],
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[1],
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[2]);
}
static void send_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float temp_eulers_zxy_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                        3, temp_eulers_zxy_des,
                                        3, oneloop_andi.sta_state.att,
                                        3, oneloop_andi.sta_ref.att,
                                        3, oneloop_andi.sta_state.att_d,
                                        3, oneloop_andi.sta_ref.att_d,                                       
                                        3, oneloop_andi.sta_state.att_2d,
                                        3, oneloop_andi.sta_ref.att_2d,
                                        3, oneloop_andi.sta_ref.att_3d,                                       
                                        ANDI_NUM_ACT, actuator_state_1l);                                      
}
// static void send_oneloop_actuator_state(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_ACTUATOR_STATE(trans, dev, AC_ID, 
//                                         ANDI_NUM_ACT, actuator_state_1l,
//                                         ANDI_NUM_ACT_TOT, andi_u);
// }
static void send_guidance_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE(trans, dev, AC_ID,
                                        &oneloop_andi.gui_ref.pos[0],
                                        &oneloop_andi.gui_ref.pos[1],
                                        &oneloop_andi.gui_ref.pos[2],
                                        &oneloop_andi.gui_state.pos[0],
                                        &oneloop_andi.gui_state.pos[1],
                                        &oneloop_andi.gui_state.pos[2],
                                        &oneloop_andi.gui_ref.vel[0],
                                        &oneloop_andi.gui_ref.vel[1],
                                        &oneloop_andi.gui_ref.vel[2],
                                        &oneloop_andi.gui_state.vel[0],
                                        &oneloop_andi.gui_state.vel[1],
                                        &oneloop_andi.gui_state.vel[2],
                                        &oneloop_andi.gui_ref.acc[0],
                                        &oneloop_andi.gui_ref.acc[1],
                                        &oneloop_andi.gui_ref.acc[2],
                                        &oneloop_andi.gui_state.acc[0],
                                        &oneloop_andi.gui_state.acc[1],
                                        &oneloop_andi.gui_state.acc[2],
                                        &oneloop_andi.gui_ref.jer[0],
                                        &oneloop_andi.gui_ref.jer[1],
                                        &oneloop_andi.gui_ref.jer[2]);
}

static void debug_vect(struct transport_tx *trans, struct link_device *dev, char *name, float *data, int datasize)
{
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID,
                           strlen(name), name,
                           datasize, data);
}

static void send_oneloop_debug(struct transport_tx *trans, struct link_device *dev)
{
  float temp_debug_vect[9];
  temp_debug_vect[0] = cf.ax.model;
  temp_debug_vect[1] = cf.ay.model;
  temp_debug_vect[2] = cf.az.model;
  temp_debug_vect[3] = cf.p_dot.model;
  temp_debug_vect[4] = cf.q_dot.model;
  temp_debug_vect[5] = cf.r_dot.model;
  temp_debug_vect[6] = cf.p.model;
  temp_debug_vect[7] = cf.q.model;
  temp_debug_vect[8] = cf.r.model;
  debug_vect(trans, dev, "model_cf", temp_debug_vect, 9);
}
#endif

/** @brief Function to make sure that inputs are positive non zero vaues*/
static float positive_non_zero(float input)
{
  if (input < FLT_EPSILON) {
    input = 0.00001;
  }
  return input;
}

/** @brief  Error Controller Gain Design */

static float k_e_1_2_f_v2(float omega, float zeta) {
    omega = positive_non_zero(omega);
    zeta  = positive_non_zero(zeta);
    return (omega * omega);
}

static float k_e_2_2_f_v2(float omega, float zeta) {
    omega = positive_non_zero(omega);
    zeta  = positive_non_zero(zeta);
    return (2* zeta * omega);
}

static float k_e_1_3_f_v2(float omega_n, UNUSED float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    p1      = positive_non_zero(p1);
    return (omega_n * omega_n * p1);
}

static float k_e_2_3_f_v2(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return (omega_n * omega_n + 2 * zeta * omega_n * p1);
}

static float k_e_3_3_f_v2(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return (2 * zeta * omega_n + p1);
}

/** @brief Reference Model Gain Design */

static float k_rm_1_3_f(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return (omega_n * omega_n * p1) / (omega_n * omega_n + omega_n * p1 * zeta * 2.0);
}

static float k_rm_2_3_f(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return (omega_n * omega_n + omega_n * p1 * zeta * 2.0) / (p1 + omega_n * zeta * 2.0);
}

static float k_rm_3_3_f(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return p1 + omega_n * zeta * 2.0;
}

/** @brief Attitude Rates to Euler Conversion Function */
void float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3])
{
  float sphi   = sinf(e[0]);
  float cphi   = cosf(e[0]);
  float stheta = sinf(e[1]);
  float ctheta = cosf(e[1]);
  r[0] =  edot[0] - stheta * edot[2];
  r[1] =  cphi * edot[1] + sphi * ctheta * edot[2];
  r[2] = -sphi * edot[1] + cphi * ctheta * edot[2];
}

/** @brief Attitude Euler to Rates Conversion Function */
void float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3])
{
  float sphi   = sinf(e[0]);
  float cphi   = cosf(e[0]);
  float stheta = sinf(e[1]);
  float ctheta = cosf(e[1]);
  if (fabs(ctheta) < FLT_EPSILON){
    ctheta = FLT_EPSILON;
  }
  edot[0] = r[0] + sphi*stheta/ctheta*r[1] + cphi*stheta/ctheta*r[2];
  edot[1] = cphi*r[1] - sphi*r[2];
  edot[2] = sphi/ctheta*r[1] + cphi/ctheta*r[2];
}

/** @brief Calculate Scaled Error between two 3D arrays*/
void err_nd(float err[], float a[], float b[], float k[], int n)
{
  int8_t i;
  for (i = 0; i < n; i++) {
    err[i] = k[i] * (a[i] - b[i]);
  }
}

/** @brief Calculate Scaled Error between two 3D arrays*/
void err_sum_nd(float err[], float a[], float b[], float k[], float c[], int n)
{
  int8_t i;
  for (i = 0; i < n; i++) {
    err[i] = k[i] * (a[i] - b[i]);
    err[i] += c[i];
  }
}

/** @brief Integrate in time 3D array*/
void integrate_nd(float dt, float a[], float a_dot[], int n)
{
  int8_t i;
  for (i = 0; i < n; i++) {
    a[i] = a[i] + dt * a_dot[i];
  }
}

/** @brief Scale a 3D array to within a 3D bound */
void vect_bound_nd(float vect[], float bound, int n) {
  float norm = float_vect_norm(vect,n);
  norm = positive_non_zero(norm);
  if((norm-bound) > FLT_EPSILON) {
    float scale = bound/norm;
    int8_t i;
    for(i = 0; i < n; i++) {
      vect[i] *= scale;
    }
  }
}

/** @brief Scale a 3D array to within a 3D bound */
void acc_body_bound(struct FloatVect2* vect, float bound) {
  int n = 2;
  float v[2] = {vect->x, vect->y};
  float sign_v0 = (v[0] > 0.f) ? 1.f : (v[0] < 0.f) ? -1.f : 0.f;
  float sign_v1 = (v[1] > 0.f) ? 1.f : (v[1] < 0.f) ? -1.f : 0.f;
  float norm = float_vect_norm(v,n);
  v[0] = fabsf(v[0]);
  v[1] = fabsf(v[1]);
  norm = positive_non_zero(norm);
  if((norm-bound) > FLT_EPSILON) {
    v[0] = Min(v[0], bound);
    float acc_b_y_2 = bound*bound - v[0]*v[0];
    acc_b_y_2 = positive_non_zero(acc_b_y_2);
    v[1] = sqrtf(acc_b_y_2);
  }
  vect->x = sign_v0*v[0];
  vect->y = sign_v1*v[1];
}

/** @brief Calculate velocity limit based on acceleration limit */
float bound_v_from_a(float e_x[], float v_bound, float a_bound, int n) {
  float norm = float_vect_norm(e_x,n);
  norm = fmaxf(norm, 1.0);
  float v_bound_a  = sqrtf(fabs(2.0 * a_bound * norm));
  return fminf(v_bound, v_bound_a);
}

/** 
 * @brief Reference Model Definition for 3rd order system with attitude conversion functions
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param ow_psi          Overwrite psi (for navigation functions) [bool]
 * @param psi_overwrite   Overwrite psi (for navigation functions) [values]
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3], float max_ang_jerk){
  float e_x[3];
  float e_x_rates[3];
  float e_x_d[3];
  float e_x_2d[3];
  float x_d_eul_ref[3];
  err_nd(e_x, x_des, x_ref, k1_rm, 3);
  float temp_diff = x_des[2] - x_ref[2];
  NormRadAngle(temp_diff);
  e_x[2] = k1_rm[2] * temp_diff; // Correction for Heading error +-Pi
  float_rates_of_euler_dot_vec(e_x_rates, x_ref, e_x);
  err_nd(e_x_d, e_x_rates, x_d_ref, k2_rm, 3);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, 3);
  float_vect_copy(x_3d_ref,e_x_2d,3);
  vect_bound_nd(x_3d_ref, max_ang_jerk, 3);
  if(ow_psi){x_3d_ref[2] = psi_overwrite[3];}
  integrate_nd(dt, x_2d_ref, x_3d_ref, 3);
  if(ow_psi){x_2d_ref[2] = psi_overwrite[2];}
  integrate_nd(dt, x_d_ref, x_2d_ref, 3);
  if(ow_psi){x_d_ref[2] = psi_overwrite[1];}
  float_euler_dot_of_rates_vec(x_d_ref, x_ref, x_d_eul_ref);
  integrate_nd(dt, x_ref, x_d_eul_ref, 3);
  if(ow_psi){x_ref[2] = psi_overwrite[0];}
  NormRadAngle(x_ref[2]);
  //printf("k1_rm: %f %f %f\n", k1_rm[0], k1_rm[1], k1_rm[2]);
  //printf("k2_rm: %f %f %f\n", k2_rm[0], k2_rm[1], k2_rm[2]);
  //printf("k3_rm: %f %f %f\n", k3_rm[0], k3_rm[1], k3_rm[2]);
}

/** 
 * @brief Reference Model Definition for 3rd order system 
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float e_x_2d   = k3_rm * (e_x_d- *x_2d_ref);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to the heading angle
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float temp_diff = x_des - *x_ref;
  NormRadAngle(temp_diff);
  float e_x      = k1_rm * temp_diff;
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float e_x_2d   = k3_rm * (e_x_d- *x_2d_ref);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param x_d_bound       Bound for the 2nd order reference signal
 * @param x_2d_bound      Bound for the 3rd order reference signal
 * @param x_3d_bound      Bound for the 4th order reference signal
 * @param n               Number of dimensions
 */
void rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n){
  float e_x[n];
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x, x_des, x_ref, k1_rm, n);
  float max_x_d = bound_v_from_a(e_x, x_d_bound, x_2d_bound, n);
  vect_bound_nd(e_x, max_x_d, n);
  err_nd(e_x_d, e_x, x_d_ref, k2_rm, n);
  vect_bound_nd(e_x_d,x_2d_bound, n);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref,e_x_2d,n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
  integrate_nd(dt, x_d_ref, x_2d_ref, n);
  integrate_nd(dt, x_ref, x_d_ref, n);
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt              Delta time [s]
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_d_des         Desired 2nd order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param x_2d_bound      Bound for the 3rd order reference signal
 * @param x_3d_bound      Bound for the 4th order reference signal
 * @param n               Number of dimensions
 */
void rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n){
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x_d, x_d_des, x_d_ref, k2_rm, n);
  vect_bound_nd(e_x_d,x_2d_bound, n);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref,e_x_2d,n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
  integrate_nd(dt, x_d_ref, x_2d_ref, n);
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt              Delta time [s]
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_2d_des        Desired 3rd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param x_3d_bound      Bound for the 4th order reference signal
 * @param n               Number of dimensions
 */
void rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n){
  float e_x_2d[n];
  err_nd(e_x_2d, x_2d_des, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref,e_x_2d,n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
}


/** 
 * @brief Reference Model Definition for 2nd order system
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_2nd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  *x_2d_ref = e_x_d;
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/** 
 * @brief Error Controller Definition for 3rd order system 
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param x               Current 1st order signal
 * @param x_d             Current 2nd order signal
 * @param x_2d            Current 3rd order signal
 * @param k1_e            Error Controller Gain 1st order signal
 * @param k2_e            Error Controller Gain 2nd order signal
 * @param k3_e            Error Controller Gain 3rd order signal
 */
static float ec_3rd(float x_ref, float x_d_ref, float x_2d_ref, float x_3d_ref, float x, float x_d, float x_2d, float k1_e, float k2_e, float k3_e){
  float y_4d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+k3_e*(x_2d_ref-x_2d)+x_3d_ref;
  return y_4d;
}
void ec_3rd_pos( float y_4d[], float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x[], float x_d[], float x_2d[], float k1_e[], float k2_e[], float k3_e[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n){
  float e_x_d[n];
  float e_x_2d[n];

  err_sum_nd(e_x_d, x_ref, x, k1_e, x_d_ref, n);
  vect_bound_nd(e_x_d, x_d_bound, n);
  
  err_sum_nd(e_x_2d, e_x_d, x_d, k2_e, x_2d_ref, n);
  vect_bound_nd(e_x_2d,x_2d_bound, n);

  err_sum_nd(y_4d, e_x_2d, x_2d, k3_e, x_3d_ref, n);
  vect_bound_nd(y_4d, x_3d_bound, n);
}
/** 
 * @brief Error Controller Definition for 3rd order system specific to attitude
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param x               Current 1st order signal
 * @param x_d             Current 2nd order signal
 * @param x_2d            Current 3rd order signal
 * @param k1_e            Error Controller Gain 1st order signal
 * @param k2_e            Error Controller Gain 2nd order signal
 * @param k3_e            Error Controller Gain 3rd order signal
 */
void ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3], float max_ang_jerk){
  float y_4d_1[3];
  float y_4d_2[3];
  float y_4d_3[3];
  err_nd(y_4d_1, x_ref, x, k1_e, 3);
  float temp_diff = x_ref[2] - x[2];
  NormRadAngle(temp_diff);
  float e_x = temp_diff;
  y_4d_1[2] = k1_e[2] * e_x; // Correction for Heading error +-Pi
  err_nd(y_4d_2, x_d_ref, x_d, k2_e, 3);
  err_nd(y_4d_3, x_2d_ref, x_2d, k3_e, 3);
  float_vect_copy(y_4d,x_3d_ref,3);
  float_vect_sum(y_4d, y_4d, y_4d_1, 3);
  float_vect_sum(y_4d, y_4d, y_4d_2, 3);
  float_vect_sum(y_4d, y_4d, y_4d_3, 3);
  vect_bound_nd(y_4d, max_ang_jerk, 3);
}

/**
 * @brief  Third Order to First Order Dynamics Approximation
 * @param p1              Pole 1
 * @param p2              Pole 2
 * @param p3              Pole 3
 * @param rm_k            Reference Model Gain
 */
static float w_approx(float p1, float p2, float p3, float rm_k){
  p1   = positive_non_zero(p1);
  p2   = positive_non_zero(p2);
  p3   = positive_non_zero(p3);
  rm_k = positive_non_zero(rm_k);
  float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
  tao  = positive_non_zero(tao);
  return 1.0/tao;
}

/**
 * @brief Calculate EC poles given RM poles
 * @param p_rm      Reference Model Pole (3 coincident poles)
 * @param slow_pole Pole of the slowest dynamics
 * @param k         EC / RM ratio
 * @param omega_n   Natural Frequency
 */
static float ec_poles(float p_rm, float slow_pole, float k){
  p_rm      = positive_non_zero(p_rm);
  slow_pole = positive_non_zero(slow_pole);
  k         = positive_non_zero(k);
  float omega_n = (2*p_rm*slow_pole*k)/(3*slow_pole-p_rm);
  return omega_n;
}

/**
 * @brief Initialize Position of Poles
 * 
 */
void init_poles_att(void){
  float slow_pole  = 22.0; // Pole of the slowest dynamics used in the attitude controller
  p_att_e.omega_n  = ec_poles(p_att_rm.omega_n,  slow_pole, 1.28);
  p_head_e.omega_n = ec_poles(p_head_rm.omega_n, slow_pole, 1.28);
}
void init_poles_pos(void){
  act_dynamics[COMMAND_ROLL]  = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[COMMAND_PITCH] = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  float slow_pole = act_dynamics[COMMAND_ROLL]; // Pole of the slowest dynamics used in the position controller
  p_pos_e.omega_n = ec_poles(p_pos_rm.omega_n,slow_pole,1.28);//1.0;
  p_alt_e.omega_n = ec_poles(p_alt_rm.omega_n,slow_pole,1.28);//1.0;// 3.0;
}

/**
 * @brief Initialize Position of Poles
 * 
 */
void init_poles(void){

  // Attitude Controller Poles----------------------------------------------------------
  float slow_pole = 22.0; // Pole of the slowest dynamics used in the attitude controller

  p_att_rm.omega_n = 10.0; // 10.0 4.71
  p_att_rm.zeta    = 1.0;
  p_att_rm.p3      = p_att_rm.omega_n * p_att_rm.zeta;

  p_att_e.omega_n = ec_poles(p_att_rm.omega_n,  slow_pole, 1.28); //4.50;
  p_att_e.zeta    = 1.0;
  p_att_e.p3      = slow_pole;

  p_head_rm.omega_n = 7.0; // 7.0 2.56
  p_head_rm.zeta    = 1.0;
  p_head_rm.p3      = p_head_rm.omega_n * p_head_rm.zeta;

  p_head_e.omega_n = ec_poles(p_head_rm.omega_n, slow_pole, 1.28); //1.80;
  p_head_e.zeta    = 1.0;
  p_head_e.p3      = slow_pole;

  act_dynamics[COMMAND_ROLL]  = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[COMMAND_PITCH] = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);

  // Position Controller Poles----------------------------------------------------------
  slow_pole = act_dynamics[COMMAND_ROLL]; // Pole of the slowest dynamics used in the position controller
#ifdef ONELOOP_ANDI_POLES_POS_OMEGA_N
  p_pos_rm.omega_n = ONELOOP_ANDI_POLES_POS_OMEGA_N;
#else
  p_pos_rm.omega_n = 0.93; //2.2;
#endif
  p_pos_rm.zeta    = 1.0;  
  p_pos_rm.p3      = p_pos_rm.omega_n * p_pos_rm.zeta;

  p_pos_e.omega_n = ec_poles(p_pos_rm.omega_n,slow_pole,1.28);//1.0;
  p_pos_e.zeta    = 1.0; 
  p_pos_e.p3      = slow_pole; 
#ifdef ONELOOP_ANDI_POLES_ALT_OMEGA_N
  p_alt_rm.omega_n = ONELOOP_ANDI_POLES_ALT_OMEGA_N;
#else
  p_alt_rm.omega_n = 0.93; //2.2;
#endif
  p_alt_rm.zeta    = 1.0;
  p_alt_rm.p3      = p_alt_rm.omega_n * p_alt_rm.zeta;

  p_alt_e.omega_n = ec_poles(p_alt_rm.omega_n,slow_pole,1.28);//1.0;// 3.0;
  p_alt_e.zeta    = 1.0; 
  p_alt_e.p3      = slow_pole; 
}

/** 
 * @brief Initialize Controller Gains
 * FIXME: Calculate the gains dynamically for transition
 */
void init_controller(void){
  /*Register a variable from nav_hybrid. Should be improved when nav hybrid is final.*/
  float max_wind  = 20.0;
  max_v_nav = nav_max_speed + max_wind;
  max_a_nav = nav_max_acceleration_sp;
  /*Some calculations in case new poles have been specified*/
  init_poles_att();
  init_poles_pos();
  p_att_rm.p3  = p_att_rm.omega_n  * p_att_rm.zeta;
  p_pos_rm.p3  = p_pos_rm.omega_n  * p_pos_rm.zeta;
  p_alt_rm.p3  = p_alt_rm.omega_n  * p_alt_rm.zeta;
  p_head_rm.p3 = p_head_rm.omega_n * p_head_rm.zeta;

  //--ANDI Controller gains --------------------------------------------------------------------------------
  /*Attitude Loop*/
  k_att_e.k1[0]  = k_e_1_3_f_v2(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k2[0]  = k_e_2_3_f_v2(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k3[0]  = k_e_3_3_f_v2(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k1[1]  = k_att_e.k1[0]; 
  k_att_e.k2[1]  = k_att_e.k2[0]; 
  k_att_e.k3[1]  = k_att_e.k3[0]; 

  k_att_rm.k1[0] = k_rm_1_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k2[0] = k_rm_2_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k3[0] = k_rm_3_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k1[1] = k_att_rm.k1[0];
  k_att_rm.k2[1] = k_att_rm.k2[0];
  k_att_rm.k3[1] = k_att_rm.k3[0];
  
  //printf("Attitude RM Gains: %f %f %f\n", k_att_rm.k1[0], k_att_rm.k2[0], k_att_rm.k3[0]);
  //printf("Attitude E Gains: %f %f %f\n", k_att_e.k1[0], k_att_e.k2[0], k_att_e.k3[0]);
  //printf("Heading E Gains: %f %f %f\n", k_att_e.k1[2], k_att_e.k2[2], k_att_e.k3[2]);
  /*Heading Loop NAV*/
  k_att_e.k1[2]  = k_e_1_3_f_v2(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k2[2]  = k_e_2_3_f_v2(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k3[2]  = k_e_3_3_f_v2(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);

  k_att_rm.k1[2] = k_rm_1_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k2[2] = k_rm_2_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k3[2] = k_rm_3_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);

  /*Position Loop*/
  // k_pos_e.k1[0]  = k_e_1_3_f_v2(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  // k_pos_e.k2[0]  = k_e_2_3_f_v2(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  // k_pos_e.k3[0]  = k_e_3_3_f_v2(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  k_pos_e.k1[0]  = k_rm_1_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  k_pos_e.k2[0]  = k_rm_2_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  k_pos_e.k3[0]  = k_rm_3_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  k_pos_e.k1[1]  = k_pos_e.k1[0];  
  k_pos_e.k2[1]  = k_pos_e.k2[0];  
  k_pos_e.k3[1]  = k_pos_e.k3[0]; 

  k_pos_rm.k1[0] = k_rm_1_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);
  k_pos_rm.k2[0] = k_rm_2_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);
  k_pos_rm.k3[0] = k_rm_3_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);
  k_pos_rm.k1[1] = k_pos_rm.k1[0];  
  k_pos_rm.k2[1] = k_pos_rm.k2[0];  
  k_pos_rm.k3[1] = k_pos_rm.k3[0];
  nav_hybrid_pos_gain   = k_pos_rm.k1[0];
  nav_hybrid_max_bank   = ONELOOP_ANDI_MAX_BANK;

  /*Altitude Loop*/
  // k_pos_e.k1[2]  = k_e_1_3_f_v2(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  // k_pos_e.k2[2]  = k_e_2_3_f_v2(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  // k_pos_e.k3[2]  = k_e_3_3_f_v2(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  k_pos_e.k1[2]  = k_rm_1_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  k_pos_e.k2[2]  = k_rm_2_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  k_pos_e.k3[2]  = k_rm_3_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);

  k_pos_rm.k1[2] = k_rm_1_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3);
  k_pos_rm.k2[2] = k_rm_2_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3);
  k_pos_rm.k3[2] = k_rm_3_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3);
  
  //--INDI Controller gains --------------------------------------------------------------------------------
    /*Attitude Loop*/
  k_att_e_indi.k1[0]  = k_e_1_2_f_v2(p_att_e.omega_n, p_att_e.zeta);
  k_att_e_indi.k2[0]  = k_e_2_2_f_v2(p_att_e.omega_n, p_att_e.zeta);
  k_att_e_indi.k3[0]  = 1.0;
  k_att_e_indi.k1[1]  = k_att_e_indi.k1[0]; 
  k_att_e_indi.k2[1]  = k_att_e_indi.k2[0]; 
  k_att_e_indi.k3[1]  = k_att_e_indi.k3[0]; 

  /*Heading Loop NAV*/
  k_att_e_indi.k1[2]  = k_e_1_2_f_v2(p_head_e.omega_n, p_head_e.zeta);
  k_att_e_indi.k2[2]  = k_e_2_2_f_v2(p_head_e.omega_n, p_head_e.zeta);
  k_att_e_indi.k3[2]  = 1.0;

  /*Position Loop*/
  k_pos_e_indi.k1[0]  = k_e_1_2_f_v2(p_pos_e.omega_n, p_pos_e.zeta);
  k_pos_e_indi.k2[0]  = k_e_2_2_f_v2(p_pos_e.omega_n, p_pos_e.zeta);
  k_pos_e_indi.k3[0]  = 1.0;
  k_pos_e_indi.k1[1]  = k_pos_e_indi.k1[0];  
  k_pos_e_indi.k2[1]  = k_pos_e_indi.k2[0];  
  k_pos_e_indi.k3[1]  = k_pos_e_indi.k3[0]; 

  /*Altitude Loop*/
  k_pos_e_indi.k1[2]  = k_e_1_2_f_v2(p_alt_e.omega_n, p_alt_e.zeta);
  k_pos_e_indi.k2[2]  = k_e_2_2_f_v2(p_alt_e.omega_n, p_alt_e.zeta);
  k_pos_e_indi.k3[2]  = 1.0;
   
  //------------------------------------------------------------------------------------------
  /*Approximated Dynamics*/
  act_dynamics[COMMAND_ROLL]   = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[COMMAND_PITCH]  = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
}
// Complementary Filters Functions -----------------------------------------------------------
/** @brief Initialize the Complementary Filters 2nd Order Butterworth */
void init_cf2(struct CF2_t *cf, float fc){
  cf->freq     = fc;
  cf->freq_set = fc;
  cf->tau      = 1/(2*M_PI*cf->freq);
  init_butterworth_2_low_pass(&cf->model_filt,    cf->tau, 1.0 / PERIODIC_FREQUENCY, 0.0);
  init_butterworth_2_low_pass(&cf->feedback_filt, cf->tau, 1.0 / PERIODIC_FREQUENCY, 0.0);
  cf->model    = 0.0;
  cf->feedback = 0.0;
  cf->out      = 0.0;
}
/** @brief  Initialize the Complementary Filters 4th Order Butterworth*/
void init_cf4(struct CF4_t *cf, float fc){
  cf->freq     = fc;
  cf->freq_set = fc;
  cf->tau      = 1/(2*M_PI*cf->freq);
  init_butterworth_4_low_pass(&cf->model_filt,    cf->tau, 1.0 / PERIODIC_FREQUENCY, 0.0);
  init_butterworth_4_low_pass(&cf->feedback_filt, cf->tau, 1.0 / PERIODIC_FREQUENCY, 0.0);
  cf->model    = 0.0;
  cf->feedback = 0.0;
  cf->out      = 0.0;
}
/** @brief  Initialize all the Complementary Filters */
void init_all_cf(void){
  init_cf2(&cf.ax,    oneloop_andi_filt_cutoff_a);
  init_cf2(&cf.ay,    oneloop_andi_filt_cutoff_a);
  init_cf2(&cf.az,    oneloop_andi_filt_cutoff_a);
  init_cf4(&cf.p_dot, 2.0);
  init_cf4(&cf.q_dot, 2.0);
  init_cf2(&cf.r_dot, oneloop_andi_filt_cutoff);
  init_cf2(&cf.p,     oneloop_andi_filt_cutoff_p);
  init_cf2(&cf.q,     oneloop_andi_filt_cutoff_q);
  init_cf2(&cf.r,     oneloop_andi_filt_cutoff_r);
}
/** @brief Reinitialize 2nd Order CF if new frequency setting or if forced */
void reinit_cf2(struct CF2_t *cf, bool reinit){
  if(cf->freq != cf->freq_set || reinit){
    cf->freq = cf->freq_set;
    cf->tau = 1/(2*M_PI*cf->freq);
    init_butterworth_2_low_pass(&cf->model_filt,    cf->tau, 1.0 / PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&cf->model_filt));
    init_butterworth_2_low_pass(&cf->feedback_filt, cf->tau, 1.0 / PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&cf->feedback_filt));
  }
}
/** @brief Reinitialize 4th Order CF if new frequency setting or if forced */
void reinit_cf4(struct CF4_t *cf, bool reinit){
  if(cf->freq != cf->freq_set || reinit){
    cf->freq = cf->freq_set;
    cf->tau = 1/(2*M_PI*cf->freq);
    init_butterworth_4_low_pass(&cf->model_filt,    cf->tau, 1.0 / PERIODIC_FREQUENCY, get_butterworth_4_low_pass(&cf->model_filt));
    init_butterworth_4_low_pass(&cf->feedback_filt, cf->tau, 1.0 / PERIODIC_FREQUENCY, get_butterworth_4_low_pass(&cf->feedback_filt));
  }
}
/** @brief Reinitialize all the Complementary Filters */
void reinit_all_cf(bool reinit){
  reinit_cf2(&cf.ax,    reinit);
  reinit_cf2(&cf.ay,    reinit);
  reinit_cf2(&cf.az,    reinit);
  reinit_cf4(&cf.p_dot, reinit);
  reinit_cf4(&cf.q_dot, reinit);
  reinit_cf2(&cf.r_dot, reinit);
  reinit_cf2(&cf.p,     reinit);
  reinit_cf2(&cf.q,     reinit);
  reinit_cf2(&cf.r,     reinit);
}
//------------------------------------------------------------------------------------------
/** @brief  Initialize the filters */
void init_filter(void)
{
  // Store Notch filter values
#ifdef ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ
  oneloop_notch.roll.freq = ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ,
#else
  oneloop_notch.roll.freq = 8.46,
#endif
#ifdef ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ
  oneloop_notch.pitch.freq = ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ,
#else
  oneloop_notch.pitch.freq = 6.44,
#endif
#ifdef ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ
  oneloop_notch.yaw.freq = ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ,
#else
  oneloop_notch.yaw.freq = 17.90,
#endif
  oneloop_notch.roll.bandwidth  = 2.0;
  oneloop_notch.pitch.bandwidth = 1.0;
  oneloop_notch.yaw.bandwidth   = 4.0;
  // Initialize Notch filters
  notch_filter_init(&oneloop_notch.roll.filter, oneloop_notch.roll.freq, oneloop_notch.roll.bandwidth, PERIODIC_FREQUENCY);
  notch_filter_init(&oneloop_notch.pitch.filter, oneloop_notch.pitch.freq, oneloop_notch.pitch.bandwidth, PERIODIC_FREQUENCY);
  notch_filter_init(&oneloop_notch.yaw.filter, oneloop_notch.yaw.freq, oneloop_notch.yaw.bandwidth, PERIODIC_FREQUENCY);
  // Filtering of the velocities 
  float tau   = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff);
  float tau_v = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_v);
  //printf("tau: %f tau_v: %f\n", tau, tau_v);
  //printf("initializing filters\n");
  init_butterworth_2_low_pass(&filt_veloc_N,      tau_v, 1.0 / PERIODIC_FREQUENCY, 0.0);
  init_butterworth_2_low_pass(&filt_veloc_E,      tau_v, 1.0 / PERIODIC_FREQUENCY, 0.0);
  init_butterworth_2_low_pass(&filt_veloc_D,      tau_v, 1.0 / PERIODIC_FREQUENCY, 0.0);
  init_butterworth_2_low_pass(&accely_filt,       tau,   1.0 / PERIODIC_FREQUENCY, 0.0);
  init_butterworth_2_low_pass(&airspeed_filt,     tau,   1.0 / PERIODIC_FREQUENCY, 0.0);
}


/** @brief  Propagate the filters */
void oneloop_andi_propagate_filters(void) {
  reinit_all_cf(false);
  struct  NedCoor_f *accel = stateGetAccelNed_f();
  struct  NedCoor_f *veloc = stateGetSpeedNed_f();
  //printf("veloc: %f %f %f\n", veloc->x, veloc->y, veloc->z);
  struct  FloatRates *body_rates = stateGetBodyRates_f();
  // Store Feedbacks in the Complementary Filters
  cf.ax.feedback    = accel->x;
  cf.ay.feedback    = accel->y;
  cf.az.feedback    = accel->z;
//#define USE_ROLL_NOTCH
//#define USE_PITCH_NOTCH
#define USE_YAW_NOTCH
  float temp_p_dot  = (body_rates->p-cf.p.feedback)*PERIODIC_FREQUENCY;
  float temp_q_dot  = (body_rates->q-cf.q.feedback)*PERIODIC_FREQUENCY;
  float temp_r_dot  = (body_rates->r-cf.r.feedback)*PERIODIC_FREQUENCY;
#ifdef USE_ROLL_NOTCH
  notch_filter_update(&oneloop_notch.roll.filter, &temp_p_dot, &cf.p_dot.feedback);
#else
  cf.p_dot.feedback = temp_p_dot;
#endif
#ifdef USE_PITCH_NOTCH
  notch_filter_update(&oneloop_notch.pitch.filter, &temp_q_dot, &cf.q_dot.feedback);
#else
  cf.q_dot.feedback = temp_q_dot;
#endif
#ifdef USE_YAW_NOTCH
  notch_filter_update(&oneloop_notch.yaw.filter, &temp_r_dot, &cf.r_dot.feedback);
#else
  cf.r_dot.feedback = temp_r_dot;
#endif
  cf.p.feedback     = body_rates->p;
  cf.q.feedback     = body_rates->q;
  cf.r.feedback     = body_rates->r;
  // Update Filters of Feedbacks
  update_butterworth_2_low_pass(&cf.ax.feedback_filt,    cf.ax.feedback);
  update_butterworth_2_low_pass(&cf.ay.feedback_filt,    cf.ay.feedback);
  update_butterworth_2_low_pass(&cf.az.feedback_filt,    cf.az.feedback);
  update_butterworth_4_low_pass(&cf.p_dot.feedback_filt, cf.p_dot.feedback);
  update_butterworth_4_low_pass(&cf.q_dot.feedback_filt, cf.q_dot.feedback);
  update_butterworth_2_low_pass(&cf.r_dot.feedback_filt, cf.r_dot.feedback);
  update_butterworth_2_low_pass(&cf.p.feedback_filt,     cf.p.feedback);
  update_butterworth_2_low_pass(&cf.q.feedback_filt,     cf.q.feedback);
  update_butterworth_2_low_pass(&cf.r.feedback_filt,     cf.r.feedback); 
  //printf("veloc: %f %f %f\n", veloc->x, veloc->y, veloc->z);
  update_butterworth_2_low_pass(&filt_veloc_N,      veloc->x);
  update_butterworth_2_low_pass(&filt_veloc_E,      veloc->y);
  update_butterworth_2_low_pass(&filt_veloc_D,      veloc->z); 
  //printf("veloc_filt: %f %f %f\n", filt_veloc_N.o[0], filt_veloc_E.o[0], filt_veloc_D.o[0]);
  // Calculate Model Predictions for Linear and Angular Accelerations Using the Effectiveness Matrix
  calc_model();
  // Update Filters of Model Predictions for Linear and Angular Accelerations
  update_butterworth_2_low_pass(&cf.ax.model_filt,    cf.ax.model);
  update_butterworth_2_low_pass(&cf.ay.model_filt,    cf.ay.model);
  update_butterworth_2_low_pass(&cf.az.model_filt,    cf.az.model);
  update_butterworth_4_low_pass(&cf.p_dot.model_filt, cf.p_dot.model);
  update_butterworth_4_low_pass(&cf.q_dot.model_filt, cf.q_dot.model);
  update_butterworth_2_low_pass(&cf.r_dot.model_filt, cf.r_dot.model);
  // Calculate Complementary Filter outputs for Linear and Angular Accelerations 
  cf.ax.out    = cf.ax.feedback_filt.o[0]    + cf.ax.model    - cf.ax.model_filt.o[0];
  cf.ay.out    = cf.ay.feedback_filt.o[0]    + cf.ay.model    - cf.ay.model_filt.o[0];
  cf.az.out    = cf.az.feedback_filt.o[0]    + cf.az.model    - cf.az.model_filt.o[0];
  //cf.p_dot.out = cf.p_dot.feedback_filt.o[0] + cf.p_dot.model - cf.p_dot.model_filt.o[0];
  cf.p_dot.out = cf.p_dot.feedback_filt.lp2.o[0] + cf.p_dot.model - cf.p_dot.model_filt.lp2.o[0];
  //cf.q_dot.out = cf.q_dot.feedback_filt.o[0] + cf.q_dot.model - cf.q_dot.model_filt.o[0];
  cf.q_dot.out = cf.q_dot.feedback_filt.lp2.o[0] + cf.q_dot.model - cf.q_dot.model_filt.lp2.o[0];
  cf.r_dot.out = cf.r_dot.feedback_filt.o[0] + cf.r_dot.model - cf.r_dot.model_filt.o[0];
  // Calculate Model Predictions for Angular Rates Using the output of the angular acceleration Complementary Filter
  cf.p.model   = cf.p.model + cf.p_dot.out / PERIODIC_FREQUENCY;
  cf.q.model   = cf.q.model + cf.q_dot.out / PERIODIC_FREQUENCY;
  cf.r.model   = cf.r.model + cf.r_dot.out / PERIODIC_FREQUENCY;
  // Update Filters of Model Predictions for Angular Rates
  update_butterworth_2_low_pass(&cf.p.model_filt,     cf.p.model);
  update_butterworth_2_low_pass(&cf.q.model_filt,     cf.q.model);
  update_butterworth_2_low_pass(&cf.r.model_filt,     cf.r.model);
  // Calculate Complementary Filter outputs for Angular Rates
  cf.p.out     = cf.p.feedback_filt.o[0]     + cf.p.model     - cf.p.model_filt.o[0];
  cf.q.out     = cf.q.feedback_filt.o[0]     + cf.q.model     - cf.q.model_filt.o[0];
  cf.r.out     = cf.r.feedback_filt.o[0]     + cf.r.model     - cf.r.model_filt.o[0];
  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);
  float airspeed_meas = stateGetAirspeed_f();
  Bound(airspeed_meas, 0.0, 30.0);
  update_butterworth_2_low_pass(&airspeed_filt, airspeed_meas);
}

/** @brief Init function of Oneloop ANDI controller  */
void oneloop_andi_init(void)
{ 
  oneloop_andi.half_loop = true;
  oneloop_andi.ctrl_type = CTRL_ANDI;
  init_poles();
  // Make sure that the dynamics are positive and non-zero
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    act_dynamics[i] = positive_non_zero(act_dynamics[i]);
  }
  // Initialize Effectiveness matrix
  calc_normalization();
  G1G2_oneloop(oneloop_andi.ctrl_type);
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = EFF_MAT_G[i];
  }
  // Initialize filters and other variables
  init_all_cf();
  init_filter();
  init_controller();
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du_n, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l,ANDI_NUM_ACT);
  float_vect_zero(oneloop_andi.sta_ref.att,3);
  float_vect_zero(oneloop_andi.sta_ref.att_d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_2d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_3d,3);
  float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(nu_n, ANDI_OUTPUTS);
  float_vect_zero(nav_target,3);
  float_vect_zero(nav_target_new,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_STAB, send_eff_mat_stab_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_GUID, send_eff_mat_guid_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE, send_guidance_oneloop_andi);
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATOR_STATE, send_oneloop_actuator_state);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_oneloop_debug);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_oneloop);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_oneloop);
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMPLEMENTARY_FILTER, send_cf_oneloop);
  #endif
}

/**
 * @brief Function that resets important values upon engaging Oneloop ANDI.
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_andi_enter(bool half_loop_sp, int ctrl_type)
{
  ele_min = 0.0;
  oneloop_andi.half_loop     = half_loop_sp;
  oneloop_andi.ctrl_type     = ctrl_type;
  psi_des_rad   = eulers_zxy.psi; 
  psi_des_deg   = DegOfRad(eulers_zxy.psi);
  calc_normalization();
  G1G2_oneloop(oneloop_andi.ctrl_type);
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = EFF_MAT_G[i];
  }
  reinit_all_cf(true);
  init_filter();
  init_controller();
  /* Stabilization Reset */
  float_vect_zero(oneloop_andi.sta_ref.att,2);
  float_vect_zero(oneloop_andi.sta_ref.att_d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_2d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_3d,3);
  float_vect_zero(nav_target,3);
  float_vect_zero(nav_target_new,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  psi_des_rad;
  /*Guidance Reset*/
}

/**
 * @brief  Function to generate the reference signals for the oneloop controller
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param PSA_des    Desired position/speed/acceleration
 * @param rm_order_h Order of the reference model for horizontal guidance
 * @param rm_order_v Order of the reference model for vertical guidance
 */
void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v, bool in_flight_oneloop)
{
  // Initialize some variables
  a_thrust = 0.0;
  nav_target[0] = PSA_des.x;
  nav_target[1] = PSA_des.y;
  nav_target[2] = PSA_des.z;
  float thrust_cmd_1l = 0.0;
  float des_r = 0.0;
  // Generate reference signals with reference model 
  if(half_loop){
    // Disregard X and Y jerk objectives
    WLS_one_p.Wv[0] = 0.0;
    WLS_one_p.Wv[1] = 0.0;
    // Overwrite references with actual signals (for consistent plotting)
    float_vect_copy(oneloop_andi.gui_ref.pos,oneloop_andi.gui_state.pos,3);
    float_vect_copy(oneloop_andi.gui_ref.vel,oneloop_andi.gui_state.vel,3);
    float_vect_copy(oneloop_andi.gui_ref.acc,oneloop_andi.gui_state.acc,3);
    float_vect_zero(oneloop_andi.gui_ref.jer,3);
    // Set desired attitude with stick input
    eulers_zxy_des.phi   = (float) (radio_control_get(RADIO_ROLL)) /MAX_PPRZ * ONELOOP_ANDI_MAX_PHI  ;
    eulers_zxy_des.theta = (float) (radio_control_get(RADIO_PITCH))/MAX_PPRZ * ONELOOP_ANDI_MAX_THETA;
    // Set desired Yaw rate with stick input
    des_r = (float) (radio_control_get(RADIO_YAW))/MAX_PPRZ*max_r;            // Get yaw rate from stick
    BoundAbs(des_r,max_r);                                                    // Bound yaw rate
    float delta_psi_des_rad = des_r * dt_1l;                                  // Integrate desired Yaw rate to get desired change in yaw
    float delta_psi_rad = eulers_zxy_des.psi-eulers_zxy.psi;                  // Calculate current yaw difference between des and actual
    NormRadAngle(delta_psi_rad);                                              // Normalize the difference
    if (fabs(delta_psi_rad) > RadOfDeg(30.0)){                                // If difference is bigger than 10 deg do not further increment desired
      delta_psi_des_rad = 0.0;
    }
    psi_des_rad += delta_psi_des_rad;                                         // Incrementdesired yaw
    NormRadAngle(psi_des_rad);
    // Register Attitude Setpoints from previous loop
    if (!in_flight_oneloop){
      psi_des_rad   = eulers_zxy.psi;
    }
    eulers_zxy_des.psi = psi_des_rad;
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
    // Create commands adhoc to get actuators to the wanted level
    thrust_cmd_1l = (float) radio_control_get(RADIO_THROTTLE);
    Bound(thrust_cmd_1l,0.0,MAX_PPRZ); 
    int8_t i;
    for (i = 0; i < ANDI_NUM_ACT; i++) {
     a_thrust +=(thrust_cmd_1l - use_increment*actuator_state_1l[i]) * bwls_1l[2][i] / (ratio_u_un[i] * ratio_vn_v[RW_aD]);
    }
    rm_3rd_attitude(dt_1l, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, att_des, false, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3, max_j_ang);
  }else{
    // Make sure X and Y jerk objectives are active
    WLS_one_p.Wv[0] = Wv_backup[0];
    WLS_one_p.Wv[1] = Wv_backup[1];
    // Generate Reference signals for positioning using RM
    if (rm_order_h == 3){
      rm_3rd_pos(dt_1l, oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target, k_pos_rm.k1, k_pos_rm.k2, k_pos_rm.k3, max_v_nav, max_a_nav, max_j_nav, 2);    
    } else if (rm_order_h == 2){
      float_vect_copy(oneloop_andi.gui_ref.pos, oneloop_andi.gui_state.pos,2);
      float_vect_copy(oneloop_andi.gui_ref.vel, oneloop_andi.gui_state.vel,2);
      reshape_wind();//returns accel sp as nav target new
      rm_1st_pos(dt_1l, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target_new, k_pos_rm.k3, max_j_nav, 2); 
      // rm_2nd_pos(dt_1l, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target_new, k_pos_rm.k2, k_pos_rm.k3, max_a_nav, max_j_nav, 2);   
    } else if (rm_order_h == 1){
      float_vect_copy(oneloop_andi.gui_ref.pos, oneloop_andi.gui_state.pos,2);
      float_vect_copy(oneloop_andi.gui_ref.vel, oneloop_andi.gui_state.vel,2);
      rm_1st_pos(dt_1l, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target, k_pos_rm.k3, max_j_nav, 2);   
    }
    // Update desired Heading (psi_des_rad) based on previous loop or changed setting
    if (heading_manual){
      psi_des_rad = RadOfDeg(psi_des_deg);
      if (yaw_stick_in_auto){
        psi_des_rad += (float) (radio_control_get(RADIO_YAW))/MAX_PPRZ*max_r * dt_1l;
      }
    } else {
      float ref_mag_vel = float_vect_norm(oneloop_andi.gui_ref.vel,2);
      psi_des_rad += oneloop_andi_sideslip() * dt_1l;
      NormRadAngle(psi_des_rad);
      if (ref_mag_vel > 3.0){
        float psi_gs = atan2f(oneloop_andi.gui_ref.vel[1],oneloop_andi.gui_ref.vel[0]);
        float delta_des_gs = psi_gs-psi_des_rad;  // Calculate current yaw difference between des and gs
        NormRadAngle(delta_des_gs);                            
        if (fabs(delta_des_gs) > RadOfDeg(60.0)){        // If difference is bigger than 60 deg bound the des psi angle so that it does not deviate too much
          delta_des_gs *= RadOfDeg(60.0)/fabs(delta_des_gs);
          psi_des_rad = psi_gs - delta_des_gs;
          NormRadAngle(psi_des_rad);
        }
      }
    }
    // Register Attitude Setpoints from previous loop
    if (!in_flight_oneloop){
      psi_des_rad   = eulers_zxy.psi;
    }
    eulers_zxy_des.psi = psi_des_rad;
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
    // The RM functions want an array as input. Create a single entry array and write the vertical guidance entries. 
    float single_value_ref[1]        = {oneloop_andi.gui_ref.pos[2]};
    float single_value_d_ref[1]      = {oneloop_andi.gui_ref.vel[2]};
    float single_value_2d_ref[1]     = {oneloop_andi.gui_ref.acc[2]};
    float single_value_3d_ref[1]     = {oneloop_andi.gui_ref.jer[2]};
    float single_value_nav_target[1] = {nav_target[2]};
    float single_value_k1_rm[1]      = {k_pos_rm.k1[2]};
    float single_value_k2_rm[1]      = {k_pos_rm.k2[2]};
    float single_value_k3_rm[1]      = {k_pos_rm.k3[2]};
    if (rm_order_v == 3){
      rm_3rd_pos(dt_1l, single_value_ref, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k1_rm, single_value_k2_rm, single_value_k3_rm, max_v_nav_v, max_a_nav, max_j_nav, 1);    
      oneloop_andi.gui_ref.pos[2] = single_value_ref[0];
      oneloop_andi.gui_ref.vel[2] = single_value_d_ref[0];
      oneloop_andi.gui_ref.acc[2] = single_value_2d_ref[0];
      oneloop_andi.gui_ref.jer[2] = single_value_3d_ref[0];
    } else if (rm_order_v == 2){
      rm_2nd_pos(dt_1l, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k2_rm, single_value_k3_rm, max_a_nav, max_j_nav, 1);   
      oneloop_andi.gui_ref.pos[2] = oneloop_andi.gui_state.pos[2];
      oneloop_andi.gui_ref.vel[2] = single_value_d_ref[0];
      oneloop_andi.gui_ref.acc[2] = single_value_2d_ref[0];
      oneloop_andi.gui_ref.jer[2] = single_value_3d_ref[0];
    } else if (rm_order_v == 1){
      rm_1st_pos(dt_1l, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k3_rm, max_j_nav, 1); 
      oneloop_andi.gui_ref.pos[2] = oneloop_andi.gui_state.pos[2];
      oneloop_andi.gui_ref.vel[2] = oneloop_andi.gui_state.vel[2];
      oneloop_andi.gui_ref.acc[2] = single_value_2d_ref[0];
      oneloop_andi.gui_ref.jer[2] = single_value_3d_ref[0];  
    }    
    // Run chirp test if turnerd on (overwrite the guidance references)
    chirp_call(&chirp_on, &chirp_first_call, &t_0_chirp, &time_elapsed_chirp, f0_chirp, f1_chirp, t_chirp, A_chirp, chirp_axis, att_des[2], oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer,p_ref_0);
    // Generate Reference signals for attitude using RM
    // FIX ME ow not yet defined, will be useful in the future to have accurate psi tracking in NAV functions
    bool ow_psi = false;
    rm_3rd_attitude(dt_1l, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, att_des, ow_psi, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3, max_j_ang);
 }
}

/**
 * @brief  Main function that runs the controller and performs control allocation
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param in_flight  The drone is in flight
 * @param PSA_des    Desired position/speed/acceleration
 * @param rm_order_h Order of the reference model for horizontal guidance
 * @param rm_order_v Order of the reference model for vertical guidance
 */
void oneloop_andi_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v)
{
  // At beginnig of the loop: (1) Register Attitude, (2) Initialize gains of RM and EC, (3) Calculate Normalization of Actuators Signals, (4) Propagate Actuator Model, (5) Update effectiveness matrix
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  init_controller();
  calc_normalization();
  get_act_state_oneloop();
  G1G2_oneloop(oneloop_andi.ctrl_type);
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = EFF_MAT_G[i];
  }
  
  // If drone is not on the ground use incremental law
  use_increment = 0.0;
  bool  in_flight_oneloop       = false;
  if(in_flight) {
    use_increment = 1.0;
    in_flight_oneloop       = true;
  } 
  if (ONELOOP_ANDI_DEBUG_MODE) {
    in_flight_oneloop = false;
  }
  
  // Register the state of the drone in the variables used in RM and EC
  // (1) Attitude related
  oneloop_andi.sta_state.att[0]    = eulers_zxy.phi  ;
  oneloop_andi.sta_state.att[1]    = eulers_zxy.theta;
  oneloop_andi.sta_state.att[2]    = eulers_zxy.psi  ;
  oneloop_andi_propagate_filters();   //needs to be after update of attitude vector
  oneloop_andi.sta_state.att_d[0]  = cf.p.out;
  oneloop_andi.sta_state.att_d[1]  = cf.q.out;
  oneloop_andi.sta_state.att_d[2]  = cf.r.out;
  oneloop_andi.sta_state.att_2d[0] = cf.p_dot.out;
  oneloop_andi.sta_state.att_2d[1] = cf.q_dot.out;
  oneloop_andi.sta_state.att_2d[2] = cf.r_dot.out;
  // (2) Position related
  oneloop_andi.gui_state.pos[0] = stateGetPositionNed_f()->x;   
  oneloop_andi.gui_state.pos[1] = stateGetPositionNed_f()->y;   
  oneloop_andi.gui_state.pos[2] = stateGetPositionNed_f()->z;   
  oneloop_andi.gui_state.vel[0] = filt_veloc_N.o[0];      
  oneloop_andi.gui_state.vel[1] = filt_veloc_E.o[0];      
  oneloop_andi.gui_state.vel[2] = filt_veloc_D.o[0];      
  oneloop_andi.gui_state.acc[0] = cf.ax.out;
  oneloop_andi.gui_state.acc[1] = cf.ay.out;
  oneloop_andi.gui_state.acc[2] = cf.az.out;
  // Calculated feedforward signal for yaw control
  g2_ff = 0.0;

  for (i = 0; i < ANDI_NUM_ACT; i++) {
    if (oneloop_andi.ctrl_type == CTRL_ANDI){
      g2_ff += G2_RW[i] * act_dynamics[i] * andi_du[i];
    } else if (oneloop_andi.ctrl_type == CTRL_INDI){
      g2_ff += G2_RW[i]* andi_du_n[i];
    }
  }
  //G2 is scaled by ANDI_G_SCALING to make it readable
  g2_ff = g2_ff;
  // Run the Reference Model (RM)
  oneloop_andi_RM(half_loop, PSA_des, rm_order_h, rm_order_v, in_flight_oneloop);
  // Guidance Pseudo Control Vector (nu) based on error controller
  if(half_loop){
    nu[0] = 0.0;
    nu[1] = 0.0;
    nu[2] = a_thrust;
    ctrl_off = false;
  }else{
    if(oneloop_andi.ctrl_type == CTRL_ANDI){
      ec_3rd_pos(nu, oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, oneloop_andi.gui_state.pos, oneloop_andi.gui_state.vel, oneloop_andi.gui_state.acc, k_pos_e.k1, k_pos_e.k2, k_pos_e.k3, max_v_nav, max_a_nav, max_j_nav, 3);
      // nu[0] = ec_3rd(oneloop_andi.gui_ref.pos[0], oneloop_andi.gui_ref.vel[0], oneloop_andi.gui_ref.acc[0], oneloop_andi.gui_ref.jer[0], oneloop_andi.gui_state.pos[0], oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.acc[0], k_pos_e.k1[0], k_pos_e.k2[0], k_pos_e.k3[0]);
      // nu[1] = ec_3rd(oneloop_andi.gui_ref.pos[1], oneloop_andi.gui_ref.vel[1], oneloop_andi.gui_ref.acc[1], oneloop_andi.gui_ref.jer[1], oneloop_andi.gui_state.pos[1], oneloop_andi.gui_state.vel[1], oneloop_andi.gui_state.acc[1], k_pos_e.k1[1], k_pos_e.k2[1], k_pos_e.k3[1]);
      // nu[2] = ec_3rd(oneloop_andi.gui_ref.pos[2], oneloop_andi.gui_ref.vel[2], oneloop_andi.gui_ref.acc[2], oneloop_andi.gui_ref.jer[2], oneloop_andi.gui_state.pos[2], oneloop_andi.gui_state.vel[2], oneloop_andi.gui_state.acc[2], k_pos_e.k1[2], k_pos_e.k2[2], k_pos_e.k3[2]); 
    } else if (oneloop_andi.ctrl_type == CTRL_INDI){
      nu[0] = ec_3rd(oneloop_andi.gui_ref.pos[0], oneloop_andi.gui_ref.vel[0], oneloop_andi.gui_ref.acc[0], 0.0, oneloop_andi.gui_state.pos[0], oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.acc[0], k_pos_e_indi.k1[0], k_pos_e_indi.k2[0], k_pos_e_indi.k3[0]);
      nu[1] = ec_3rd(oneloop_andi.gui_ref.pos[1], oneloop_andi.gui_ref.vel[1], oneloop_andi.gui_ref.acc[1], 0.0, oneloop_andi.gui_state.pos[1], oneloop_andi.gui_state.vel[1], oneloop_andi.gui_state.acc[1], k_pos_e_indi.k1[1], k_pos_e_indi.k2[1], k_pos_e_indi.k3[1]);
      nu[2] = ec_3rd(oneloop_andi.gui_ref.pos[2], oneloop_andi.gui_ref.vel[2], oneloop_andi.gui_ref.acc[2], 0.0, oneloop_andi.gui_state.pos[2], oneloop_andi.gui_state.vel[2], oneloop_andi.gui_state.acc[2], k_pos_e_indi.k1[2], k_pos_e_indi.k2[2], k_pos_e_indi.k3[2]);  
    }
  }
  // Attitude Pseudo Control Vector (nu) based on error controller
  float y_4d_att[3];  
  if(oneloop_andi.ctrl_type == CTRL_ANDI){
    ec_3rd_att(y_4d_att, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, oneloop_andi.sta_state.att, oneloop_andi.sta_state.att_d, oneloop_andi.sta_state.att_2d, k_att_e.k1, k_att_e.k2, k_att_e.k3, max_j_ang);
  } else if (oneloop_andi.ctrl_type == CTRL_INDI){
    float dummy0[3] = {0.0, 0.0, 0.0};
    ec_3rd_att(y_4d_att, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, dummy0, oneloop_andi.sta_state.att, oneloop_andi.sta_state.att_d, oneloop_andi.sta_state.att_2d, k_att_e_indi.k1, k_att_e_indi.k2, k_att_e_indi.k3, max_j_ang);
  }
  nu[3] = y_4d_att[0];  
  nu[4] = y_4d_att[1]; 
  nu[5] = y_4d_att[2] + g2_ff; 
  if (!chirp_on){
    pitch_pref = radio_control.values[RADIO_AUX5]; 
    pitch_pref = pitch_pref / MAX_PPRZ * theta_pref_max;
    Bound(pitch_pref,0.0,theta_pref_max);
  }
  u_pref[COMMAND_PITCH] = pitch_pref;
  // Calculate the min and max increments
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    switch (i) {
      case COMMAND_MOTOR_FRONT:
      case COMMAND_MOTOR_RIGHT:
      case COMMAND_MOTOR_BACK:
      case COMMAND_MOTOR_LEFT: {
        float skew_bound = RW.skew.deg;
        Bound(skew_bound,70.0,90.0);
        float Wu_sched   = (Wu_quad_motors_fwd-Wu_backup[i])/(90.0-70.0)*(skew_bound-70)+Wu_backup[i];
        WLS_one_p.Wu[i]     = Wu_sched;
        WLS_one_p.u_min[i]  = (act_min[i] - actuator_state_1l[i])/ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i]  - actuator_state_1l[i])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i] - actuator_state_1l[i])/ratio_u_un[i];
        if (rotwing_state.hover_motors_enabled){
          WLS_one_p.u_max[i]  = (act_max[i] - actuator_state_1l[i])/ratio_u_un[i];
        } else
        {
          WLS_one_p.u_max[i]  = (0.0 - actuator_state_1l[i])/ratio_u_un[i];
        }
        break;
      }
      case COMMAND_MOTOR_PUSHER:
      case COMMAND_RUDDER:
        WLS_one_p.u_min[i]  = (act_min[i] - actuator_state_1l[i])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i] - actuator_state_1l[i])/ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i]  - actuator_state_1l[i])/ratio_u_un[i]; 
        break;      
      case COMMAND_AILERONS:
        if(RW.skew.deg > 25.0){
            WLS_one_p.u_min[i]  = (act_min[i] - actuator_state_1l[i])/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (act_max[i] - actuator_state_1l[i])/ratio_u_un[i];
          } else {
            WLS_one_p.u_min[i]  = (0.0 - actuator_state_1l[i])/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (0.0 - actuator_state_1l[i])/ratio_u_un[i];
          }
        WLS_one_p.u_pref[i] = (u_pref[i]  - actuator_state_1l[i])/ratio_u_un[i];   
        break;      
      case COMMAND_FLAPS:
        if(RW.skew.deg > 50.0){
            WLS_one_p.u_min[i]  = (act_min[i] - actuator_state_1l[i])/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (act_max[i] - actuator_state_1l[i])/ratio_u_un[i];
          } else {
            WLS_one_p.u_min[i]  = (0.0 - actuator_state_1l[i])/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (0.0 - actuator_state_1l[i])/ratio_u_un[i];
          }
        WLS_one_p.u_pref[i] = (u_pref[i]  - actuator_state_1l[i])/ratio_u_un[i];   
        break;
      case COMMAND_ELEVATOR:  
        WLS_one_p.u_min[i]  = (act_min[i] - actuator_state_1l[i])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i] - actuator_state_1l[i])/ratio_u_un[i];
        u_pref[i]           = RW.ele_pref;
        WLS_one_p.u_pref[i] = (u_pref[i]  - actuator_state_1l[i])/ratio_u_un[i];  
        break;     
      case COMMAND_ROLL:
        WLS_one_p.u_min[i]  = (act_min[i] - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i] - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i]  - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
        break;         
      case COMMAND_PITCH:
        if (RW.skew.deg > 70.0){
          WLS_one_p.u_min[i]  = (RadOfDeg(-17.0) - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
          WLS_one_p.u_max[i]  = (RadOfDeg(17.0)  - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
        } else {
          WLS_one_p.u_min[i]  = (act_min[i] - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
          WLS_one_p.u_max[i]  = (act_max[i] - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
        }
        WLS_one_p.u_pref[i] = (u_pref[i]  - oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
        break;           
    }
  }
  // WLS Control Allocator
  normalize_nu();
  wls_alloc(&WLS_one_p, bwls_1l, 0, 0, 10);
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    andi_du_n[i] = WLS_one_p.u[i];
    andi_du[i] = (float)(andi_du_n[i] * ratio_u_un[i]);
  }

  if (in_flight_oneloop) {
    // Add the increments to the actuators
    float_vect_sum(andi_u, actuator_state_1l, andi_du, ANDI_NUM_ACT);
    andi_u[COMMAND_ROLL]  = andi_du[COMMAND_ROLL]  + oneloop_andi.sta_state.att[0];
    andi_u[COMMAND_PITCH] = andi_du[COMMAND_PITCH] + oneloop_andi.sta_state.att[1];
  } else {
    // Not in flight, so don't increment
    float_vect_copy(andi_u, andi_du, ANDI_NUM_ACT);
    andi_u[COMMAND_ROLL]  = andi_du[COMMAND_ROLL];
    andi_u[COMMAND_PITCH] = andi_du[COMMAND_PITCH];
  }
#ifdef COMMAND_MOTOR_PUSHER
  if ((half_loop)){
    andi_u[COMMAND_MOTOR_PUSHER] = radio_control.values[RADIO_AUX4];
  }
#endif  
  // TODO : USE THE PROVIDED MAX AND MIN and change limits for phi and theta
  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    Bound(andi_u[i], act_min[i], act_max[i]);
  }

  /*Commit the actuator command*/
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    commands[i] = (int16_t) andi_u[i];
  }
  if (rotwing_state.fail_pusher_motor){
    commands[COMMAND_MOTOR_PUSHER] = 0;//Min(1000,andi_u[COMMAND_MOTOR_PUSHER]);
  }
  commands[COMMAND_THRUST] = (commands[COMMAND_MOTOR_FRONT] + commands[COMMAND_MOTOR_RIGHT] + commands[COMMAND_MOTOR_BACK] + commands[COMMAND_MOTOR_LEFT])/num_thrusters_oneloop;
  autopilot.throttle = commands[COMMAND_THRUST];
  stabilization.cmd[COMMAND_THRUST] = commands[COMMAND_THRUST];
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    eulers_zxy_des.phi   =  andi_u[COMMAND_ROLL];
    eulers_zxy_des.theta =  andi_u[COMMAND_PITCH];
  } else {
    eulers_zxy_des.phi   =  andi_u[COMMAND_ROLL];
    eulers_zxy_des.theta =  andi_u[COMMAND_PITCH];
  }
  if (heading_manual){
    psi_des_deg = DegOfRad(psi_des_rad);
  } 

  stabilization.cmd[COMMAND_ROLL]  = (int16_t) (DegOfRad(eulers_zxy_des.phi  ) * MAX_PPRZ / DegOfRad(ONELOOP_ANDI_MAX_PHI  ));
  stabilization.cmd[COMMAND_PITCH] = (int16_t) (DegOfRad(eulers_zxy_des.theta) * MAX_PPRZ / DegOfRad(ONELOOP_ANDI_MAX_THETA));
  stabilization.cmd[COMMAND_YAW]   = (int16_t) (psi_des_deg * MAX_PPRZ / 180.0);
}

/** @brief  Function to reconstruct actuator state using first order dynamics */
void get_act_state_oneloop(void)
{
  int8_t i;
  float prev_actuator_state_1l;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    prev_actuator_state_1l = actuator_state_1l[i];
    actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (andi_u[i] - prev_actuator_state_1l);
    if(!autopilot_get_motors_on()){
      actuator_state_1l[i] = 0.0;
    }
    Bound(actuator_state_1l[i],act_min[i], act_max[i]);
  }
}

/**
 * @brief Function that samples and scales the effectiveness matrix
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */
void G1G2_oneloop(int ctrl_type) {
  int i = 0;
  float scaler = 1.0;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {

    switch (i) {
      case (COMMAND_MOTOR_FRONT):
      case (COMMAND_MOTOR_RIGHT):
      case (COMMAND_MOTOR_BACK):
      case (COMMAND_MOTOR_LEFT):
      case (COMMAND_MOTOR_PUSHER):
      case (COMMAND_ELEVATOR):
      case (COMMAND_RUDDER):  
      case (COMMAND_AILERONS):
      case (COMMAND_FLAPS):   
        if(ctrl_type == CTRL_ANDI){
          scaler = act_dynamics[i] * ratio_u_un[i];
        } else if (ctrl_type == CTRL_INDI){
          scaler = ratio_u_un[i];
        }
        break;
      case (COMMAND_ROLL):
      case (COMMAND_PITCH):
        if(ctrl_type == CTRL_ANDI){
          scaler = act_dynamics[i] * ratio_u_un[i];
        } else if (ctrl_type == CTRL_INDI){
          scaler = ratio_u_un[i];
        }
        break;
      default:
        break;        
    }
    int j = 0;
    bool turn_quad_off = ((!rotwing_state.hover_motors_enabled || !rotwing_state_hover_motors_running()) && rotwing_state.state != ROTWING_STATE_FORCE_HOVER);
    for (j = 0; j < ANDI_OUTPUTS; j++) {
      EFF_MAT_G[j][i] = EFF_MAT_RW[j][i] * scaler * ratio_vn_v[j];
      if (airspeed_filt.o[0] < ELE_MIN_AS && i == COMMAND_ELEVATOR){
        EFF_MAT_G[j][i] = 0.0;
      }
      if (turn_quad_off && i < 4){
        EFF_MAT_G[j][i] = 0.0;
      }
      if (ctrl_off && i < 4  && j == 5){ //hack test
        EFF_MAT_G[j][i] = 0.0;
      } 
      if (ctrl_off && i < 4  && j == 4){ //hack test
        EFF_MAT_G[j][i] = 0.0;
      } 
      if (ctrl_off && i < 4  && j == 3){ //hack test
        EFF_MAT_G[j][i] = 0.0;
      } 
    }
  }
}

/** @brief  Calculate Normalization of actuators and discrete actuator dynamics  */
void calc_normalization(void){
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    act_dynamics_d[i] = 1.0-exp(-act_dynamics[i]*dt_1l);
    Bound(act_dynamics_d[i],0.00001,1.0);
    Bound(act_max[i],0,MAX_PPRZ);
    Bound(act_min[i],-MAX_PPRZ,0);
    float ratio_numerator = act_max[i]-act_min[i];
    ratio_numerator = positive_non_zero(ratio_numerator);// make sure numerator is non-zero
    float ratio_denominator = act_max_norm[i]-act_min_norm[i];
    ratio_denominator = positive_non_zero(ratio_denominator); // make sure denominator is non-zero
    ratio_u_un[i] = ratio_numerator/ratio_denominator;
    ratio_u_un[i] = positive_non_zero(ratio_u_un[i]);// make sure ratio is not zero
  }
  for (i = 0; i < ANDI_OUTPUTS; i++){
    float ratio_numerator = positive_non_zero(nu_norm_max);
    float ratio_denominator = 1.0;
    switch (i) {
      case (RW_aN):
      case (RW_aE):
      case (RW_aD):
        ratio_denominator = positive_non_zero(max_j_nav);
        ratio_vn_v[i] = ratio_numerator/ratio_denominator;
        break;
      case (RW_ap):
      case (RW_aq):
      case (RW_ar):
        ratio_denominator = positive_non_zero(max_j_ang);
        ratio_vn_v[i] = ratio_numerator/ratio_denominator;
        break;
    }
  }
}

/** @brief  Function to normalize the pseudo control vector */
void normalize_nu(void){
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++){
    nu_n[i] = nu[i] * ratio_vn_v[i];
    WLS_one_p.v[i] = nu_n[i];
  }
}

/** @brief  Function that calculates the model prediction for the complementary filter. */
void calc_model(void){
  int8_t i;
  int8_t j;
  // // Absolute Model Prediction : 
  float sphi   = sinf(eulers_zxy.phi);
  float cphi   = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi   = sinf(eulers_zxy.psi);
  float cpsi   = cosf(eulers_zxy.psi);
  // Thrust and Pusher force estimation
  float L      = RW.wing.L / RW.m;          // Lift specific force
  float T      = RW.T / RW.m;             //  Thrust specific force. Minus gravity is a guesstimate.
  float P      = RW.P / RW.m;               // Pusher specific force

  cf.ax.model = -(cpsi * stheta + ctheta * sphi * spsi) * T + (cpsi * ctheta - sphi * spsi * stheta) * P - sphi * spsi * L;
  cf.ay.model = -(spsi * stheta - cpsi * ctheta * sphi) * T + (ctheta * spsi + cpsi * sphi * stheta) * P + cpsi * sphi * L;
  cf.az.model = g - cphi * ctheta * T - cphi * stheta * P - cphi * L;
  float model_pqr_dot[3] = {0.0, 0.0, 0.0};
  for (i = 0; i < 3; i++){ // For loop for prediction of angular acceleration 
    for (j = 0; j < ANDI_NUM_ACT; j++){
      if(j == COMMAND_ELEVATOR){
        model_pqr_dot[i] = model_pqr_dot[i] +  (actuator_state_1l[j] - RW.ele_pref) * EFF_MAT_RW[i+3][j]; // Ele pref is incidence angle
      } else {
        model_pqr_dot[i] = model_pqr_dot[i] +  actuator_state_1l[j] * EFF_MAT_RW[i+3][j];
      }
    }
  }
  cf.p_dot.model = model_pqr_dot[0];
  cf.q_dot.model = model_pqr_dot[1];
  cf.r_dot.model = model_pqr_dot[2];
}

/** @brief  Function that maps navigation inputs to the oneloop controller for the generated autopilot. */
void oneloop_from_nav(bool in_flight)
{
  if (!in_flight) {
    oneloop_andi_enter(false, oneloop_andi.ctrl_type);
  }
  struct FloatVect3 PSA_des;
  PSA_des.x = stateGetPositionNed_f()->x;
  PSA_des.y = stateGetPositionNed_f()->y;
  PSA_des.z = stateGetPositionNed_f()->z;
  int    rm_order_h = 3;
  int    rm_order_v = 3;
  // Oneloop controller wants desired targets and handles reference generation internally
  switch (nav.setpoint_mode) {
    case NAV_SETPOINT_MODE_POS:
      PSA_des.x   = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.y));
      PSA_des.y   = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.x));
      rm_order_h  = 3;
      break;
    case NAV_SETPOINT_MODE_SPEED:
      PSA_des.x   = SPEED_FLOAT_OF_BFP(SPEED_BFP_OF_REAL(nav.speed.y));
      PSA_des.y   = SPEED_FLOAT_OF_BFP(SPEED_BFP_OF_REAL(nav.speed.x));
      rm_order_h  = 2;
      break;
  }
  switch (nav.vertical_mode) {
    case NAV_VERTICAL_MODE_ALT:
      PSA_des.z   = POS_FLOAT_OF_BFP(-POS_BFP_OF_REAL(nav.nav_altitude));
      rm_order_v  = 3;
      break;
    case NAV_VERTICAL_MODE_CLIMB:
      PSA_des.z   = SPEED_FLOAT_OF_BFP(-SPEED_BFP_OF_REAL(nav.climb));
      rm_order_v  = 2;
      break;
  }
  oneloop_andi_run(in_flight, false, PSA_des, rm_order_h, rm_order_v);
}

/** @brief Function to calculate corrections for sideslip*/
float oneloop_andi_sideslip(void)
{
  // Coordinated turn
  // feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega = 0.0;
  const float max_phi = RadOfDeg(ONELOOP_ANDI_MAX_BANK);
  float airspeed_turn = airspeed_filt.o[0];
  Bound(airspeed_turn, 1.0f, 30.0f);
  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;
  // Prevent flipping
  if( (eulers_zxy.theta > 0.0f) && ( fabs(eulers_zxy.phi) < eulers_zxy.theta)) {
    coordinated_turn_roll = ((eulers_zxy.phi > 0.0f) - (eulers_zxy.phi < 0.0f)) * eulers_zxy.theta;
  }
  BoundAbs(coordinated_turn_roll, max_phi);
  omega = g / airspeed_turn * tanf(coordinated_turn_roll);
  #ifdef FWD_SIDESLIP_GAIN
  // Add sideslip correction
  omega -= accely_filt.o[0]*fwd_sideslip_gain;
  #endif
  return omega;
}

/** @brief Function to calculate the position reference during the chirp*/
static float chirp_pos_p_ref(float delta_t, float f0, float k, float A){
  float p_ref_fun = sinf(delta_t * M_PI * (f0 + k * delta_t) * 2.0);
  return (A * p_ref_fun);
}
/** @brief Function to calculate the velocity reference during the chirp*/
static float chirp_pos_v_ref(float delta_t, float f0, float k, float A){
  float v_ref_fun = cosf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * (M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0);
  return (A * v_ref_fun);
}
/** @brief Function to calculate the acceleration reference during the chirp*/
static float chirp_pos_a_ref(float delta_t, float f0, float k, float A){
  float a_ref_fun = -sinf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * pow((M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0), 2) + k * M_PI * cosf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * 4.0;
  return (A * a_ref_fun);
}
/** @brief Function to calculate the jerk reference during the chirp*/
static float chirp_pos_j_ref(float delta_t, float f0, float k, float A){
  float j_ref_fun = -cosf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * pow((M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0), 3) - k * M_PI * sinf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * (M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0) * 1.2e+1;
  return (A * j_ref_fun);
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt      [s]    time passed since start of the chirp
 * @param f0      [Hz]   initial frequency of the chirp
 * @param f1      [Hz]   final frequency of the chirp
 * @param t_chirp [s]    duration of the chirp
 * @param p_ref   [m]    position reference
 * @param v_ref   [m/s]  velocity reference
 * @param a_ref   [m/s2] acceleration reference
 * @param j_ref   [m/s3] jerk reference
 */
void chirp_pos(float time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]) {
  f0      = positive_non_zero(f0);
  f1      = positive_non_zero(f1);
  t_chirp = positive_non_zero(t_chirp);
  A       = positive_non_zero(A);
  if ((f1-f0) < -FLT_EPSILON){
    f1 = f0;
  }
  // 0 body x, 1 body y, 2 body z, 3 pitch pref
  if (n > 3){
    n = 0;
  }
  if (n < 0){
    n = 0;
  }
  // I think there should not be a problem with f1 being equal to f0
  float k = (f1 - f0) / t_chirp;
  float p_ref_chirp = chirp_pos_p_ref(time_elapsed, f0, k, A);
  float v_ref_chirp = chirp_pos_v_ref(time_elapsed, f0, k, A);
  float a_ref_chirp = chirp_pos_a_ref(time_elapsed, f0, k, A);
  float j_ref_chirp = chirp_pos_j_ref(time_elapsed, f0, k, A);

  float spsi   = sinf(psi);
  float cpsi   = cosf(psi);
  float mult_0 = 0.0;
  float mult_1 = 0.0;
  float mult_2 = 0.0;
  if (n == 0){
    mult_0 = cpsi;
    mult_1 = spsi;
    mult_2 = 0.0;
  }else if(n==1){
    mult_0 = -spsi;
    mult_1 = cpsi;
    mult_2 = 0.0;
  }else if(n==2){
    mult_0 = 0.0;
    mult_1 = 0.0;
    mult_2 = 1.0;
  }
  // Do not overwrite the reference if chirp is not on that axis
  if (n == 2){
    p_ref[2] = p_ref_0[2] + p_ref_chirp * mult_2;
    v_ref[2] = v_ref_chirp * mult_2;
    a_ref[2] = a_ref_chirp * mult_2;
    j_ref[2] = j_ref_chirp * mult_2;
  } else if (n < 2){
    p_ref[0] = p_ref_0[0] + p_ref_chirp * mult_0;
    p_ref[1] = p_ref_0[1] + p_ref_chirp * mult_1; 
    v_ref[0] = v_ref_chirp * mult_0;
    v_ref[1] = v_ref_chirp * mult_1;
    a_ref[0] = a_ref_chirp * mult_0;
    a_ref[1] = a_ref_chirp * mult_1; 
    j_ref[0] = j_ref_chirp * mult_0;
    j_ref[1] = j_ref_chirp * mult_1;
  } else { //Pitch preferred chirp, for now a little bit hacked in...
    pitch_pref = p_ref_chirp;
    pitch_pref = (pitch_pref / A + 1.0) * (theta_pref_max / 2.0);
    float pitch_offset = RadOfDeg(5.0);
    pitch_pref = pitch_pref + pitch_offset;
    Bound(pitch_pref,0.0,25.0);
  }
}

void chirp_call(bool *chirp_on, bool *chirp_first_call, float* t_0, float* time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]){
  if (*chirp_on){
    if (*chirp_first_call){
      *time_elapsed = 0.0;
      *chirp_first_call = false;
      *t_0 = get_sys_time_float();
      p_ref_0[0] = p_ref[0];
      p_ref_0[1] = p_ref[1];
      p_ref_0[2] = p_ref[2];
    }
    if (*time_elapsed < t_chirp){
      *time_elapsed = get_sys_time_float() - *t_0;
      chirp_pos(*time_elapsed, f0, f1, t_chirp, A, n, psi, p_ref, v_ref, a_ref, j_ref, p_ref_0);
    } else {
      *chirp_on   = false;
      *chirp_first_call = true;
      *time_elapsed = 0.0;
      *t_0 = 0.0;
      oneloop_andi_enter(false, oneloop_andi.ctrl_type);
    }
  }
}

/** Quadplanes can still be in-flight with COMMAND_THRUST==0 and can even soar not descending in updrafts with all thrust off */
bool autopilot_in_flight_end_detection(bool motors_on UNUSED) {
  return ! motors_on;
}


void reshape_wind(void)
{
  float psi = eulers_zxy.psi;
  float cpsi = cosf(psi);
  float spsi = sinf(psi);
  float airspeed = airspeed_filt.o[0];
  struct FloatVect2 NT_v_NE     = {nav_target[0], nav_target[1]}; // Nav target in North and East frame
  struct FloatVect2 airspeed_v  = { cpsi * airspeed, spsi * airspeed };
  struct FloatVect2 windspeed;
  struct FloatVect2 groundspeed = { oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.vel[1] };
  struct FloatVect2 des_as_NE;
  struct FloatVect2 des_as_B;
  struct FloatVect2 des_acc_B;
  VECT2_DIFF(windspeed, groundspeed, airspeed_v); // Wind speed in North and East frame
  VECT2_DIFF(des_as_NE, NT_v_NE, windspeed); // Desired airspeed in North and East frame
  float norm_des_as = FLOAT_VECT2_NORM(des_as_NE);
  gi_unbounded_airspeed_sp = norm_des_as;
  //Check if some minimum airspeed is desired (e.g. to prevent stall)
  if (norm_des_as < min_as) {
     norm_des_as = min_as;
  }
  nav_target_new[0] = NT_v_NE.x;
  nav_target_new[1] = NT_v_NE.y;
  // if the desired airspeed is larger than the max airspeed or we are in force forward reshape gs des to cancel wind and fly at max airspeed
  if ((norm_des_as > max_as)||(force_forward)){
    float groundspeed_factor = 0.0f;
    if (FLOAT_VECT2_NORM(windspeed) < max_as) {
      float av = NT_v_NE.x * NT_v_NE.x + NT_v_NE.y * NT_v_NE.y; // norm squared of nav target 
      float bv = -2.f * (windspeed.x * NT_v_NE.x + windspeed.y * NT_v_NE.y);
      float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - max_as * max_as;
      float dv = bv * bv - 4.0f * av * cv;
      // dv can only be positive, but just in case
      if (dv < 0.0f) {
        dv = fabsf(dv);
      }
      float d_sqrt = sqrtf(dv);
      groundspeed_factor = (-bv + d_sqrt)  / (2.0f * av); 
    }
    des_as_NE.x = groundspeed_factor * NT_v_NE.x - windspeed.x;
    des_as_NE.y = groundspeed_factor * NT_v_NE.y - windspeed.y;
    NT_v_NE.x   = groundspeed_factor * NT_v_NE.x;
    NT_v_NE.y   = groundspeed_factor * NT_v_NE.y; 
  }
  norm_des_as = FLOAT_VECT2_NORM(des_as_NE); // Recalculate norm of desired airspeed
  des_as_B.x  = norm_des_as; // Desired airspeed in body x frame
  des_as_B.y  = 0.0; // Desired airspeed in body y frame
  if (((airspeed > ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD) && (norm_des_as > (ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD+2.0f)))|| (force_forward)){
    float delta_psi = atan2f(des_as_NE.y, des_as_NE.x) - psi; 
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    des_acc_B.y = delta_psi * 5.0;//gih_params.heading_bank_gain;
    des_acc_B.x = (des_as_B.x - airspeed) * k_pos_rm.k2[0];//gih_params.speed_gain;
    acc_body_bound(&des_acc_B, max_a_nav); // Scale down side acceleration if norm is too large
    nav_target_new[0] = cpsi * des_acc_B.x - spsi * des_acc_B.y;
    nav_target_new[1] = spsi * des_acc_B.x + cpsi * des_acc_B.y; 
  } else {
    nav_target_new[0] = (NT_v_NE.x - groundspeed.x) * k_pos_rm.k2[0];
    nav_target_new[1] = (NT_v_NE.y - groundspeed.y) * k_pos_rm.k2[1];
  }
  vect_bound_nd(nav_target_new, max_a_nav, 2);
}

void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed) {
  min_as = min_airspeed;
  max_as = max_airspeed;
}


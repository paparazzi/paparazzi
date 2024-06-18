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
#include "math/wls/wls_alloc.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/rot_wing_drone/rotwing_state.h"
#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif
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

#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wv[ANDI_OUTPUTS]     = ONELOOP_ANDI_WV;
static float Wv_wls[ANDI_OUTPUTS] = ONELOOP_ANDI_WV;
#else
static float Wv[ANDI_OUTPUTS]     = {1.0};
static float Wv_wls[ANDI_OUTPUTS] = {1.0};
#endif

#ifdef ONELOOP_ANDI_WU // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
static float Wu[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_WU;
#else
static float Wu[ANDI_NUM_ACT_TOT] = {1.0};
#endif

#ifdef ONELOOP_ANDI_U_PREF
static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_U_PREF;
#else
static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
#endif

#ifndef ONELOOP_ANDI_DEBUG_MODE
#error "Debug Mode not defined"
#define ONELOOP_ANDI_DEBUG_MODE  FALSE;
#endif

#ifndef ONELOOP_ANDI_AC_HAS_PUSHER
#error "Did not specify if ac has a pusher"
#define ONELOOP_ANDI_AC_HAS_PUSHER  FALSE;
#endif

#if    !ONELOOP_ANDI_AC_HAS_PUSHER
#define ONELOOP_ANDI_PUSHER_IDX  0
#endif

#ifndef ONELOOP_ANDI_PUSHER_IDX
#error "Did not specify pusher index"
#define ONELOOP_ANDI_PUSHER_IDX  4
#endif

// Assume phi and theta are the first actuators after the real ones unless otherwise specified
#ifndef ONELOOP_ANDI_PHI_IDX
#define ONELOOP_ANDI_PHI_IDX  ANDI_NUM_ACT
#endif

#define ONELOOP_ANDI_MAX_BANK  act_max[ONELOOP_ANDI_PHI_IDX] // assuming abs of max and min is the same
#define ONELOOP_ANDI_MAX_PHI   act_max[ONELOOP_ANDI_PHI_IDX] // assuming abs of max and min is the same

#ifndef ONELOOP_ANDI_THETA_IDX
#define ONELOOP_ANDI_THETA_IDX  ANDI_NUM_ACT+1
#endif

#define ONELOOP_ANDI_MAX_THETA   act_max[ONELOOP_ANDI_THETA_IDX] // assuming abs of max and min is the same

#ifndef ONELOOP_THETA_PREF_MAX
float theta_pref_max = RadOfDeg(20.0);
#else
float theta_pref_max = RadOfDeg(ONELOOP_THETA_PREF_MAX);
#endif

#if ANDI_NUM_ACT_TOT != WLS_N_U
#error Matrix-WLS_N_U is not equal to the number of actuators: define WLS_N_U == ANDI_NUM_ACT_TOT in airframe file
#define WLS_N_U == ANDI_NUM_ACT_TOT
#endif
#if ANDI_OUTPUTS != WLS_N_V
#error Matrix-WLS_N_V is not equal to the number of controlled axis: define WLS_N_V == ANDI_OUTPUTS in airframe file
#define WLS_N_V == ANDI_OUTPUTS
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

#ifdef NAV_HYBRID_MAX_AIRSPEED
float max_v_nav = NAV_HYBRID_MAX_AIRSPEED; // Consider implications of difference Ground speed and airspeed
#else
float max_v_nav = 5.0;
#endif

#ifndef FWD_SIDESLIP_GAIN
float fwd_sideslip_gain = 1.0;
#else
float fwd_sideslip_gain = FWD_SIDESLIP_GAIN;
#endif

/*  Define Section of the functions used in this module*/
void  init_poles(void);
void  calc_normalization(void);
void  sum_g1g2_1l(int ctrl_type);
void  get_act_state_oneloop(void);
void  oneloop_andi_propagate_filters(void);
void  init_filter(void);
void  init_controller(void);
void  float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3]);
void  float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3]);
void  err_nd(float err[], float a[], float b[], float k[], int n);
void  integrate_nd(float dt, float a[], float a_dot[], int n);
void  vect_bound_nd(float vect[], float bound, int n);
void  rm_2nd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm);
void  rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3]);
void  rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n);
void  rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n);
void  rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n);
void  ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3]);
void  calc_model(int ctrl_type);
float oneloop_andi_sideslip(void);
void  chirp_pos(float time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  chirp_call(bool* chirp_on, bool* chirp_first_call, float* t_0_chirp, float* time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);

/*Define general struct of the Oneloop ANDI controller*/
struct OneloopGeneral oneloop_andi;

/* Oneloop Misc variables*/
static float use_increment = 0.0;
static float nav_target[3]; // Can be a position, speed or acceleration depending on the guidance H mode
static float dt_1l = 1./PERIODIC_FREQUENCY;
static float g   = 9.81; // [m/s^2] Gravitational Acceleration

/* Oneloop Control Variables*/
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
static float andi_du_n[ANDI_NUM_ACT_TOT];
float nu[ANDI_OUTPUTS];
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
bool heading_manual = true;
bool yaw_stick_in_auto = false;

/*WLS Settings*/
static float gamma_wls = 1000.0;
static float du_min_1l[ANDI_NUM_ACT_TOT]; 
static float du_max_1l[ANDI_NUM_ACT_TOT];
static float du_pref_1l[ANDI_NUM_ACT_TOT];
static float pitch_pref = 0;
static int   number_iter = 0;

/*Complementary Filter Variables*/
static float model_pred[ANDI_OUTPUTS];
static float ang_acc[3];
static float lin_acc[3];

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
float g2_1l[ANDI_NUM_ACT_TOT]               = ONELOOP_ANDI_G2; //scaled by ANDI_G_SCALING
float g1_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT] = {ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_THRUST, ONELOOP_ANDI_G1_ROLL, ONELOOP_ANDI_G1_PITCH, ONELOOP_ANDI_G1_YAW};  
float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float *bwls_1l[ANDI_OUTPUTS];
float ratio_u_un[ANDI_NUM_ACT_TOT];
float ratio_vn_v[ANDI_NUM_ACT_TOT];

/*Filters Initialization*/
static Butterworth2LowPass filt_accel_ned[3];                 // Low pass filter for acceleration NED (1)                       - oneloop_andi_filt_cutoff_a (tau_a)
static Butterworth2LowPass filt_veloc_ned[3];                 // Low pass filter for velocity NED                      - oneloop_andi_filt_cutoff_a (tau_a)       
static Butterworth2LowPass rates_filt_bt[3];                  // Low pass filter for angular rates                              - ONELOOP_ANDI_FILT_CUTOFF_P/Q/R
static Butterworth2LowPass model_pred_la_filt[3];             // Low pass filter for model prediction linear acceleration (1)   - oneloop_andi_filt_cutoff_a (tau_a)
static Butterworth2LowPass att_dot_meas_lowpass_filters[3];   // Low pass filter for attitude derivative measurements           - oneloop_andi_filt_cutoff (tau)
static Butterworth2LowPass model_pred_aa_filt[3];             // Low pass filter for model prediction angular acceleration      - oneloop_andi_filt_cutoff (tau)
static Butterworth2LowPass accely_filt;                       // Low pass filter for acceleration in y direction                - oneloop_andi_filt_cutoff (tau)
static Butterworth2LowPass airspeed_filt;                     // Low pass filter for airspeed                                   - oneloop_andi_filt_cutoff (tau)

/* Define messages of the module*/
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_eff_mat_g_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0;
  pprz_msg_send_EFF_MAT_G(trans, dev, AC_ID, 
                ANDI_NUM_ACT_TOT, g1g2_1l[0],
                ANDI_NUM_ACT_TOT, g1g2_1l[1],
                ANDI_NUM_ACT_TOT, g1g2_1l[2],
                ANDI_NUM_ACT_TOT, g1g2_1l[3],
                ANDI_NUM_ACT_TOT, g1g2_1l[4],
                ANDI_NUM_ACT_TOT, g1g2_1l[5], 
                                    1, &zero,
                                    1, &zero);
}
static void send_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                        &oneloop_andi.sta_state.att[0],
                                        &oneloop_andi.sta_state.att[1],
                                        &oneloop_andi.sta_state.att[2],
                                        &oneloop_andi.sta_ref.att[0],
                                        &oneloop_andi.sta_ref.att[1],
                                        &oneloop_andi.sta_ref.att[2],
                                        &oneloop_andi.sta_state.att_d[0],
                                        &oneloop_andi.sta_state.att_d[1],
                                        &oneloop_andi.sta_state.att_d[2],
                                        &oneloop_andi.sta_ref.att_d[0],
                                        &oneloop_andi.sta_ref.att_d[1],
                                        &oneloop_andi.sta_ref.att_d[2],                                       
                                        &oneloop_andi.sta_state.att_2d[0],
                                        &oneloop_andi.sta_state.att_2d[1],
                                        &oneloop_andi.sta_state.att_2d[2],
                                        &oneloop_andi.sta_ref.att_2d[0],
                                        &oneloop_andi.sta_ref.att_2d[1],
                                        &oneloop_andi.sta_ref.att_2d[2],                                        
                                        ANDI_OUTPUTS, nu,
                                        ANDI_NUM_ACT, actuator_state_1l);                                      
}

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
  float temp_debug_vect[2];
  temp_debug_vect[0] = andi_u[ONELOOP_ANDI_THETA_IDX];
  temp_debug_vect[1] = pitch_pref;
  debug_vect(trans, dev, "andi_u_pitch_pref", temp_debug_vect, 2);
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
void rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3]){
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
  if(ow_psi){x_3d_ref[2] = psi_overwrite[3];}
  integrate_nd(dt, x_2d_ref, x_3d_ref, 3);
  if(ow_psi){x_2d_ref[2] = psi_overwrite[2];}
  integrate_nd(dt, x_d_ref, x_2d_ref, 3);
  if(ow_psi){x_d_ref[2] = psi_overwrite[1];}
  float_euler_dot_of_rates_vec(x_d_ref, x_ref, x_d_eul_ref);
  integrate_nd(dt, x_ref, x_d_eul_ref, 3);
  if(ow_psi){x_ref[2] = psi_overwrite[0];}
  NormRadAngle(x_ref[2]);
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
  vect_bound_nd(e_x,x_d_bound, n);
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
void ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3]){
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
 * @brief Initialize Position of Poles
 * 
 */
void init_poles(void){

  // Attitude Controller Poles----------------------------------------------------------
  float slow_pole = 10.1; // Pole of the slowest dynamics used in the attitude controller

  p_att_e.omega_n = 4.50;
  p_att_e.zeta    = 1.0;
  p_att_e.p3      = slow_pole;

  p_att_rm.omega_n = 4.71;
  p_att_rm.zeta    = 1.0;
  p_att_rm.p3      = p_att_rm.omega_n * p_att_rm.zeta;

  p_head_e.omega_n = 1.80;
  p_head_e.zeta    = 1.0;
  p_head_e.p3      = slow_pole;

  p_head_rm.omega_n = 2.56;
  p_head_rm.zeta    = 1.0;
  p_head_rm.p3      = p_head_rm.omega_n * p_head_rm.zeta;

  act_dynamics[ONELOOP_ANDI_PHI_IDX]   = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[ONELOOP_ANDI_THETA_IDX] = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);

  // Position Controller Poles----------------------------------------------------------
  slow_pole = act_dynamics[ONELOOP_ANDI_PHI_IDX]; // Pole of the slowest dynamics used in the position controller

  p_pos_e.omega_n = 1.0;
  p_pos_e.zeta    = 1.0; 
  p_pos_e.p3      = slow_pole; 

  p_pos_rm.omega_n = 0.93;
  p_pos_rm.zeta    = 1.0;  
  p_pos_rm.p3      = p_pos_rm.omega_n * p_pos_rm.zeta;

  p_alt_e.omega_n = 3.0;
  p_alt_e.zeta    = 1.0; 
  p_alt_e.p3      = slow_pole; 

  p_alt_rm.omega_n = 1.93;
  p_alt_rm.zeta    = 1.0;
  p_alt_rm.p3      = p_alt_rm.omega_n * p_alt_rm.zeta;
}
/** 
 * @brief Initialize Controller Gains
 * FIXME: Calculate the gains dynamically for transition
 */
void init_controller(void){
  /*Register a variable from nav_hybrid. Should be improved when nav hybrid is final.*/
  max_v_nav = nav_max_speed;
  /*Some calculations in case new poles have been specified*/
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

  /*Heading Loop NAV*/
  k_att_e.k1[2]  = k_e_1_3_f_v2(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k2[2]  = k_e_2_3_f_v2(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k3[2]  = k_e_3_3_f_v2(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);

  k_att_rm.k1[2] = k_rm_1_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k2[2] = k_rm_2_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k3[2] = k_rm_3_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);

  /*Position Loop*/
  k_pos_e.k1[0]  = k_e_1_3_f_v2(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  k_pos_e.k2[0]  = k_e_2_3_f_v2(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  k_pos_e.k3[0]  = k_e_3_3_f_v2(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
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
  k_pos_e.k1[2]  = k_e_1_3_f_v2(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  k_pos_e.k2[2]  = k_e_2_3_f_v2(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  k_pos_e.k3[2]  = k_e_3_3_f_v2(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
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
  act_dynamics[ONELOOP_ANDI_PHI_IDX]   = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[ONELOOP_ANDI_THETA_IDX] = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
}


/** @brief  Initialize the filters */
void init_filter(void)
{
  float tau   = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff);
  float tau_a = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_a);
  float tau_v = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_v);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  // Filtering of the Inputs with 3 dimensions (e.g. rates and accelerations)
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&model_pred_aa_filt[i],           tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned[i],               tau_a, sample_time, 0.0 );
    init_butterworth_2_low_pass(&filt_veloc_ned[i],               tau_v, sample_time, 0.0 );
    init_butterworth_2_low_pass(&model_pred_la_filt[i],           tau_a, sample_time, 0.0); 
  }

  // Init rate filter for feedback
  float time_constants[3] = {1.0 / (2 * M_PI * oneloop_andi_filt_cutoff_p), 1.0 / (2 * M_PI * oneloop_andi_filt_cutoff_q), 1.0 / (2 * M_PI * oneloop_andi_filt_cutoff_r)};
  init_butterworth_2_low_pass(&rates_filt_bt[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_butterworth_2_low_pass(&rates_filt_bt[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_butterworth_2_low_pass(&rates_filt_bt[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);
  
  // Some other filters
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&airspeed_filt, tau, sample_time, 0.0);
}


/** @brief  Propagate the filters */
void oneloop_andi_propagate_filters(void) {
  struct  NedCoor_f *accel = stateGetAccelNed_f();
  struct  NedCoor_f *veloc = stateGetSpeedNed_f();
  struct  FloatRates *body_rates = stateGetBodyRates_f();
  float   rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z); 
  update_butterworth_2_low_pass(&filt_veloc_ned[0], veloc->x);
  update_butterworth_2_low_pass(&filt_veloc_ned[1], veloc->y);
  update_butterworth_2_low_pass(&filt_veloc_ned[2], veloc->z); 
  calc_model(oneloop_andi.ctrl_type);
  int8_t i;

  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&model_pred_aa_filt[i], model_pred[3+i]);
    update_butterworth_2_low_pass(&model_pred_la_filt[i],   model_pred[i]);
    update_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&rates_filt_bt[i], rate_vect[i]);

    ang_acc[i] = (att_dot_meas_lowpass_filters[i].o[0]- att_dot_meas_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY + model_pred[3+i] - model_pred_aa_filt[i].o[0];
    lin_acc[i] = filt_accel_ned[i].o[0] + model_pred[i] - model_pred_la_filt[i].o[0];     
  }
  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);
  float airspeed_meas = stateGetAirspeed_f();
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
  sum_g1g2_1l(oneloop_andi.ctrl_type);
  for (i = 0; i < ANDI_OUTPUTS; i++) {
   bwls_1l[i] = g1g2_1l[i];
  }

  // Initialize filters and other variables
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
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(nav_target,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  float_vect_zero(model_pred,ANDI_OUTPUTS);

  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_G, send_eff_mat_g_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE, send_guidance_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_oneloop_debug);
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
  oneloop_andi.half_loop     = half_loop_sp;
  oneloop_andi.ctrl_type     = ctrl_type;
  psi_des_rad   = eulers_zxy.psi; 
  psi_des_deg   = DegOfRad(eulers_zxy.psi);
  calc_normalization();
  sum_g1g2_1l(oneloop_andi.ctrl_type);
  init_filter();
  init_controller();
  /* Stabilization Reset */
  float_vect_zero(oneloop_andi.sta_ref.att,2);
  float_vect_zero(oneloop_andi.sta_ref.att_d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_2d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_3d,3);
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(nav_target,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  psi_des_rad;
  float_vect_zero(model_pred,ANDI_OUTPUTS);
  /*Guidance Reset*/
}

/**
 * @brief  Function to generate the reference signals for the oneloop controller
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param PSA_des    Desired position/speed/acceleration
 * @param rm_order_h Order of the reference model for horizontal guidance
 * @param rm_order_v Order of the reference model for vertical guidance
 */
void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v)
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
    Wv_wls[0] = 0.0;
    Wv_wls[1] = 0.0;
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
    if (fabs(delta_psi_rad) > RadOfDeg(10.0)){                                // If difference is bigger than 10 deg do not further increment desired
      delta_psi_des_rad = 0.0;
    }
    psi_des_rad += delta_psi_des_rad;                                         // Incrementdesired yaw
    NormRadAngle(psi_des_rad);
    eulers_zxy_des.psi = psi_des_rad;
    // Register Attitude Setpoints from previous loop
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
    // Create commands adhoc to get actuators to the wanted level
    thrust_cmd_1l = (float) radio_control_get(RADIO_THROTTLE);
    Bound(thrust_cmd_1l,0.0,MAX_PPRZ); 
    int8_t i;
    for (i = 0; i < ANDI_NUM_ACT; i++) {
     a_thrust +=(thrust_cmd_1l - use_increment*actuator_state_1l[i]) * bwls_1l[2][i] / (ratio_u_un[i] * ratio_vn_v[i]);
    }
    rm_3rd_attitude(dt_1l, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, att_des, false, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3);
  }else{
    // Make sure X and Y jerk objectives are active
    Wv_wls[0] = Wv[0];
    Wv_wls[1] = Wv[1];
    // Generate Reference signals for positioning using RM
    if (rm_order_h == 3){
      rm_3rd_pos(dt_1l, oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target, k_pos_rm.k1, k_pos_rm.k2, k_pos_rm.k3, max_v_nav, max_a_nav, max_j_nav, 2);    
    } else if (rm_order_h == 2){
      float_vect_copy(oneloop_andi.gui_ref.pos, oneloop_andi.gui_state.pos,2);
      rm_2nd_pos(dt_1l, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target, k_pos_rm.k2, k_pos_rm.k3, max_a_nav, max_j_nav, 2);   
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
      if (ref_mag_vel > 3.0){
        psi_des_rad = atan2f(oneloop_andi.gui_ref.vel[1],oneloop_andi.gui_ref.vel[0]);
      }
      psi_des_rad += oneloop_andi_sideslip() * dt_1l;
      NormRadAngle(psi_des_rad);
    }
    // Register Attitude Setpoints from previous loop
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, psi_des_rad};
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
      rm_3rd_pos(dt_1l, single_value_ref, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k1_rm, single_value_k2_rm, single_value_k3_rm, max_v_nav, max_a_nav, max_j_nav, 1);    
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
    rm_3rd_attitude(dt_1l, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, att_des, ow_psi, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3);
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
  sum_g1g2_1l(oneloop_andi.ctrl_type);
  
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
  oneloop_andi.sta_state.att[0]    = eulers_zxy.phi        * use_increment;
  oneloop_andi.sta_state.att[1]    = eulers_zxy.theta      * use_increment;
  oneloop_andi.sta_state.att[2]    = eulers_zxy.psi        * use_increment;
  oneloop_andi_propagate_filters();   //needs to be after update of attitude vector
  oneloop_andi.sta_state.att_d[0]  = rates_filt_bt[0].o[0] * use_increment;
  oneloop_andi.sta_state.att_d[1]  = rates_filt_bt[1].o[0] * use_increment;
  oneloop_andi.sta_state.att_d[2]  = rates_filt_bt[2].o[0] * use_increment;
  oneloop_andi.sta_state.att_2d[0] = ang_acc[0]            * use_increment;
  oneloop_andi.sta_state.att_2d[1] = ang_acc[1]            * use_increment;
  oneloop_andi.sta_state.att_2d[2] = ang_acc[2]            * use_increment;
  // (2) Position related
  oneloop_andi.gui_state.pos[0] = stateGetPositionNed_f()->x;   
  oneloop_andi.gui_state.pos[1] = stateGetPositionNed_f()->y;   
  oneloop_andi.gui_state.pos[2] = stateGetPositionNed_f()->z;   
  oneloop_andi.gui_state.vel[0] = filt_veloc_ned[0].o[0];      
  oneloop_andi.gui_state.vel[1] = filt_veloc_ned[1].o[0];      
  oneloop_andi.gui_state.vel[2] = filt_veloc_ned[2].o[0];      
  oneloop_andi.gui_state.acc[0] = lin_acc[0];
  oneloop_andi.gui_state.acc[1] = lin_acc[1];
  oneloop_andi.gui_state.acc[2] = lin_acc[2];
  
  // Update the effectiveness matrix used in WLS
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = g1g2_1l[i];
  }
  // Calculated feedforward signal for yaW CONTROL
  g2_ff = 0.0;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    if (oneloop_andi.ctrl_type == CTRL_ANDI){
      g2_ff += g2_1l[i] * act_dynamics[i] * andi_du[i];
    } else if (oneloop_andi.ctrl_type == CTRL_INDI){
      g2_ff += g2_1l[i]* andi_du_n[i];
    }
  }
  //G2 is scaled by ANDI_G_SCALING to make it readable
  g2_ff = g2_ff / ANDI_G_SCALING;
  // Run the Reference Model (RM)
  oneloop_andi_RM(half_loop, PSA_des, rm_order_h, rm_order_v);

  // Guidance Pseudo Control Vector (nu) based on error controller
  if(half_loop){
    nu[0] = 0.0;
    nu[1] = 0.0;
    nu[2] = a_thrust;
  }else{
    if(oneloop_andi.ctrl_type == CTRL_ANDI){
      nu[0] = ec_3rd(oneloop_andi.gui_ref.pos[0], oneloop_andi.gui_ref.vel[0], oneloop_andi.gui_ref.acc[0], oneloop_andi.gui_ref.jer[0], oneloop_andi.gui_state.pos[0], oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.acc[0], k_pos_e.k1[0], k_pos_e.k2[0], k_pos_e.k3[0]);
      nu[1] = ec_3rd(oneloop_andi.gui_ref.pos[1], oneloop_andi.gui_ref.vel[1], oneloop_andi.gui_ref.acc[1], oneloop_andi.gui_ref.jer[1], oneloop_andi.gui_state.pos[1], oneloop_andi.gui_state.vel[1], oneloop_andi.gui_state.acc[1], k_pos_e.k1[1], k_pos_e.k2[1], k_pos_e.k3[1]);
      nu[2] = ec_3rd(oneloop_andi.gui_ref.pos[2], oneloop_andi.gui_ref.vel[2], oneloop_andi.gui_ref.acc[2], oneloop_andi.gui_ref.jer[2], oneloop_andi.gui_state.pos[2], oneloop_andi.gui_state.vel[2], oneloop_andi.gui_state.acc[2], k_pos_e.k1[2], k_pos_e.k2[2], k_pos_e.k3[2]); 
    } else if (oneloop_andi.ctrl_type == CTRL_INDI){
      nu[0] = ec_3rd(oneloop_andi.gui_ref.pos[0], oneloop_andi.gui_ref.vel[0], oneloop_andi.gui_ref.acc[0], 0.0, oneloop_andi.gui_state.pos[0], oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.acc[0], k_pos_e_indi.k1[0], k_pos_e_indi.k2[0], k_pos_e_indi.k3[0]);
      nu[1] = ec_3rd(oneloop_andi.gui_ref.pos[1], oneloop_andi.gui_ref.vel[1], oneloop_andi.gui_ref.acc[1], 0.0, oneloop_andi.gui_state.pos[1], oneloop_andi.gui_state.vel[1], oneloop_andi.gui_state.acc[1], k_pos_e_indi.k1[1], k_pos_e_indi.k2[1], k_pos_e_indi.k3[1]);
      nu[2] = ec_3rd(oneloop_andi.gui_ref.pos[2], oneloop_andi.gui_ref.vel[2], oneloop_andi.gui_ref.acc[2], 0.0, oneloop_andi.gui_state.pos[2], oneloop_andi.gui_state.vel[2], oneloop_andi.gui_state.acc[2], k_pos_e_indi.k1[2], k_pos_e_indi.k2[2], k_pos_e_indi.k3[2]);  
    }
  }

  // Attitude Pseudo Control Vector (nu) based on error controller
  float y_4d_att[3];  
  if(oneloop_andi.ctrl_type == CTRL_ANDI){
    ec_3rd_att(y_4d_att, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, oneloop_andi.sta_state.att, oneloop_andi.sta_state.att_d, oneloop_andi.sta_state.att_2d, k_att_e.k1, k_att_e.k2, k_att_e.k3);
  } else if (oneloop_andi.ctrl_type == CTRL_INDI){
    float dummy0[3] = {0.0, 0.0, 0.0};
    ec_3rd_att(y_4d_att, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, dummy0, oneloop_andi.sta_state.att, oneloop_andi.sta_state.att_d, oneloop_andi.sta_state.att_2d, k_att_e_indi.k1, k_att_e_indi.k2, k_att_e_indi.k3);
  }
  nu[3] = y_4d_att[0];  
  nu[4] = y_4d_att[1]; 
  nu[5] = y_4d_att[2] + g2_ff; 
  if (!chirp_on){
    pitch_pref = radio_control.values[RADIO_AUX5]; 
    pitch_pref = pitch_pref / MAX_PPRZ * theta_pref_max;
    Bound(pitch_pref,0.0,theta_pref_max);
  }
  u_pref[ONELOOP_ANDI_THETA_IDX] = pitch_pref;
  // Calculate the min and max increments
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    if(i<ANDI_NUM_ACT){
      du_min_1l[i]  = (act_min[i] - use_increment * actuator_state_1l[i])/ratio_u_un[i];
      du_max_1l[i]  = (act_max[i] - use_increment * actuator_state_1l[i])/ratio_u_un[i];
      du_pref_1l[i] = (u_pref[i]  - use_increment * actuator_state_1l[i])/ratio_u_un[i];
    }else{
      du_min_1l[i]  = (act_min[i] - use_increment * oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
      du_max_1l[i]  = (act_max[i] - use_increment * oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
      du_pref_1l[i] = (u_pref[i]  - use_increment * oneloop_andi.sta_state.att[i-ANDI_NUM_ACT])/ratio_u_un[i];
    }
  }
 
  // WLS Control Allocator
  number_iter = wls_alloc(andi_du_n, nu, du_min_1l, du_max_1l, bwls_1l, 0, 0, Wv_wls, Wu, du_pref_1l, gamma_wls, 10, ANDI_NUM_ACT_TOT, ANDI_OUTPUTS);

  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    andi_du[i] = (float)(andi_du_n[i] * ratio_u_un[i]);
  }

  if (in_flight_oneloop) {
    // Add the increments to the actuators
    float_vect_sum(andi_u, actuator_state_1l, andi_du, ANDI_NUM_ACT);
    andi_u[ONELOOP_ANDI_PHI_IDX]   = andi_du[ONELOOP_ANDI_PHI_IDX]   + oneloop_andi.sta_state.att[0];
    andi_u[ONELOOP_ANDI_THETA_IDX] = andi_du[ONELOOP_ANDI_THETA_IDX] + oneloop_andi.sta_state.att[1];
  } else {
    // Not in flight, so don't increment
    float_vect_copy(andi_u, andi_du, ANDI_NUM_ACT);
    andi_u[ONELOOP_ANDI_PHI_IDX] = andi_du[ONELOOP_ANDI_PHI_IDX];
    andi_u[ONELOOP_ANDI_THETA_IDX] = andi_du[ONELOOP_ANDI_THETA_IDX];
  }

  if ((ONELOOP_ANDI_AC_HAS_PUSHER)&&(half_loop)){
    andi_u[ONELOOP_ANDI_PUSHER_IDX] = radio_control.values[RADIO_AUX4];
  }
  // TODO : USE THE PROVIDED MAX AND MIN and change limits for phi and theta
  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    Bound(andi_u[i], act_min[i], act_max[i]);
  }

  /*Commit the actuator command*/
  stabilization.cmd[COMMAND_THRUST] = 0;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) andi_u[i];
    stabilization.cmd[COMMAND_THRUST] += actuator_state_1l[i];
  }
  stabilization.cmd[COMMAND_THRUST] = stabilization.cmd[COMMAND_THRUST]/num_thrusters_oneloop;
  autopilot.throttle = stabilization_cmd[COMMAND_THRUST];

  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    eulers_zxy_des.phi   =  andi_u[ONELOOP_ANDI_PHI_IDX];
    eulers_zxy_des.theta =  andi_u[ONELOOP_ANDI_THETA_IDX];
  } else {
    eulers_zxy_des.phi   =  andi_u[ONELOOP_ANDI_PHI_IDX];
    eulers_zxy_des.theta =  andi_u[ONELOOP_ANDI_THETA_IDX];
  }
  if (heading_manual){
    psi_des_deg = DegOfRad(psi_des_rad);
  }  
  stabilization_cmd[COMMAND_ROLL]  = (int16_t) (DegOfRad(eulers_zxy_des.phi  ) * MAX_PPRZ / DegOfRad(ONELOOP_ANDI_MAX_PHI  ));
  stabilization_cmd[COMMAND_PITCH] = (int16_t) (DegOfRad(eulers_zxy_des.theta) * MAX_PPRZ / DegOfRad(ONELOOP_ANDI_MAX_THETA));
  stabilization_cmd[COMMAND_YAW]   = (int16_t) (psi_des_deg * MAX_PPRZ / 180.0);
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
    Bound(actuator_state_1l[i],0,MAX_PPRZ);
  }
}

/**
 * @brief Function that sums g1 and g2 to obtain the g1_g2 matrix. It also undoes the scaling that was done to make the values readable
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */
void sum_g1g2_1l(int ctrl_type) {
  int i = 0;
  
  // Trig of attitude angles
  float sphi   = sinf(eulers_zxy.phi);
  float cphi   = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi   = sinf(eulers_zxy.psi);
  float cpsi   = cosf(eulers_zxy.psi);
  // Trig of skew
  // float skew   = 0.0;
  // if (ONELOOP_ANDI_SCHEDULING){
  //   skew   = rotwing_state_skewing.wing_angle_deg;
  // }
  // float sskew  = sinf(skew);
  // float cskew  = cosf(skew);
  // float s2skew = sskew * sskew;
  // float c2skew = cskew * cskew;
  // float s3skew = sskew * s2skew;
  // float c3skew = cskew * c2skew;

  // Thrust and Pusher force estimation
  float T      = -g/(cphi*ctheta);//minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float P      = 0.0;
  if (ONELOOP_ANDI_AC_HAS_PUSHER){
    P    = actuator_state_1l[ONELOOP_ANDI_PUSHER_IDX] * g1_1l[2][ONELOOP_ANDI_PUSHER_IDX] / ANDI_G_SCALING;
  } 
  float scaler  = 1.0;
  float sched_p = 1.0; // Scheduler variable for roll axis 
  float sched_q = 1.0; // Scheduler variable for pitch axis
  //float I_com
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    // Effectiveness vector for real actuators (e.g. motors, servos)
    if (i < ANDI_NUM_ACT){
      if(ctrl_type == CTRL_ANDI){
        scaler = act_dynamics[i] * ratio_u_un[i] * ratio_vn_v[i] / ANDI_G_SCALING;
      } else if (ctrl_type == CTRL_INDI){
        scaler = ratio_u_un[i] * ratio_vn_v[i] / ANDI_G_SCALING;
      }  
      // switch (i){
      //   case 1: 
      //     sched_p = cskew;
      //     sched_q = sskew * g1_1l[4][0] / g1_1l[4][1] * 0.9645; // sin(skew)*mF_effctiveness*ratio_of_arms
      //     break; 
      //   case 3: 
      //     sched_p = cskew;
      //     sched_q = sskew * g1_1l[4][2] / g1_1l[4][3] * 0.9645; // sin(skew)*mF_effctiveness*ratio_of_arms
      //     break;
      //   default:
      //     sched_p = 1.0;
      //     sched_q = 1.0;
      //     break;
      // }
      g1g2_1l[0][i] = (cpsi * stheta + ctheta * sphi * spsi) * g1_1l[2][i] * scaler;
      g1g2_1l[1][i] = (spsi * stheta - cpsi * ctheta * sphi) * g1_1l[2][i] * scaler;
      g1g2_1l[2][i] = (cphi * ctheta                       ) * g1_1l[2][i] * scaler;
      g1g2_1l[3][i] = (g1_1l[3][i])                                        * scaler * sched_p;
      g1g2_1l[4][i] = (g1_1l[4][i])                                        * scaler * sched_q;
      g1g2_1l[5][i] = (g1_1l[5][i] + g2_1l[i])                             * scaler;
      if ((ONELOOP_ANDI_AC_HAS_PUSHER)&&(i==ONELOOP_ANDI_PUSHER_IDX)){
        g1g2_1l[0][i] = (cpsi * ctheta - sphi * spsi * stheta) * g1_1l[2][i] * scaler;
        g1g2_1l[1][i] = (ctheta * spsi + cpsi * sphi * stheta) * g1_1l[2][i] * scaler;
        g1g2_1l[2][i] = (- cphi * stheta                     ) * g1_1l[2][i] * scaler;
        g1g2_1l[3][i] = 0.0;
        g1g2_1l[4][i] = 0.0;
        g1g2_1l[5][i] = 0.0;
      }
    }else{
      if(ctrl_type == CTRL_ANDI){
        scaler = act_dynamics[i] * ratio_u_un[i] * ratio_vn_v[i];
      } else if (ctrl_type == CTRL_INDI){
        scaler = ratio_u_un[i] * ratio_vn_v[i];
      }
      // Effectiveness vector for Phi (virtual actuator)
      if (i == ONELOOP_ANDI_PHI_IDX){
        g1g2_1l[0][i] = ( cphi * ctheta * spsi * T - cphi * spsi * stheta * P) * scaler;
        g1g2_1l[1][i] = (-cphi * ctheta * cpsi * T + cphi * cpsi * stheta * P) * scaler;
        g1g2_1l[2][i] = (-sphi * ctheta * T + sphi * stheta * P)               * scaler;
        g1g2_1l[3][i] = 0.0;
        g1g2_1l[4][i] = 0.0;
        g1g2_1l[5][i] = 0.0;
      }
      // Effectiveness vector for Theta (virtual actuator)
      if (i == ONELOOP_ANDI_THETA_IDX){
        g1g2_1l[0][i] = ((ctheta*cpsi - sphi*stheta*spsi) * T - (cpsi * stheta + ctheta * sphi * spsi) * P) * scaler;
        g1g2_1l[1][i] = ((ctheta*spsi + sphi*stheta*cpsi) * T - (spsi * stheta - cpsi * ctheta * sphi) * P) * scaler;
        g1g2_1l[2][i] = (-stheta * cphi * T - cphi * ctheta * P)                                            * scaler;
        g1g2_1l[3][i] = 0.0;
        g1g2_1l[4][i] = 0.0;
        g1g2_1l[5][i] = 0.0;
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
    ratio_vn_v[i] = 1.0;
    Bound(act_max[i],0,MAX_PPRZ);
    Bound(act_min[i],-MAX_PPRZ,0);
    float ratio_numerator = act_max[i]-act_min[i];
    ratio_numerator = positive_non_zero(ratio_numerator);// make sure numerator is non-zero
    float ratio_denominator = act_max_norm[i]-act_min_norm[i];
    ratio_denominator = positive_non_zero(ratio_denominator); // make sure denominator is non-zero
    ratio_u_un[i] = ratio_numerator/ratio_denominator;
    ratio_u_un[i] = positive_non_zero(ratio_u_un[i]);// make sure ratio is not zero
  }
}

/** @brief  Function that calculates the model prediction for the complementary filter. */
void calc_model(int ctrl_type){
  int8_t i;
  int8_t j;
  // Absolute Model Prediction : 
  float sphi   = sinf(eulers_zxy.phi);
  float cphi   = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi   = sinf(eulers_zxy.psi);
  float cpsi   = cosf(eulers_zxy.psi);

  float T      = -g/(cphi*ctheta); // -9.81;
  float P      = 0.0;
  if (ONELOOP_ANDI_AC_HAS_PUSHER){  
   P      = actuator_state_1l[ONELOOP_ANDI_PUSHER_IDX] * g1_1l[2][ONELOOP_ANDI_PUSHER_IDX] / ANDI_G_SCALING;
  }
  model_pred[0] = (cpsi * stheta + ctheta * sphi * spsi) * T + (cpsi * ctheta - sphi * spsi * stheta) * P;
  model_pred[1] = (spsi * stheta - cpsi * ctheta * sphi) * T + (ctheta * spsi + cpsi * sphi * stheta) * P;
  model_pred[2] = g + cphi * ctheta * T - cphi * stheta * P;

  for (i = 3; i < ANDI_OUTPUTS; i++){ // For loop for prediction of angular acceleration
    model_pred[i] = 0.0;              // 
    for (j = 0; j < ANDI_NUM_ACT; j++){
      if (ctrl_type == CTRL_ANDI){
        model_pred[i] = model_pred[i] +  actuator_state_1l[j] * g1g2_1l[i][j] / (act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j]);
      } else if (ctrl_type == CTRL_INDI){
        model_pred[i] = model_pred[i] +  actuator_state_1l[j] * g1g2_1l[i][j] / (ratio_u_un[j] * ratio_vn_v[j]);
      }
    }
  }
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
  float omega;
  const float max_phi = RadOfDeg(ONELOOP_ANDI_MAX_BANK);
  float airspeed_turn = airspeed_filt.o[0];
  Bound(airspeed_turn, 10.0f, 30.0f);
  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;
  // Prevent flipping
  if( (andi_u[ONELOOP_ANDI_THETA_IDX] > 0.0f) && ( fabs(andi_u[ONELOOP_ANDI_PHI_IDX]) < andi_u[ONELOOP_ANDI_THETA_IDX])) {
    coordinated_turn_roll = ((andi_u[ONELOOP_ANDI_PHI_IDX] > 0.0f) - (andi_u[ONELOOP_ANDI_PHI_IDX] < 0.0f)) * andi_u[ONELOOP_ANDI_THETA_IDX];
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
 * @brief Function to perform position and attitude chirps
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
  // i think there should not be a problem with f1 being equal to f0
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
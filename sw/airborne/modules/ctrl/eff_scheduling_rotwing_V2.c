/*
 * Copyright (C) 2023 Tomaso De Ponti <T.M.L.DePonti@tudelft.nl>
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

/** @file "modules/ctrl/eff_scheduling_rotwing_V2.c"
 * @author Tomaso De Ponti <T.M.L.DePonti@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#include "modules/ctrl/eff_scheduling_rotwing_V2.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "modules/ins/ins_ext_pose.h"

#define FORCE_ONELOOP
#ifdef FORCE_ONELOOP
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
float actuator_state_filt_vect[EFF_MAT_COLS_NB] = {0};
#else
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#ifndef ROTWING_EFF_SCHED_IXX_BODY
#error "NO ROTWING_EFF_SCHED_IXX_BODY defined"
#endif

#ifndef ROTWING_EFF_SCHED_IYY_BODY
#error "NO ROTWING_EFF_SCHED_IYY_BODY defined"
#endif

#ifndef ROTWING_EFF_SCHED_IZZ
#error "NO ROTWING_EFF_SCHED_IZZ defined"
#endif

#ifndef ROTWING_EFF_SCHED_IXX_WING
#error "NO ROTWING_EFF_SCHED_IXX_WING defined"
#endif

#ifndef ROTWING_EFF_SCHED_IYY_WING
#error "NO ROTWING_EFF_SCHED_IYY_WING defined"
#endif

#ifndef ROTWING_EFF_SCHED_M
#error "NO ROTWING_EFF_SCHED_M defined"
#endif

/* Effectiveness Matrix definition */
float G2_RW[EFF_MAT_COLS_NB]                       = {0};//ROTWING_EFF_SCHED_G2; //scaled by RW_G_SCALE
float G1_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB]      = {0};//{ROTWING_EFF_SCHED_G1_ZERO, ROTWING_EFF_SCHED_G1_ZERO, ROTWING_EFF_SCHED_G1_THRUST, ROTWING_EFF_SCHED_G1_ROLL, ROTWING_EFF_SCHED_G1_PITCH, ROTWING_EFF_SCHED_G1_YAW}; //scaled by RW_G_SCALE 
float EFF_MAT_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB] = {0};
static float flt_cut_ap = 2.0e-3;
static float flt_cut    = 1.0e-4;

struct FloatEulers eulers_zxy_RW_EFF;
static Butterworth2LowPass skew_filt; 
/* Temp variables*/
bool airspeed_fake_on = false;
float airspeed_fake = 0.0;
float ele_eff = 19.36; // (0.88*22.0);
float roll_eff = 5.5 ;//3.835;
float yaw_eff  = 0.390;
float ele_min = 0.0;
/* Define Forces and Moments tructs for each actuator*/
struct RW_Model RW;

inline void eff_scheduling_rotwing_update_wing_angle(void);
inline void eff_scheduling_rotwing_update_airspeed(void);
void  ele_pref_sched(void);
void  update_attitude(void);
void  sum_EFF_MAT_RW(void);
void  init_RW_Model(void);
void  calc_G1_G2_RW(void);  

/** ABI binding wing position data.
 */
#ifndef WING_ROTATION_CAN_ROTWING_ID
#define WING_ROTATION_CAN_ROTWING_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(WING_ROTATION_CAN_ROTWING_ID)
static abi_event wing_position_ev;

float skew_meas = 0.0;
static void wing_position_cb(uint8_t sender_id UNUSED, struct act_feedback_t *pos_msg, uint8_t num_act)
{
  for (int i=0; i<num_act; i++){
    if (pos_msg[i].set.position && (pos_msg[i].idx == SERVO_ROTATION_MECH_IDX))
    {
      skew_meas = 0.5 * M_PI - pos_msg[i].position;
      Bound(skew_meas, 0, 0.5 * M_PI);
    }
  }
}

#include "generated/modules.h"
PRINT_CONFIG_VAR(EFF_SCHEDULING_ROTWING_PERIODIC_FREQ)
void eff_scheduling_rotwing_init(void)
{
  init_RW_Model();
  update_attitude();
  AbiBindMsgACT_FEEDBACK(WING_ROTATION_CAN_ROTWING_ID, &wing_position_ev, wing_position_cb);
  float tau_skew = 1.0 / (2.0 * M_PI * 5.0);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&skew_filt, tau_skew, sample_time, 0.0);
}

void init_RW_Model(void)
{
  // Inertia and mass
  RW.I.b_xx = 0.12879; // [kgm²] (0.0478 + 0.08099)
  RW.I.b_yy = 0.94950; // [kgm²] (0.7546 + 0.1949)
  RW.I.w_xx = 0.0; // [kgm²]
  RW.I.w_yy = 0.0; // [kgm²]
  RW.I.xx   = RW.I.b_xx + RW.I.w_xx; // [kgm²]
  RW.I.yy   = RW.I.b_yy + RW.I.b_yy; // [kgm²]
  RW.I.zz   = 0.975; // [kgm²]
  RW.m      = 6.670; // [kg]
  // Motor Front
  RW.mF.dFdu     = 3.835 / RW_G_SCALE; // [N  / pprz] 
  RW.mF.dMdu     = yaw_eff / RW_G_SCALE; // [Nm / pprz]
  RW.mF.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mF.l        = 0.423             ; // [m]   435                
  // Motor Right
  RW.mR.dFdu     = roll_eff / RW_G_SCALE; // [N  / pprz]
  RW.mR.dMdu     = yaw_eff / RW_G_SCALE; // [Nm / pprz]
  RW.mR.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mR.l        = 0.408             ; // [m]   375     
  // Motor Back
  RW.mB.dFdu     = 3.835 / RW_G_SCALE; // [N  / pprz]
  RW.mB.dMdu     = yaw_eff / RW_G_SCALE; // [Nm / pprz]
  RW.mB.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mB.l        = 0.423             ; // [m]        
  // Motor Left
  RW.mL.dFdu     = roll_eff / RW_G_SCALE; // [N  / pprz]
  RW.mL.dMdu     = yaw_eff / RW_G_SCALE; // [Nm / pprz]
  RW.mL.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mL.l        = 0.408             ; // [m]        
  // Motor Pusher
  RW.mP.dFdu     = 3.468 / RW_G_SCALE; // [N  / pprz]
  RW.mP.dMdu     = 0.000 / RW_G_SCALE; // [Nm / pprz]
  RW.mP.dMdud    = 0.000 / RW_G_SCALE; // [Nm / pprz]
  RW.mP.l        = 0.000             ; // [m]        
  // Elevator
  RW.ele.dFdu    = ele_eff / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz]   old value: 24.81 29.70
  RW.ele.dMdu    = 0;                                 // [Nm / pprz]
  RW.ele.dMdud   = 0;                                 // [Nm / pprz]
  RW.ele.l       = 0.85;                              // [m]    
  RW.ele_pref    = 0;           
  // Rudder
  RW.rud.dFdu   = 1.207 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz] 
  RW.rud.dMdu   = 0;                                 // [Nm / pprz]
  RW.rud.dMdud  = 0;                                 // [Nm / pprz]
  RW.rud.l      = 0.88;                              // [m]        
  // Aileron
  RW.ail.dFdu   = 7.678 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz] (1.88 * 4.084)
  RW.ail.dMdu   = 0;                                 // [Nm / pprz]
  RW.ail.dMdud  = 0;                                 // [Nm / pprz]
  RW.ail.l      = 0.68;                              // [m]        
  // Flaperon
  RW.flp.dFdu   = 10.825 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz] (1.88 * 5.758)
  RW.flp.dMdu   = 0;                                 // [Nm / pprz]
  RW.flp.dMdud  = 0;                                 // [Nm / pprz]
  RW.flp.l      = 0.36;                              // [m]     
  // Lift: k0 the v²+ k1 the sin² v² + k2 sin² v²
  RW.wing.k0    = 0.336 + 0.0507 + 0.102;
  RW.wing.k1    = 0.616;
  RW.wing.k2    = 0.121;
  // Initialize attitude
  RW.att.phi    = 0.0;
  RW.att.theta  = 0.0; 
  RW.att.psi    = 0.0; 
  RW.att.sphi   = 0.0; 
  RW.att.cphi   = 0.0; 
  RW.att.stheta = 0.0; 
  RW.att.ctheta = 0.0; 
  RW.att.spsi   = 0.0; 
  RW.att.cpsi   = 0.0;
  // Initialize skew variables to quad values
  RW.skew.rad   = 0.0; // ABI input
  RW.skew.deg   = 0.0;
  RW.skew.cosr  = 1.0;
  RW.skew.sinr  = 0.0;
  RW.skew.cosr2 = 1.0;
  RW.skew.sinr2 = 0.0;
  RW.skew.sinr3 = 0.0;
  // Initialize airspeed
  RW.as  = 0.0;
  RW.as2 = 0.0;
}

/*Update the attitude*/
void  update_attitude(void)
{
  float_eulers_of_quat_zxy(&eulers_zxy_RW_EFF, stateGetNedToBodyQuat_f());
  RW.att.phi    = eulers_zxy_RW_EFF.phi;
  RW.att.theta  = eulers_zxy_RW_EFF.theta;
  RW.att.psi    = eulers_zxy_RW_EFF.psi;
  RW.att.sphi   = sinf(eulers_zxy_RW_EFF.phi);
  RW.att.cphi   = cosf(eulers_zxy_RW_EFF.phi);
  RW.att.stheta = sinf(eulers_zxy_RW_EFF.theta);
  RW.att.ctheta = cosf(eulers_zxy_RW_EFF.theta);
  RW.att.spsi   = sinf(eulers_zxy_RW_EFF.psi);
  RW.att.cpsi   = cosf(eulers_zxy_RW_EFF.psi);
}
/* Function to precalculate once some constant effectiveness values to improve efficiency*/
void calc_G1_G2_RW(void)
{
  // Inertia
  RW.I.xx = RW.I.b_xx + RW.skew.cosr2 * RW.I.w_xx + RW.skew.sinr2 * RW.I.w_yy;
  RW.I.yy = RW.I.b_yy + RW.skew.sinr2 * RW.I.w_xx + RW.skew.cosr2 * RW.I.w_yy;
  Bound(RW.I.xx, 0.01, 100.);
  Bound(RW.I.yy, 0.01, 100.);
  // Motor Front
  G1_RW[RW_aZ][COMMAND_MOTOR_FRONT]  = -RW.mF.dFdu / RW.m;
  G1_RW[RW_aq][COMMAND_MOTOR_FRONT]  =  (RW.mF.dFdu * RW.mF.l) / RW.I.yy;
  G1_RW[RW_ar][COMMAND_MOTOR_FRONT]  = -RW.mF.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_FRONT]      = -RW.mF.dMdud / RW.I.zz;
  // Motor Right
  G1_RW[RW_aZ][COMMAND_MOTOR_RIGHT]  = -RW.mR.dFdu / RW.m;
  G1_RW[RW_ap][COMMAND_MOTOR_RIGHT]  = -(RW.mR.dFdu * RW.mR.l * RW.skew.cosr) / RW.I.xx;
  G1_RW[RW_aq][COMMAND_MOTOR_RIGHT]  =  (RW.mR.dFdu * RW.mR.l * RW.skew.sinr) / RW.I.yy;
  G1_RW[RW_ar][COMMAND_MOTOR_RIGHT]  =  RW.mR.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_RIGHT]      =  RW.mR.dMdud / RW.I.zz;
  // Motor Back
  G1_RW[RW_aZ][COMMAND_MOTOR_BACK]   = -RW.mB.dFdu / RW.m;
  G1_RW[RW_aq][COMMAND_MOTOR_BACK]   = -(RW.mB.dFdu * RW.mB.l) / RW.I.yy;
  G1_RW[RW_ar][COMMAND_MOTOR_BACK]   = -RW.mB.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_BACK]       = -RW.mB.dMdud / RW.I.zz;
  // Motor Left
  G1_RW[RW_aZ][COMMAND_MOTOR_LEFT]   = -RW.mL.dFdu / RW.m;
  G1_RW[RW_ap][COMMAND_MOTOR_LEFT]   =  (RW.mL.dFdu * RW.mL.l * RW.skew.cosr) / RW.I.xx;
  G1_RW[RW_aq][COMMAND_MOTOR_LEFT]   = -(RW.mL.dFdu * RW.mL.l * RW.skew.sinr) / RW.I.yy;
  G1_RW[RW_ar][COMMAND_MOTOR_LEFT]   =  RW.mL.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_LEFT]       =  RW.mL.dMdud / RW.I.zz;
  // Motor Pusher
  G1_RW[RW_aX][COMMAND_MOTOR_PUSHER] =  RW.mP.dFdu / RW.m;
  // Elevator
  RW.ele.dFdu                     = ele_eff / (RW_G_SCALE * RW_G_SCALE);
  G1_RW[RW_aq][COMMAND_ELEVATOR]     =  (RW.ele.dFdu * RW.as2 * RW.ele.l) / RW.I.yy;
  // Rudder
  G1_RW[RW_ar][COMMAND_RUDDER]       =  (RW.rud.dFdu * RW.as2 * RW.rud.l) / RW.I.zz ;
  // Aileron
  G1_RW[RW_ap][COMMAND_AILERONS]     =  (RW.ail.dFdu * RW.as2 * RW.ail.l * RW.skew.sinr3) / RW.I.xx;
  G1_RW[RW_aq][COMMAND_AILERONS]     =  (RW.ail.dFdu * RW.as2 * RW.ail.l * (RW.skew.cosr-RW.skew.cosr3)) / RW.I.yy;
  
  // Flaperon
  G1_RW[RW_ap][COMMAND_FLAPS]        =  (RW.flp.dFdu * RW.as2 * RW.flp.l * RW.skew.sinr3) / RW.I.xx;
  G1_RW[RW_aq][COMMAND_FLAPS]        =  (RW.flp.dFdu * RW.as2 * RW.flp.l * (RW.skew.cosr-RW.skew.cosr3)) / RW.I.yy;
  // Lift and thrust
  RW.wing.dLdtheta                =  (RW.wing.k0 + RW.wing.k1 * RW.skew.sinr2) * RW.as2;
  Bound(RW.wing.dLdtheta, 0.0, 1300.0);
  RW.wing.L                       =  RW.wing.k0 * RW.att.theta * RW.as2 + RW.wing.k1 * RW.att.theta * RW.skew.sinr2 * RW.as2 + RW.wing.k2 * RW.skew.sinr2 * RW.as2;
  Bound(RW.wing.L, 0.0, 350.0);
  RW.T = actuator_state_1l[COMMAND_MOTOR_FRONT] * RW.mF.dFdu + actuator_state_1l[COMMAND_MOTOR_RIGHT] * RW.mR.dFdu + actuator_state_1l[COMMAND_MOTOR_BACK] * RW.mB.dFdu + actuator_state_1l[COMMAND_MOTOR_LEFT] * RW.mL.dFdu;
  Bound(RW.T, 0.0, 180.0);
  RW.P                            = actuator_state_1l[COMMAND_MOTOR_PUSHER] * RW.mP.dFdu;
}

void eff_scheduling_rotwing_periodic(void)
{
  update_attitude();
  eff_scheduling_rotwing_update_wing_angle();
  eff_scheduling_rotwing_update_airspeed();
  ele_pref_sched();
  calc_G1_G2_RW();
  sum_EFF_MAT_RW();
}


/**
 * @brief Function that sums g1 and g2 to obtain the g1_g2 matrix. It also undoes the scaling that was done to make the values readable
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */


void sum_EFF_MAT_RW(void) {
  
  // Thrust and Pusher force estimation
  float L      = RW.wing.L / RW.m;          // Lift specific force
  float dLdtheta = RW.wing.dLdtheta / RW.m; // Lift specific force derivative with pitch
  float T      = RW.T / RW.m;             //  Thrust specific force. Minus gravity is a guesstimate.
  float P      = RW.P / RW.m;               // Pusher specific force
  float T_L    = T + L * RW.att.ctheta;     // Thrust and Lift term
  float P_L    = P + L * RW.att.stheta;     // Pusher and Lift term
  int i = 0;
  int j = 0;

  for (i = 0; i < EFF_MAT_COLS_NB; i++) {
    switch (i) {
    case (COMMAND_MOTOR_FRONT):
    case (COMMAND_MOTOR_BACK):
    case (COMMAND_MOTOR_RIGHT):     
    case (COMMAND_MOTOR_LEFT):
      EFF_MAT_RW[RW_aN][i] = (RW.att.cpsi * RW.att.stheta + RW.att.ctheta * RW.att.sphi   * RW.att.spsi) * G1_RW[RW_aZ][i];
      EFF_MAT_RW[RW_aE][i] = (RW.att.spsi * RW.att.stheta - RW.att.cpsi   * RW.att.ctheta * RW.att.sphi) * G1_RW[RW_aZ][i];
      EFF_MAT_RW[RW_aD][i] = (RW.att.cphi * RW.att.ctheta                                              ) * G1_RW[RW_aZ][i];
      EFF_MAT_RW[RW_ap][i] = (G1_RW[RW_ap][i])                                       ;
      EFF_MAT_RW[RW_aq][i] = (G1_RW[RW_aq][i])                                       ;
      EFF_MAT_RW[RW_ar][i] = (G1_RW[RW_ar][i] + G2_RW[i])                            ;
      break;
    case (COMMAND_MOTOR_PUSHER): 
      EFF_MAT_RW[RW_aN][i] = (RW.att.cpsi   * RW.att.ctheta - RW.att.sphi * RW.att.spsi * RW.att.stheta) * G1_RW[RW_aX][i];
      EFF_MAT_RW[RW_aE][i] = (RW.att.ctheta * RW.att.spsi   + RW.att.cpsi * RW.att.sphi * RW.att.stheta) * G1_RW[RW_aX][i];
      EFF_MAT_RW[RW_aD][i] = (- RW.att.cphi * RW.att.stheta                                            ) * G1_RW[RW_aX][i];
      EFF_MAT_RW[RW_ap][i] = 0.0;
      EFF_MAT_RW[RW_aq][i] = 0.0;
      EFF_MAT_RW[RW_ar][i] = 0.0;
      break;
    case (COMMAND_ELEVATOR):
    case (COMMAND_RUDDER): 
    case (COMMAND_AILERONS):
    case (COMMAND_FLAPS):  
      EFF_MAT_RW[RW_aN][i] = 0.0;
      EFF_MAT_RW[RW_aE][i] = 0.0;
      EFF_MAT_RW[RW_aD][i] = 0.0;
      EFF_MAT_RW[RW_ap][i] = G1_RW[RW_ap][i];
      EFF_MAT_RW[RW_aq][i] = G1_RW[RW_aq][i];
      EFF_MAT_RW[RW_ar][i] = G1_RW[RW_ar][i];
      break;     
    case (COMMAND_ROLL):
      EFF_MAT_RW[RW_aN][i] = (-RW.att.cphi * RW.att.ctheta * RW.att.spsi * T_L - RW.att.cphi * RW.att.spsi * RW.att.stheta * P_L);
      EFF_MAT_RW[RW_aE][i] = ( RW.att.cphi * RW.att.ctheta * RW.att.cpsi * T_L + RW.att.cphi * RW.att.cpsi * RW.att.stheta * P_L);
      EFF_MAT_RW[RW_aD][i] = ( RW.att.sphi * RW.att.ctheta * T_L + RW.att.sphi * RW.att.stheta * P_L);
      EFF_MAT_RW[RW_ap][i] = 0.0;
      EFF_MAT_RW[RW_aq][i] = 0.0;
      EFF_MAT_RW[RW_ar][i] = 0.0;  
      break;
    case (COMMAND_PITCH):
      EFF_MAT_RW[RW_aN][i] = (-(RW.att.ctheta * RW.att.cpsi - RW.att.sphi * RW.att.stheta * RW.att.spsi) * T - (RW.att.cpsi * RW.att.stheta + RW.att.ctheta * RW.att.sphi   * RW.att.spsi) * P - RW.att.sphi * RW.att.spsi * dLdtheta);
      EFF_MAT_RW[RW_aE][i] = (-(RW.att.ctheta * RW.att.spsi + RW.att.sphi * RW.att.stheta * RW.att.cpsi) * T - (RW.att.spsi * RW.att.stheta - RW.att.cpsi   * RW.att.ctheta * RW.att.sphi) * P + RW.att.sphi * RW.att.cpsi * dLdtheta);
      EFF_MAT_RW[RW_aD][i] = ( RW.att.stheta * RW.att.cphi * T - RW.att.cphi * RW.att.ctheta * P - RW.att.cphi * dLdtheta)                                           ;
      EFF_MAT_RW[RW_ap][i] = 0.0;
      EFF_MAT_RW[RW_aq][i] = 0.0;
      EFF_MAT_RW[RW_ar][i] = 0.0;
      break;
    default:
      break;
    }
  }
  for (i = 0; i < EFF_MAT_ROWS_NB; i++) {
    for(j = 0; j < EFF_MAT_COLS_NB; j++) {
      float abs = fabs(EFF_MAT_RW[i][j]);
      switch (i) {
        case (RW_aN):
        case (RW_aE):
        case (RW_aD):
        case (RW_aq):
        case (RW_ar):
          if (abs < flt_cut) {
            EFF_MAT_RW[i][j] = 0.0;
          }
          break;
        case (RW_ap):
          if (abs < flt_cut_ap) {
            EFF_MAT_RW[i][j] = 0.0;
          }
          break;
      }
    }
  }
}

void eff_scheduling_rotwing_update_wing_angle(void)
{
  // Calculate sin and cosines of rotation
  update_butterworth_2_low_pass(&skew_filt, skew_meas);
  RW.skew.rad   = skew_filt.o[0];
  RW.skew.deg   = RW.skew.rad / M_PI * 180.;
  RW.skew.cosr  = cosf(RW.skew.rad);
  RW.skew.sinr  = sinf(RW.skew.rad);
  RW.skew.cosr2 = RW.skew.cosr * RW.skew.cosr;
  RW.skew.sinr2 = RW.skew.sinr * RW.skew.sinr;
  RW.skew.sinr3 = RW.skew.sinr2 * RW.skew.sinr;
  RW.skew.cosr3 = RW.skew.cosr2 * RW.skew.cosr;
#ifdef INS_EXT_VISION_ROTATION
  // Define an INS external pose quaternion rotation from the wing rotation angle
  struct FloatEulers rot_e = {0, 0, RW.skew.rad};
  float_quat_of_eulers(&ins_ext_vision_rot, &rot_e);
#endif

}
float time = 0.0;
void eff_scheduling_rotwing_update_airspeed(void)
{
  RW.as = stateGetAirspeed_f();
  Bound(RW.as, 0. , 30.);
  RW.as2 = RW.as * RW.as;
  Bound(RW.as2, 0. , 900.);
  if(airspeed_fake_on) {
    //float freq = 0.5;
    //float amp = 3.0;
    time = time + 0.002;
    RW.as = airspeed_fake ;//+ amp * sinf(2.0 * M_PI * freq * time);
    RW.as2 = RW.as * RW.as;
  } else {
    time = 0.0;
  }
}

void ele_pref_sched(void)
{
  if (RW.as > ELE_MIN_AS){
    RW.ele_pref = (ZERO_ELE_PPRZ - ele_min) / (ELE_MAX_AS - ELE_MIN_AS) * (RW.as - ELE_MIN_AS) + ele_min;
    Bound(RW.ele_pref,ele_min,ZERO_ELE_PPRZ);
  } else {
    RW.ele_pref = ele_min;
  }
}

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

/** @file "modules/ctrl/eff_scheduling_plusquad.c"
 * @author Tomaso De Ponti <T.M.L.DePonti@tudelft.nl>
 * The control effectiveness scheduler for the plus quad drone type
 */

#include "modules/ctrl/eff_scheduling_plusquad.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "modules/ins/ins_ext_pose.h"

#define FORCE_ONELOOP
#ifdef FORCE_ONELOOP
#include "firmwares/rotorcraft/oneloop/oneloop_nB.h"
float actuator_state_filt_vect[EFF_MAT_COLS_NB] = {0};
#else
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#ifndef PLUSQUAD_EFF_SCHED_IXX
#error "NO PLUSQUAD_EFF_SCHED_IXX defined"
#endif

#ifndef PLUSQUAD_EFF_SCHED_IYY
#error "NO PLUSQUAD_EFF_SCHED_IYY defined"
#endif

#ifndef PLUSQUAD_EFF_SCHED_IZZ
#error "NO PLUSQUAD_EFF_SCHED_IZZ defined"
#endif

#ifndef PLUSQUAD_EFF_SCHED_M
#error "NO PLUSQUAD_EFF_SCHED_M defined"
#endif

bool manual_roll  = false;
bool manual_pitch = false;
bool manual_yaw   = false;

float roll_mult   = 1.0;
float pitch_mult  = 1.0;
float yaw_mult    = 1.0;
float thrust_mult = 1.0;
float cutoff_prev = 1.0;
/* Effectiveness Matrix definition */
float G2_RW[EFF_MAT_COLS_NB]                       = {0};//PLUSQUAD_EFF_SCHED_G2; //scaled by RW_G_SCALE
float G1_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB]      = {0};//{PLUSQUAD_EFF_SCHED_G1_ZERO, PLUSQUAD_EFF_SCHED_G1_ZERO, PLUSQUAD_EFF_SCHED_G1_THRUST, PLUSQUAD_EFF_SCHED_G1_ROLL, PLUSQUAD_EFF_SCHED_G1_PITCH, PLUSQUAD_EFF_SCHED_G1_YAW}; //scaled by RW_G_SCALE 
float EFF_MAT_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB] = {0};
float I_inv[3][3]                                  = {0};
static float flt_cut_a  = 1.0e-6;
static float flt_cut_ap = 2.0e-3;
static float flt_cut    = 1.0e-4;

struct FloatEulers eulers_zxy_RW_EFF;
static Butterworth2LowPass phi_filt;
static Butterworth2LowPass theta_filt;
static Butterworth2LowPass psi_filt;
/* Temp variables*/
int G2_on = 1;
float roll_eff = 8.9;//15.402;//3.835;//3.835;5.5
float yaw_eff  = 0.237; // 1.3171*0.390=0.514 or 0.659 and 0.812 (pitch - roll)
float ele_min = 0.0;
/* Define Forces and Moments tructs for each actuator*/
struct RW_Model RW;

int thrust_curve = 2;
float temp_mQ_k = 1.37; //Manual Value For Tuning2
void  update_attitude(void);
void  sum_EFF_MAT_RW(void);
void  init_RW_Model(void);
void  calc_G1_G2_RW(void);  
float calc_thrust_curve(float k1, float k2, float k3, float u);
float calc_thrust_curve_d(float k1, float k2, float u);
void calc_all_thrust_curve(void);
void init_all_thrust_curve(void);


#include "generated/modules.h"
PRINT_CONFIG_VAR(EFF_SCHEDULING_ROTWING_PERIODIC_FREQ)
void eff_scheduling_rotwing_init(void)
{
  init_RW_Model();
  update_attitude();
  float tau_att = 1.0 / (2.0 * M_PI * oneloop_nB_filt_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&phi_filt, tau_att, sample_time, 0.0);
  init_butterworth_2_low_pass(&theta_filt, tau_att, sample_time, 0.0);
  init_butterworth_2_low_pass(&psi_filt, tau_att, sample_time, 0.0);
}

void init_RW_Model(void)
{
  // Inertia and mass
  RW.I.xx   = PLUSQUAD_EFF_SCHED_IXX;    // [kgm²]
  RW.I.yy   = PLUSQUAD_EFF_SCHED_IYY;    // [kgm²]
  RW.I.zz   = PLUSQUAD_EFF_SCHED_IZZ;    // [kgm²]
  RW.m      = PLUSQUAD_EFF_SCHED_M;      // [kg]
  
  // Init the thrust curves
  init_all_thrust_curve();
  // Motor Front
  RW.mF.dFdu  = PLUSQUAD_EFF_SCHED_MF_dFdu / RW_G_SCALE;
  RW.mF.dMdu  = PLUSQUAD_EFF_SCHED_MF_dMdu / RW_G_SCALE;
  RW.mF.dMdud = PLUSQUAD_EFF_SCHED_MF_dMdud / (RW_G_SCALE * RW_G_SCALE);
  RW.mF.l     = PLUSQUAD_EFF_SCHED_MF_l;

  // Motor Right
  RW.mR.dFdu  = PLUSQUAD_EFF_SCHED_MR_dFdu / RW_G_SCALE;
  RW.mR.dMdu  = PLUSQUAD_EFF_SCHED_MR_dMdu / RW_G_SCALE;
  RW.mR.dMdud = PLUSQUAD_EFF_SCHED_MR_dMdud / (RW_G_SCALE * RW_G_SCALE);
  RW.mR.l     = PLUSQUAD_EFF_SCHED_MR_l;

  // Motor Back
  RW.mB.dFdu  = PLUSQUAD_EFF_SCHED_MB_dFdu / RW_G_SCALE;
  RW.mB.dMdu  = PLUSQUAD_EFF_SCHED_MB_dMdu / RW_G_SCALE;
  RW.mB.dMdud = PLUSQUAD_EFF_SCHED_MB_dMdud / (RW_G_SCALE * RW_G_SCALE);
  RW.mB.l     = PLUSQUAD_EFF_SCHED_MB_l;

  // Motor Left
  RW.mL.dFdu  = PLUSQUAD_EFF_SCHED_ML_dFdu / RW_G_SCALE;
  RW.mL.dMdu  = PLUSQUAD_EFF_SCHED_ML_dMdu / RW_G_SCALE;
  RW.mL.dMdud = PLUSQUAD_EFF_SCHED_ML_dMdud / (RW_G_SCALE * RW_G_SCALE);
  RW.mL.l     = PLUSQUAD_EFF_SCHED_ML_l;

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

  cutoff_prev = oneloop_nB_filt_cutoff;
}

/*Update the attitude*/
void  update_attitude(void)
{
  if(cutoff_prev != oneloop_nB_filt_cutoff){
    float tau_att = 1.0 / (2.0 * M_PI * oneloop_nB_filt_cutoff);
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    init_butterworth_2_low_pass(&phi_filt, tau_att, sample_time, RW.att.phi);
    init_butterworth_2_low_pass(&theta_filt, tau_att, sample_time, RW.att.theta);
    init_butterworth_2_low_pass(&psi_filt, tau_att, sample_time, RW.att.psi);
    cutoff_prev = oneloop_nB_filt_cutoff;
  }
  float_eulers_of_quat_zxy(&eulers_zxy_RW_EFF, stateGetNedToBodyQuat_f());
  update_butterworth_2_low_pass(&phi_filt, eulers_zxy_RW_EFF.phi);
  update_butterworth_2_low_pass(&theta_filt, eulers_zxy_RW_EFF.theta);
  update_butterworth_2_low_pass(&psi_filt, eulers_zxy_RW_EFF.psi);
  RW.att.phi    = phi_filt.o[0];//eulers_zxy_RW_EFF.phi;
  RW.att.theta  = theta_filt.o[0];//eulers_zxy_RW_EFF.theta;
  RW.att.psi    = psi_filt.o[0];//eulers_zxy_RW_EFF.psi;
  //printf("Attitude filt: %f %f %f\n", RW.att.phi, RW.att.theta, RW.att.psi);
  RW.att.sphi   = sinf(RW.att.phi);
  RW.att.cphi   = cosf(RW.att.phi);
  RW.att.stheta = sinf(RW.att.theta);
  RW.att.ctheta = cosf(RW.att.theta);
  RW.att.spsi   = sinf(RW.att.psi);
  RW.att.cpsi   = cosf(RW.att.psi);
}
/* Function to precalculate once some constant effectiveness values to improve efficiency*/
void calc_G1_G2_RW(void)
{
  // Inertia
  int x = 0;
  int y = 1;
  int z = 2;
  I_inv[x][x] = 1/RW.I.xx;
  I_inv[x][y] = 0.0;
  I_inv[x][z] = 0.0;
  I_inv[y][x] = 0.0;
  I_inv[y][y] = 1/RW.I.yy;
  I_inv[y][z] = 0.0;
  I_inv[z][x] = 0.0;
  I_inv[z][y] = 0.0;
  I_inv[z][z] = 1/RW.I.zz;
  //printf("I_inv_x: %f %f %f\n", I_inv[x][x], I_inv[x][y], I_inv[x][z]);
  //printf("I_inv_y: %f %f %f\n", I_inv[y][x], I_inv[y][y], I_inv[y][z]);
  //printf("I_inv_z: %f %f %f\n", I_inv[z][x], I_inv[z][y], I_inv[z][z]);
  //printf("Control - I: %f %f %f\n", 1.0/I_inv[x][x], 1.0/I_inv[y][y], 1.0/I_inv[z][z]);
  // Calc motor and control effectiveness
  calc_all_thrust_curve();

  // Motor Front
  G1_RW[RW_aZ][COMMAND_MOTOR_FRONT]  = -RW.mF.dFdu / RW.m;
  G1_RW[RW_ap][COMMAND_MOTOR_FRONT]  =  0.0;
  G1_RW[RW_aq][COMMAND_MOTOR_FRONT]  =  (RW.mF.dFdu * RW.mF.l) * I_inv[y][y];
  G1_RW[RW_ar][COMMAND_MOTOR_FRONT]  = -RW.mF.dMdu  * I_inv[z][z];
  G2_RW[COMMAND_MOTOR_FRONT]         = -G2_on*RW.mF.dMdud * I_inv[z][z] * PERIODIC_FREQUENCY;
  // Motor Right
  G1_RW[RW_aZ][COMMAND_MOTOR_RIGHT]  = -RW.mR.dFdu / RW.m;
  G1_RW[RW_ap][COMMAND_MOTOR_RIGHT]  = -RW.mR.dFdu * RW.mR.l * I_inv[x][x];
  G1_RW[RW_aq][COMMAND_MOTOR_RIGHT]  =  0.0;
  G1_RW[RW_ar][COMMAND_MOTOR_RIGHT]  =  RW.mR.dMdu  * I_inv[z][z];
  G2_RW[COMMAND_MOTOR_RIGHT]         =  G2_on*RW.mR.dMdud * I_inv[z][z] * PERIODIC_FREQUENCY;
  // Motor Back
  G1_RW[RW_aZ][COMMAND_MOTOR_BACK]   = -RW.mB.dFdu / RW.m;
  G1_RW[RW_ap][COMMAND_MOTOR_BACK]   = 0.0;
  G1_RW[RW_aq][COMMAND_MOTOR_BACK]   = -(RW.mB.dFdu * RW.mB.l) * I_inv[y][y];
  G1_RW[RW_ar][COMMAND_MOTOR_BACK]   = -RW.mB.dMdu  * I_inv[z][z];
  G2_RW[COMMAND_MOTOR_BACK]          = -G2_on*RW.mB.dMdud * I_inv[z][z] * PERIODIC_FREQUENCY;
  // Motor Left
  G1_RW[RW_aZ][COMMAND_MOTOR_LEFT]   = -RW.mL.dFdu / RW.m;
  G1_RW[RW_ap][COMMAND_MOTOR_LEFT]   =  RW.mL.dFdu * RW.mL.l * I_inv[x][x];
  G1_RW[RW_aq][COMMAND_MOTOR_LEFT]   =  0.0;
  G1_RW[RW_ar][COMMAND_MOTOR_LEFT]   =  RW.mL.dMdu  * I_inv[z][z];
  G2_RW[COMMAND_MOTOR_LEFT]          =  G2_on*RW.mL.dMdud * I_inv[z][z] * PERIODIC_FREQUENCY;
}

void eff_scheduling_rotwing_periodic(void)
{
  update_attitude();
  calc_G1_G2_RW();
  sum_EFF_MAT_RW();
}


/**
 * @brief Function that sums g1 and g2 to obtain the g1_g2 matrix. It also undoes the scaling that was done to make the values readable
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */


void sum_EFF_MAT_RW(void) {
  
  // Thrust force estimation
  float T      = RW.T / RW.m;             //  Thrust specific force. Minus gravity is a guesstimate.
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
    case (COMMAND_ROLL):
      EFF_MAT_RW[RW_aN][i] = (-RW.att.cphi * RW.att.ctheta * RW.att.spsi * T);
      EFF_MAT_RW[RW_aE][i] = ( RW.att.cphi * RW.att.ctheta * RW.att.cpsi * T);
      EFF_MAT_RW[RW_aD][i] = ( RW.att.sphi * RW.att.ctheta * T);
      EFF_MAT_RW[RW_ap][i] = 0.0;
      EFF_MAT_RW[RW_aq][i] = 0.0;
      EFF_MAT_RW[RW_ar][i] = 0.0;  
      break;
    case (COMMAND_PITCH):
      EFF_MAT_RW[RW_aN][i] = (-(RW.att.ctheta * RW.att.cpsi - RW.att.sphi * RW.att.stheta * RW.att.spsi) * T);
      EFF_MAT_RW[RW_aE][i] = (-(RW.att.ctheta * RW.att.spsi + RW.att.sphi * RW.att.stheta * RW.att.cpsi) * T);
      EFF_MAT_RW[RW_aD][i] = ( RW.att.stheta * RW.att.cphi * T)                                           ;
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
          if (abs < flt_cut_a) {
            EFF_MAT_RW[i][j] = 0.0;
          }
          break;
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
  if(manual_roll){
    EFF_MAT_RW[RW_ap][1] = -0.0370*roll_mult;//-0.0317895*roll_mult;
    EFF_MAT_RW[RW_ap][3] =  0.0370*roll_mult;// 0.0317895*roll_mult;
    
  }
  if(manual_pitch){
    EFF_MAT_RW[RW_aq][0] =  0.0314*pitch_mult;// 0.0274249*pitch_mult; 
    EFF_MAT_RW[RW_aq][2] = -0.0314*pitch_mult;//-0.0274249*pitch_mult;
  }
  if(manual_yaw){
    EFF_MAT_RW[RW_ar][0] = -0.0022*yaw_mult;//-0.0025*yaw_mult; 
    EFF_MAT_RW[RW_ar][1] =  0.0022*yaw_mult;// 0.0025*yaw_mult;
    EFF_MAT_RW[RW_ar][2] = -0.0022*yaw_mult;//-0.0025*yaw_mult;
    EFF_MAT_RW[RW_ar][3] =  0.0022*yaw_mult;// 0.0025*yaw_mult;
    //G2_RW[0] = -2.1e-5;
    //G2_RW[1] = 2.1e-5;
    //G2_RW[2] = -2.1e-5;
    //G2_RW[3] = 2.1e-5;
  }
}

float calc_thrust_curve(float k1, float k2, float k3, float u){
  return k1*u*u + k2*u + k3;
}

float calc_thrust_curve_d(float k1, float k2, float u){
  float out= 2.0*k1*u + k2;
  Bound(out, 1.0e-3, 1.0e-2);
  return out;
}

void calc_all_thrust_curve(void){
  switch(thrust_curve){
    case(0):
      // Curve calculated at the Current Motor State
      RW.mF.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MF_k1, PLUSQUAD_EFF_SCHED_MF_k2, actuator_state_1l[COMMAND_MOTOR_FRONT]);
      RW.mR.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MR_k1, PLUSQUAD_EFF_SCHED_MR_k2, actuator_state_1l[COMMAND_MOTOR_RIGHT]);
      RW.mB.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MB_k1, PLUSQUAD_EFF_SCHED_MB_k2, actuator_state_1l[COMMAND_MOTOR_BACK]) ;
      RW.mL.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_ML_k1, PLUSQUAD_EFF_SCHED_ML_k2, actuator_state_1l[COMMAND_MOTOR_LEFT]) ;
      break;
    case(1):
      // Curve statically calculated at 50% throttle
      RW.mF.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MF_k1, PLUSQUAD_EFF_SCHED_MF_k2, 4800.0);
      RW.mR.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MR_k1, PLUSQUAD_EFF_SCHED_MR_k2, 4800.0);
      RW.mB.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MB_k1, PLUSQUAD_EFF_SCHED_MB_k2, 4800.0);
      RW.mL.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_ML_k1, PLUSQUAD_EFF_SCHED_ML_k2, 4800.0);
      break;
    case(2):
      // Manual thrust for tuning
      RW.mF.dFdu = PLUSQUAD_EFF_SCHED_MF_k2*thrust_mult;//0.0009645*thrust_mult;//calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MF_k1, PLUSQUAD_EFF_SCHED_MF_k2, 4800.0);
      RW.mR.dFdu = PLUSQUAD_EFF_SCHED_MR_k2*thrust_mult;//0.0009645*thrust_mult;//temp_mQ_k/RW_G_SCALE;
      RW.mB.dFdu = PLUSQUAD_EFF_SCHED_MB_k2*thrust_mult;//0.0009645*thrust_mult;//calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MB_k1, PLUSQUAD_EFF_SCHED_MB_k2, 4800.0);;
      RW.mL.dFdu = PLUSQUAD_EFF_SCHED_ML_k2*thrust_mult;//0.0009645*thrust_mult;//temp_mQ_k/RW_G_SCALE;
      break;
  }
  // T = k1*u^2 + k2*u + k3-----|k1                      | k2                     | k3                     | u
  // float T_mF = calc_thrust_curve(PLUSQUAD_EFF_SCHED_MF_k1, PLUSQUAD_EFF_SCHED_MF_k2, PLUSQUAD_EFF_SCHED_MF_k3, actuator_state_1l[COMMAND_MOTOR_FRONT]);
  // float T_mR = calc_thrust_curve(PLUSQUAD_EFF_SCHED_MR_k1, PLUSQUAD_EFF_SCHED_MR_k2, PLUSQUAD_EFF_SCHED_MR_k3, actuator_state_1l[COMMAND_MOTOR_RIGHT]);
  // float T_mB = calc_thrust_curve(PLUSQUAD_EFF_SCHED_MB_k1, PLUSQUAD_EFF_SCHED_MB_k2, PLUSQUAD_EFF_SCHED_MB_k3, actuator_state_1l[COMMAND_MOTOR_BACK]);
  // float T_mL = calc_thrust_curve(PLUSQUAD_EFF_SCHED_ML_k1, PLUSQUAD_EFF_SCHED_ML_k2, PLUSQUAD_EFF_SCHED_ML_k3, actuator_state_1l[COMMAND_MOTOR_LEFT]);
  RW.T = RW.m*9.81/(RW.att.cphi * RW.att.ctheta);//T_mF + T_mR + T_mB + T_mL;
  Bound(RW.T, 30.0, 180.0);
}

void init_all_thrust_curve(void){
  // dTdu    = 2*k1*u + k2--------|k1                      | k2                      | u
  RW.mF.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MF_k1, PLUSQUAD_EFF_SCHED_MF_k2, 4800.0);
  RW.mR.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MR_k1, PLUSQUAD_EFF_SCHED_MR_k2, 4800.0);
  RW.mB.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_MB_k1, PLUSQUAD_EFF_SCHED_MB_k2, 4800.0);
  RW.mL.dFdu = calc_thrust_curve_d(PLUSQUAD_EFF_SCHED_ML_k1, PLUSQUAD_EFF_SCHED_ML_k2, 4800.0);
  // T = k1*u^2 + k2*u + k3-------|k1                     | k2                    | k3                     | u
  float T_mF = calc_thrust_curve(PLUSQUAD_EFF_SCHED_MF_k1, PLUSQUAD_EFF_SCHED_MF_k2, PLUSQUAD_EFF_SCHED_MF_k3, 4800.0);
  float T_mR = calc_thrust_curve(PLUSQUAD_EFF_SCHED_MR_k1, PLUSQUAD_EFF_SCHED_MR_k2, PLUSQUAD_EFF_SCHED_MR_k3, 4800.0);
  float T_mB = calc_thrust_curve(PLUSQUAD_EFF_SCHED_MB_k1, PLUSQUAD_EFF_SCHED_MB_k2, PLUSQUAD_EFF_SCHED_MB_k3, 4800.0);
  float T_mL = calc_thrust_curve(PLUSQUAD_EFF_SCHED_ML_k1, PLUSQUAD_EFF_SCHED_ML_k2, PLUSQUAD_EFF_SCHED_ML_k3, 4800.0);
  RW.T = T_mF + T_mR + T_mB + T_mL;
  Bound(RW.T, 0.0, 180.0);
}

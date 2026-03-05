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

/** @file "modules/ctrl/eff_scheduling_rotwing_V2.h"
 * @author Tomaso De Ponti <T.M.L.DePonti@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#ifndef CTRL_EFF_SCHED_ROTWING_H
#define CTRL_EFF_SCHED_ROTWING_H

#include "std.h"
#include "generated/airframe.h"

#ifndef EFF_MAT_ROWS_NB
#define EFF_MAT_ROWS_NB 6
#endif

#define RW_aX 0 // X body axis (linear acceleration) 
#define RW_aY 1 // Y body axis (linear acceleration)
#define RW_aZ 2 // Z body axis (linear acceleration)
#define RW_aN 0 // North axis (linear acceleration)
#define RW_aE 1 // East axis (linear acceleration)
#define RW_aD 2 // Down axis (linear acceleration)
#define RW_ap 3 // X body axis (angular acceleration)
#define RW_aq 4 // Y body axis (angular acceleration)
#define RW_ar 5 // Z body axis (angular acceleration)

#ifndef EFF_MAT_COLS_NB
#define EFF_MAT_COLS_NB (COMMANDS_NB_REAL + COMMANDS_NB_VIRTUAL)
#endif

#define RW_G_SCALE 1000.0f

// Elevator ctrl definitions
#ifndef ZERO_ELE_PPRZ
#define ZERO_ELE_PPRZ 8000
#endif

#ifndef ELE_MIN_AS
#define ELE_MIN_AS 8.8
#endif

#ifndef ELE_MAX_AS // should match the max airspeed
#define ELE_MAX_AS 16.0
#endif

extern float EFF_MAT_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB];
extern float G2_RW[EFF_MAT_COLS_NB]                      ; 
extern float G1_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB]     ; 
struct rotwing_eff_sched_param_t {
  float Ixx_body;                 // body MMOI around roll axis [kgm²]
  float Iyy_body;                 // body MMOI around pitch axis [kgm²]
  float Izz;                      // total MMOI around yaw axis [kgm²]
  float Ixx_wing;                 // wing MMOI around the chordwise direction of the wing [kgm²]
  float Iyy_wing;                 // wing MMOI around the spanwise direction of the wing [kgm²]
  float m;                        // mass [kg]
  float DMdpprz_hover_roll[2];    // Moment coeficients for roll motors (Scaled by 10000)
  float hover_roll_pitch_coef[2]; // Model coefficients to correct pitch effective for roll motors
  float hover_roll_roll_coef[2];  // Model coefficients to correct roll effectiveness for roll motors
  float k_elevator[3];
  float k_rudder[3];
  float k_aileron;
  float k_flaperon;
  float k_pusher[2];
  float k_elevator_deflection[2];
  float d_rudder_d_pprz;
  float k_rpm_pprz_pusher[3];
  float k_lift_wing[2];
  float k_lift_fuselage;
  float k_lift_tail;
};

struct rotwing_eff_sched_var_t {
  float Ixx;                  // Total MMOI around roll axis [kgm²]
  float Iyy;                  // Total MMOI around pitch axis [kgm²]
  float wing_rotation_rad;    // Wing rotation angle in radians: from ABI message
  float wing_rotation_deg;    // Wing rotation angle in degrees: (clone in degrees)
  float cosr;                 // cosine of wing rotation angle
  float sinr;                 // sine of wing rotation angle
  float cosr2;                // cosine² of wing rotation angle
  float sinr2;                // sine² of wing rotation angle
  float cosr3;                // cosine³ of wing rotation angle
  float sinr3;                // sine³ of wing rotation angle

  // Set during initialization
  float pitch_motor_dMdpprz;  // derivative of delta moment with respect to a delta paparazzi command for the pitch motors [Nm/pprz]
  float roll_motor_dMdpprz;   // derivative of delta moment with respect to a delta paparazzi command for the roll motors [Nm/pprz]

  // commands
  float cmd_elevator;
  float cmd_pusher;
  float cmd_pusher_scaled;
  float cmd_T_mean_scaled;

  // airspeed
  float airspeed;
  float airspeed2;
};

struct I{
  float xx;
  float yy;
  float zz;
  float w_xx;
  float w_yy;
  float b_xx;
  float b_yy;
};
struct F_M_Body{
  float dFdu;     // derivative of the force with respect to the control input (e.g. linear coefficient)
  float dMdu;     // derivative of the reaction trque with respect to the control input (e.g. linear coefficient)
  float dMdud;    // derivative of the reaction torque with respect to the control input time derivative 
  float l;        // arm length
};

struct wing_model{
  float k0; // Linear coefficient (theta u²)
  float k1; // Linear coeeficient (theta u² s(Lambda)²)
  float k2; // linear coefficient with (u² s(Lambda)²)
  float dLdtheta; // derivative of lift with respect to the pitch angle
  float L; // Lift
};

struct RW_attitude{
  float phi;
  float theta;
  float psi;
  float sphi;
  float cphi;
  float stheta;
  float ctheta;
  float spsi;
  float cpsi;
};

struct RW_skew{
  float rad;             // Wing rotation angle in radians: from ABI message
  float deg;             // Wing rotation angle in degrees: (clone in degrees)
  float cosr;                 // cosine of wing rotation angle
  float sinr;                 // sine of wing rotation angle
  float cosr2;                // cosine² of wing rotation angle
  float sinr2;                // sine² of wing rotation angle
  float cosr3;                // cosine³ of wing rotation angle
  float sinr3;                // sine³ of wing rotation angle
};
struct RW_Model{
  struct I I;     // Inertia matrix
  float m;        // mass [kg]
  float T;        // Thrust [N]
  float P;        // Pusher thrust [N]
  struct RW_skew skew;
  struct RW_attitude att;
  struct wing_model wing;
  struct F_M_Body mF;
  struct F_M_Body mR;
  struct F_M_Body mB;
  struct F_M_Body mL;
  struct F_M_Body mP;
  struct F_M_Body ele;
  struct F_M_Body rud;
  struct F_M_Body ail;
  struct F_M_Body flp;
  float as;  // airspeed [m/s] 
  float as2; // airspeed squared [m/s²]
  float ele_pref;

};

extern bool airspeed_fake_on;
extern float airspeed_fake;
extern float ele_eff;
extern float roll_eff;
extern float yaw_eff;
extern float ele_min;

extern float rotation_angle_setpoint_deg;
extern int16_t rotation_cmd;

extern float eff_sched_pusher_time;

extern void eff_scheduling_rotwing_init(void);
extern void eff_scheduling_rotwing_periodic(void);

extern struct RW_Model RW;
#endif  // CTRL_EFF_SCHED_ROTWING_H


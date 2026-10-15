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

/** @file "firmwares/rotorcraft/oneloop/oneloop_nB.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) nB controller
 */

#ifndef ONELOOP_NB_H
#define ONELOOP_NB_H
//====================================================================================================================================
// Include the header files needed
#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "generated/airframe.h"
#include "filters/low_pass_filter.h"
#include "filters/notch_filter_float.h"
//====================================================================================================================================
// Define macros used in the C file
#ifndef ANDI_NUM_ACT
#define ANDI_NUM_ACT COMMANDS_NB_REAL
#endif

#ifndef COMMANDS_NB_VIRTUAL
#define COMMANDS_NB_VIRTUAL 0
#endif

#ifndef ANDI_NUM_VIRTUAL_ACT
#define ANDI_NUM_VIRTUAL_ACT COMMANDS_NB_VIRTUAL
#endif

#define ANDI_NUM_ACT_TOT (ANDI_NUM_ACT + ANDI_NUM_VIRTUAL_ACT)

#ifndef ANDI_OUTPUTS
#error "You must specify the number of controlled axis (outputs)"
#define ANDI_OUTPUTS 6
#endif
#define ANDI_G_SCALING 1000.0f

/** Control types.*/
#define  CTRL_ANDI 0
#define  CTRL_INDI 1
#define  CTRL_NB_ANDI 2
#define  CTRL_NB_INDI 3

//====================================================================================================================================
// Declaration of global variables
extern bool auto_fault_cmd;
extern bool use_safety_killer;
extern bool safety_killer_trigger;
extern float safety_killer_cutoff;
extern float delta_fault;
extern float oneloop_nB_filt_cutoff;
extern bool vel_ctrl_in_manual;
extern bool oneloop_nB_Z_hold;
extern bool use_push_Position;
extern bool use_push_PID;
extern float max_bank;
extern float max_phi;
extern float max_theta;
extern int8_t TestMotorIDX;
extern float max_pusher_cmd;
extern float max_fault_mot;
extern float spin_prot_max_rate;
extern float spin_prot_start_rate;
extern float spin_prot_max_cmd;
extern bool  spin_trim_on;
extern float spin_trim_rate;
extern float spin_trim_bleed_rate;
extern float spin_trim_release_rate;
extern float spin_trim;
extern float spin_prot_min_gap;
extern float ec_k3;
extern float pid_a_max;
extern float pid_v_max_manual;
extern float pid_v_max_nav;
extern bool  spin_man_up;
extern bool  spin_man_down;
extern uint8_t spin_man_pair;
extern float spin_man_pitch_dir;
extern float spin_man_ramp_rate;
extern float spin_man_blend_time;
extern float spin_man_trim_rate;
extern float spin_man_done_rate;
extern uint8_t spin_man_state;
extern float spin_man_r_ref;
extern float SpinQuadRate;
extern bool  SpinQuad;
extern bool  fault_pitch_motors;
extern bool  fault_roll_motors;
extern bool  fault_ailerons;
extern float k_K;
extern float k_P;
extern float k_I;
extern float k_D;
extern float  k1_NE_tune;
extern float  k2_NE_tune;
extern bool   ctrl_off;                                 // Turn off stabilization control with the quad motors
extern float  act_state_filt_vect_1l[ANDI_NUM_ACT];     // Filtered actuator state vector to synch with the feedback signals
extern float  actuator_state_1l[ANDI_NUM_ACT_TOT];      // Actuator state vector (including virtual actuators)
extern float  nu[ANDI_OUTPUTS];                         // Virtual control vector 
extern float  g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];  // Control effectiveness matrix
extern float  andi_u[ANDI_NUM_ACT_TOT];                 // Control command vector
extern float  andi_du[ANDI_NUM_ACT_TOT];                // Control command increment vector
extern float  psi_des_deg;                              // Desired heading [deg]    
extern bool   heading_manual;                           // Specify the heading with a slider
extern bool   yaw_stick_in_auto;                        // Adjust the heading in auto mode with the yaw stick
extern float  fwd_sideslip_gain;                        // Forward sideslip gain
extern struct FloatEulers eulers_zxy_des;               // Desired ZXY Euler angles 
extern float  psi_des_rad;                              // Desired heading [rad]
extern float  max_as;                                   // Maximum airspeed [m/s]
extern float  gi_unbounded_airspeed_sp;                 // Unbounded airspeed setpoint [m/s] (mimics guidance_indi_hybrid)
extern bool   drop_yaw;                                 // Drop yaw control 
extern bool   state_compensation_on;                    // Rotational Dynamics State Compensation
//====================================================================================================================================
// Delete once hybrid nav is fixed 
struct guidance_indi_hybrid_params {
  float pos_gain;
  float pos_gainz;
  float speed_gain;
  float speed_gainz;
  float heading_bank_gain;
  float liftd_asq;
  float liftd_p80;
  float liftd_p50;
};
extern struct guidance_indi_hybrid_params gih_params;
//====================================================================================================================================
// Declaration of the oneloop struct
struct OneloopGuidanceRef {
  float pos[3];     
  float vel[3]; 
  float acc[3];
  float jer[3];
};
struct OneloopGuidanceState {
  float pos[3];     
  float vel[3]; 
  float acc[3];
};
struct OneloopStabilizationRef {
  float att[3];     
  float att_d[3]; 
  float att_2d[3];
  float att_3d[3];
};
struct OneloopStabilizationState {
  float att[3];     
  float att_d[3]; 
  float att_2d[3];
};
struct OneloopStabilizationnBState {
  struct FloatVect3 nB;     
  struct FloatVect3 nB_d; 
  struct FloatVect3 nB_2d;
  struct FloatVect3 nB_3d;
  struct FloatVect3 nI_des;
  struct FloatVect3 nI;
  struct FloatVect3 nI_d;
  struct FloatVect3 nI_2d;
  struct FloatVect3 nI_3d;
  struct FloatVect3 mu_B;
};
struct OneloopPushnB {
  float pN_d;
  float pE_d;
  float pN;
  float pE;
  float vN_d;
  float vE_d;
  float vN;
  float vE;
  float vN_d_filt;
  float vE_d_filt;
  float vN_filt;
  float vE_filt;
  float xi;
  float xi_0;
  float push_cmd;
  float max_push_cmd;
  float max_v_d;
  float varepsilon;
  int   n;
};
struct OneloopGeneral {
  bool   half_loop;
  int    ctrl_type;
  struct OneloopGuidanceRef           gui_ref;      // Guidance References
  struct OneloopGuidanceState         gui_state;    // Guidance State
  struct OneloopStabilizationRef      sta_ref;      // Stabilization References
  struct OneloopStabilizationState    sta_state;    // Stabilization State
  struct OneloopStabilizationnBState  sta_nB_state; // nB Stabilization State
  struct OneloopPushnB                push_nB;      // Pusher controller struct
};
extern struct OneloopGeneral oneloop_nB;
//====================================================================================================================================
// Declaration of the Controller gains/poles structs
struct PolePlacement{
  float omega_n;
  float zeta;
  float p3;
};
struct Gains3rdOrder{
  float k1[3];
  float k2[3];
  float k3[3];
};
struct Gains2ndOrder{
  float k2;
  float k3;
};
/*Declaration of Reference Model and Error Controller Gains*/
extern struct PolePlacement p_att_e;
extern struct PolePlacement p_att_rm;
/*Position Loop*/
extern struct PolePlacement p_pos_e;
extern struct PolePlacement p_pos_rm;
/*Altitude Loop*/
extern struct PolePlacement p_alt_e;
extern struct PolePlacement p_alt_rm;
/*Heading Loop*/
extern struct PolePlacement p_head_e;
extern struct PolePlacement p_head_rm;
/*Gains of EC and RM*/
extern struct Gains3rdOrder k_att_e;
extern struct Gains3rdOrder k_att_rm;
extern struct Gains2ndOrder k_head_e;
extern struct Gains2ndOrder k_head_rm;
extern struct Gains3rdOrder k_pos_e;
extern struct Gains3rdOrder k_pos_rm;
//====================================================================================================================================
// Declaration of filtering structs
enum FilterType {                             // enumeration of filter types
  LOWPASS_1,                                  // First order low pass
  BUTTERWORTH_2,                              // Second order Butterworth
  BUTTERWORTH_4,                              // Fourth order Butterworth
  NOTCH                                       // Notch filter
};
struct LP_t {                                 // Low pass filter structure
  float  tau;                                 // Time constant [s]
  float  freq;                                // Cut-off frequency (currently used) [Hz]
  float  freq_set;                            // Desired cut-off frequency (to be set) [Hz]
  float  bandwidth;                           // Bandwidth of notch [Hz]
  float  meas;                                // Current measurement
  float  meas_prev;                           // Previous measurement
  float  out;                                 // Filter output
  enum FilterType filter_type;                // Type of filter used for the structural modes
  union {
    struct FirstOrderLowPass lp1;             // First order low pass filter struct
    Butterworth2LowPass bw2;                  // Second order Butterworth filter struct
    Butterworth4LowPass bw4;                  // Fourth order Butterworth filter struct
    struct SecondOrderNotchFilter notch;      // Second order notch filter struct
  } meas_filt;                                // Union of different filter types   
};
struct Oneloop_LP_t {                         // Struct containing all feedback low pass filters
  struct LP_t p;                              // Roll rate filter
  struct LP_t q;                              // Pitch rate filter
  struct LP_t r;                              // Yaw rate filter
  struct LP_t p_dot;                          // Roll acceleration filter
  struct LP_t q_dot;                          // Pitch acceleration filter
  struct LP_t r_dot;                          // Yaw acceleration filter
  struct LP_t p_ddot;                         // Roll jerk filter
  struct LP_t q_ddot;                         // Pitch jerk filter
  struct LP_t r_ddot;                         // Yaw jerk filter
  struct LP_t ax;                             // X acceleration filter
  struct LP_t ay;                             // Y acceleration filter
  struct LP_t az;                             // Z acceleration filter
};
struct Oneloop_DynFilt_t {                    // Dynamics Cancelling filter 
  float fs;                                   // Sampling frequency [Hz]        
  float sigma;                                // Bandwidth of new dynamics [rad/s]     
  float varepsilon;                           // Bandwidth of the original dynamics [rad/s]
  float u_c;                                  // Current command input
  float u_c_0;                                // Previous command input
  float mu_c_0;                               // Previous filtered command output
  float mu_c;                                 // Current filtered command output
};
extern struct Oneloop_LP_t LP;                // Instance of the struct containing all feedback low pass filters
//====================================================================================================================================
// Declaration of functions
extern void oneloop_nB_init(void);
extern void oneloop_nB_enter(bool half_loop_sp, int ctrl_type);
extern void oneloop_nB_set_failsafe_setpoint(void);
extern void oneloop_nB_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des);
extern void oneloop_nB_RM(bool half_loop, struct FloatVect3 PSA_des, bool in_flight_oneloop);
extern void oneloop_nB_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
extern void oneloop_from_nav(bool in_flight);
extern void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed);
extern void guidance_set_max_bank_angle(float max_bank);
extern void guidance_set_max_climb_speed(float max_climb_speed_quad, float max_climb_speed_fwd);
extern void guidance_set_max_descend_speed(float max_descend_speed_quad, float max_descend_speed_fwd);

#endif  // ONELOOP_NB_H

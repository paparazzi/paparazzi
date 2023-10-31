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
 * One loop (Guidance + Stabilization) ANDI controller for the rotating wing drone RW3C
 */

#ifndef ONELOOP_ANDI_H
#define ONELOOP_ANDI_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#define ANDI_G_SCALING 1000.0f

extern float pos_init[3];
extern float act_state_filt_vect_1l[ANDI_NUM_ACT];
extern float actuator_state_1l[ANDI_NUM_ACT];
extern float nu[6];

extern float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];

extern float andi_u[ANDI_NUM_ACT_TOT];
extern float andi_du[ANDI_NUM_ACT_TOT];

extern bool heading_on;
extern float v_nav_des;
extern float psi_des_deg;

// Delete once hybrid nav is fixed //////////////////////////////////////////////////////////////////////////////////
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
extern bool force_forward; 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
struct OneloopGeneral {
  bool   half_loop;
  struct OneloopGuidanceRef         gui_ref;     // Guidance References
  struct OneloopGuidanceState       gui_state;   // Guidance State
  struct OneloopStabilizationRef    sta_ref;     // Stabilization References
  struct OneloopStabilizationState  sta_state;   // Stabilization State

};

extern struct OneloopGeneral oneloop_andi;

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
extern void oneloop_andi_init(void);
extern void oneloop_andi_enter(bool half_loop_sp);
extern void oneloop_andi_set_failsafe_setpoint(void);
extern void oneloop_andi_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v);
extern void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v);
extern void oneloop_andi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
extern float convert_angle(float psi);
#endif  // ONELOOP_ANDI_H

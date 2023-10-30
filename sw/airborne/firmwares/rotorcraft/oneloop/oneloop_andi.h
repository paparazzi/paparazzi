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

extern float att_ref[3];
extern float att_d_ref[3];
extern float att_2d_ref[3];
extern float att_3d_ref[3];
extern float pos_ref[3];
extern float pos_d_ref[3];
extern float pos_2d_ref[3];
extern float pos_3d_ref[3];
extern float att_1l[3];
extern float att_d[3]; 
extern float att_2d[3];
extern float pos_1l[3];
extern float pos_d[3];
extern float pos_2d[3];
extern float pos_init[3];
extern float act_state_filt_vect_1l[ANDI_NUM_ACT];
extern float actuator_state_1l[ANDI_NUM_ACT];
extern float nu[6];

extern float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];

extern float andi_u[ANDI_NUM_ACT_TOT];
extern float andi_du[ANDI_NUM_ACT_TOT];

extern bool check_1st_nav;
extern bool heading_on;
extern float v_nav_des;

extern float p1_att;
extern float p1_head;
extern float p1_pos;
extern float p2_pos;
extern float p1_alt;

extern float psi_des_deg;
extern float rm_k_attitude;

extern float w_scale;

extern bool half_loop;


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
extern bool force_forward; 

extern void oneloop_andi_init(void);
extern void oneloop_andi_enter(bool half_loop_sp);
extern void oneloop_andi_set_failsafe_setpoint(void);
extern void oneloop_andi_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v);
extern void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v);
extern void oneloop_andi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
extern float convert_angle(float psi);
//extern int32_t stabilization_cmd[COMMANDS_NB];
#endif  // ONELOOP_ANDI_H

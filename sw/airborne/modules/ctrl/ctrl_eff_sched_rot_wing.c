/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#include "modules/ctrl/ctrl_eff_sched_rot_wing.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "modules/core/abi.h"

#ifndef SERVO_ROTATION_MECH
#error ctrl_eff_sched_rot_wing requires a servo named ROTATION_MECH
#endif


#ifndef ROT_WING_EFF_SCHED_IXX_BODY
#error "NO ROT_WING_EFF_SCHED_IXX_BODY defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IYY_BODY
#error "NO ROT_WING_EFF_SCHED_IYY_BODY defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IZZ
#error "NO ROT_WING_EFF_SCHED_IZZ defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IXX_WING
#error "NO ROT_WING_EFF_SCHED_IXX_WING defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IYY_WING
#error "NO ROT_WING_EFF_SCHED_IYY_WING defined"
#endif

#ifndef ROT_WING_EFF_SCHED_M
#error "NO ROT_WING_EFF_SCHED_M defined"
#endif

#ifndef ROT_WING_EFF_SCHED_ROLL_ARM
#error "NO ROT_WING_EFF_SCHED_ROLL_ARM defined"
#endif

#ifndef ROT_WING_EFF_SCHED_PITCH_ARM
#error "NO ROT_WING_EFF_SCHED_PITCH_ARM defined"
#endif

#ifndef ROT_WING_EFF_SCHED_HOVER_DF_DPPRZ
#error "NO ROT_WING_EFF_SCHED_HOVER_DF_DPPRZ defined"
#endif

#ifndef ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF
#error "NO ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF defined"
#endif

struct rot_wing_eff_sched_param_t eff_sched_p = {
  .Ixx_body                 = ROT_WING_EFF_SCHED_IXX_BODY,
  .Iyy_body                 = ROT_WING_EFF_SCHED_IYY_BODY,
  .Izz                      = ROT_WING_EFF_SCHED_IZZ,
  .Ixx_wing                 = ROT_WING_EFF_SCHED_IXX_WING,
  .Iyy_wing                 = ROT_WING_EFF_SCHED_IYY_WING,
  .m                        = ROT_WING_EFF_SCHED_M,
  .roll_arm                 = ROT_WING_EFF_SCHED_ROLL_ARM,
  .pitch_arm                = ROT_WING_EFF_SCHED_PITCH_ARM,
  .hover_dFdpprz            = ROT_WING_EFF_SCHED_HOVER_DF_DPPRZ,
  .hover_roll_pitch_coef    = ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF
};

struct rot_wing_eff_sched_var_t eff_sched_var;

float rotation_angle_setpoint_deg = 0; // Quad mode
int16_t rotation_cmd = 9600; // Quad mode
float pprz_angle_step = 9600. / 45.; // CMD per degree

// Telemetry
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_rotating_wing_state(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t state = 0; // Quadrotor
  float angle = eff_sched_var.wing_rotation_rad / M_PI * 180.f;
  pprz_msg_send_ROTATING_WING_STATE(trans, dev, AC_ID,
                          &state,
                          &state,
                          &angle,
                          &rotation_angle_setpoint_deg,
                          0,
                          0);
}
#endif


/** ABI binding wing position data.
 */
#ifndef CTRL_EFF_SCHED_ROT_WING_ID
#define CTRL_EFF_SCHED_ROT_WING_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(CTRL_EFF_SCHED_ROT_WING_ID)
static abi_event wing_position_ev;

static void wing_position_cb(uint8_t sender_id UNUSED, struct act_feedback_t *pos_msg, uint8_t num_act)
{
  for (int i=0; i<num_act; i++){
    if (pos_msg[i].set.position && (pos_msg[i].idx =  SERVO_ROTATION_MECH))
    {
      // Get wing rotation angle from sensor
      eff_sched_var.wing_rotation_rad = 0.5 * M_PI - pos_msg[i].position;

      // Bound wing rotation angle
      Bound(eff_sched_var.wing_rotation_rad, 0, 0.5*M_PI);
    }
  }
}


inline void ctrl_eff_sched_rot_wing_update_wing_angle_sp(void);
inline void ctrl_eff_sched_rot_wing_update_wing_angle(void);
inline void ctrl_eff_sched_rot_wing_update_MMOI(void);
inline void ctrl_eff_sched_rot_wing_update_hover_motor_effectiveness(void);

void ctrl_eff_sched_rot_wing_init(void)
{
  // Initialize variables to quad values
  eff_sched_var.Ixx               = eff_sched_p.Ixx_body + eff_sched_p.Ixx_wing;             
  eff_sched_var.Iyy               = eff_sched_p.Iyy_body + eff_sched_p.Iyy_wing;
  eff_sched_var.wing_rotation_rad = 0;
  eff_sched_var.cosr              = 1;             
  eff_sched_var.sinr              = 0;              
  eff_sched_var.cosr2             = 1;              
  eff_sched_var.sinr2             = 0;
  eff_sched_var.cosr3             = 1;           

  // Set moment derivative variables
  eff_sched_var.pitch_motor_dMdpprz = eff_sched_p.hover_dFdpprz * eff_sched_p.pitch_arm;
  eff_sched_var.roll_motor_dMdpprz  = eff_sched_p.hover_dFdpprz * eff_sched_p.roll_arm;

  AbiBindMsgACT_FEEDBACK(CTRL_EFF_SCHED_ROT_WING_ID, &wing_position_ev, wing_position_cb);


  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTATING_WING_STATE, send_rotating_wing_state);
  #endif
}

void ctrl_eff_sched_rot_wing_periodic(void)
{
  // your periodic code here.
  // freq = 10.0 Hz
  ctrl_eff_sched_rot_wing_update_wing_angle_sp();
  ctrl_eff_sched_rot_wing_update_wing_angle();
  ctrl_eff_sched_rot_wing_update_MMOI();

  // Update the effectiveness values
  ctrl_eff_sched_rot_wing_update_hover_motor_effectiveness();

}

void ctrl_eff_sched_rot_wing_update_wing_angle_sp(void)
{
  rotation_cmd = MAX_PPRZ - (int16_t)(rotation_angle_setpoint_deg * pprz_angle_step);
  // Calulcate rotation_cmd
  Bound(rotation_cmd, -9600, 9600);
}

void ctrl_eff_sched_rot_wing_update_wing_angle(void)
{
  // Calculate sin and cosines of rotation
  eff_sched_var.cosr = cosf(eff_sched_var.wing_rotation_rad);
  eff_sched_var.sinr = sinf(eff_sched_var.wing_rotation_rad);

  eff_sched_var.cosr2 = eff_sched_var.cosr * eff_sched_var.cosr;
  eff_sched_var.sinr2 = eff_sched_var.sinr * eff_sched_var.sinr;

  eff_sched_var.cosr3 = eff_sched_var.cosr2 * eff_sched_var.cosr;
}

void ctrl_eff_sched_rot_wing_update_MMOI(void)
{
  eff_sched_var.Ixx = eff_sched_p.Ixx_body + eff_sched_var.cosr2 * eff_sched_p.Ixx_wing + eff_sched_var.sinr2 * eff_sched_p.Iyy_wing;
  eff_sched_var.Iyy = eff_sched_p.Iyy_body + eff_sched_var.sinr2 * eff_sched_p.Ixx_wing + eff_sched_var.cosr2 * eff_sched_p.Iyy_wing; 

  // Bound inertia
  Bound(eff_sched_var.Ixx, 0.01, 100.);
  Bound(eff_sched_var.Iyy, 0.01, 100.);
}

void ctrl_eff_sched_rot_wing_update_hover_motor_effectiveness(void)
{
  // Pitch motor effectiveness 

  float pitch_motor_q_eff = eff_sched_var.pitch_motor_dMdpprz / eff_sched_var.Iyy;

  // Roll motor effectiveness

  float roll_motor_p_eff = eff_sched_var.roll_motor_dMdpprz * eff_sched_var.cosr / eff_sched_var.Ixx;
  // float roll_motor_q_eff = eff_sched_var.roll_motor_dMdpprz * eff_sched_var.sinr *
  //  (eff_sched_p.hover_roll_pitch_coef[0] + eff_sched_p.hover_roll_pitch_coef[1] * eff_sched_var.cosr2) / eff_sched_var.Iyy;

  float roll_motor_q_eff = (eff_sched_var.roll_motor_dMdpprz * eff_sched_var.sinr / eff_sched_var.Iyy) * (eff_sched_p.hover_roll_pitch_coef[0] + eff_sched_p.hover_roll_pitch_coef[1] * eff_sched_var.cosr3);

  // Update front pitch motor q effectiveness
  g1g2[1][0] = pitch_motor_q_eff;   // pitch effectiveness front motor

  // Update back motor q effectiveness
  g1g2[1][2] = -pitch_motor_q_eff;  // pitch effectiveness back motor

  // Update right motor p and q effectiveness
  g1g2[0][1] = -roll_motor_p_eff;   // roll effectiveness right motor
  g1g2[1][1] = roll_motor_q_eff;    // pitch effectiveness right motor

  // Update left motor p and q effectiveness
  g1g2[0][3] = roll_motor_p_eff;    // roll effectiveness left motor
  g1g2[1][3] = -roll_motor_q_eff;   // pitch effectiveness left motor
}


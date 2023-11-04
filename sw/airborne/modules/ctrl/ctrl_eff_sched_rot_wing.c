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

#include "generated/airframe.h"
#include "state.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

#include "modules/actuators/actuators.h"
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

#ifndef ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH
#error "NO ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH defined"
#endif

#ifndef ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL
#error "NO ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL defined"
#endif

#ifndef ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF
#error "NO ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF defined"
#endif

#ifndef ROT_WING_EFF_SCHED_HOVER_ROLL_ROLL_COEF
#error "NO ROT_WING_EFF_SCHED_HOVER_ROLL_ROLL_COEF defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_ELEVATOR
#error "NO ROT_WING_EFF_SCHED_K_ELEVATOR defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_RUDDER
#error "NO ROT_WING_EFF_SCHED_K_RUDDER defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_AILERON
#error "NO ROT_WING_EFF_SCHED_K_AILERON defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_FLAPERON
#error "NO ROT_WING_EFF_SCHED_K_FLAPERON defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_PUSHER
#error "NO ROT_WING_EFF_SCHED_K_PUSHER defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_ELEVATOR_DEFLECTION
#error "NO ROT_WING_EFF_SCHED_K_ELEVATOR_DEFLECTION defined"
#endif

#ifndef ROT_WING_EFF_SCHED_D_RUDDER_D_PPRZ
#error "NO ROT_WING_EFF_SCHED_D_RUDDER_D_PPRZ defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_RPM_PPRZ_PUSHER
#error "NO ROT_WING_EFF_SCHED_K_RPM_PPRZ_PUSHER defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_LIFT_WING
#error "NO ROT_WING_EFF_SCHED_K_LIFT_WING defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_LIFT_FUSELAGE
#error "NO ROT_WING_EFF_SCHED_K_LIFT_FUSELAGE defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_LIFT_TAIL
#error "NO ROT_WING_EFF_SCHED_K_LIFT_TAIL defined"
#endif

struct rot_wing_eff_sched_param_t eff_sched_p = {
  .Ixx_body                 = ROT_WING_EFF_SCHED_IXX_BODY,
  .Iyy_body                 = ROT_WING_EFF_SCHED_IYY_BODY,
  .Izz                      = ROT_WING_EFF_SCHED_IZZ,
  .Ixx_wing                 = ROT_WING_EFF_SCHED_IXX_WING,
  .Iyy_wing                 = ROT_WING_EFF_SCHED_IYY_WING,
  .m                        = ROT_WING_EFF_SCHED_M,
  .hover_roll_pitch_coef    = ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF,
  .hover_roll_roll_coef     = ROT_WING_EFF_SCHED_HOVER_ROLL_ROLL_COEF,
  .k_elevator               = ROT_WING_EFF_SCHED_K_ELEVATOR,
  .k_rudder                 = ROT_WING_EFF_SCHED_K_RUDDER,
  .k_aileron                = ROT_WING_EFF_SCHED_K_AILERON,
  .k_flaperon               = ROT_WING_EFF_SCHED_K_FLAPERON,
  .k_pusher                 = ROT_WING_EFF_SCHED_K_PUSHER,
  .k_elevator_deflection    = ROT_WING_EFF_SCHED_K_ELEVATOR_DEFLECTION,
  .d_rudder_d_pprz          = ROT_WING_EFF_SCHED_D_RUDDER_D_PPRZ,
  .k_rpm_pprz_pusher        = ROT_WING_EFF_SCHED_K_RPM_PPRZ_PUSHER,
  .k_lift_wing              = ROT_WING_EFF_SCHED_K_LIFT_WING,
  .k_lift_fuselage          = ROT_WING_EFF_SCHED_K_LIFT_FUSELAGE,
  .k_lift_tail              = ROT_WING_EFF_SCHED_K_LIFT_TAIL
};

struct rot_wing_eff_sched_var_t eff_sched_var;

inline void ctrl_eff_sched_rot_wing_update_wing_angle(void);
inline void ctrl_eff_sched_rot_wing_update_MMOI(void);
inline void ctrl_eff_sched_rot_wing_update_cmd(void);
inline void ctrl_eff_sched_rot_wing_update_airspeed(void);
inline void ctrl_eff_sched_rot_wing_update_hover_motor_effectiveness(void);
inline void ctrl_eff_sched_rot_wing_update_elevator_effectiveness(void);
inline void ctrl_eff_sched_rot_wing_update_rudder_effectiveness(void);
inline void ctrl_eff_sched_rot_wing_update_aileron_effectiveness(void);
inline void ctrl_eff_sched_rot_wing_update_flaperon_effectiveness(void);
inline void ctrl_eff_sched_rot_wing_update_pusher_effectiveness(void);
inline void ctrl_eff_sched_rot_wing_schedule_liftd(void);

inline float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED);
inline void stabilization_indi_set_wls_settings(float use_increment);


/** ABI binding wing position data.
 */
#ifndef WING_ROTATION_CAN_ROT_WING_ID
#define WING_ROTATION_CAN_ROT_WING_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(WING_ROTATION_CAN_ROT_WING_ID)
static abi_event wing_position_ev;

static void wing_position_cb(uint8_t sender_id UNUSED, struct act_feedback_t *pos_msg, uint8_t num_act)
{
  for (int i=0; i<num_act; i++){
    if (pos_msg[i].set.position && (pos_msg[i].idx == SERVO_ROTATION_MECH))
    {
      // Get wing rotation angle from sensor
      eff_sched_var.wing_rotation_rad = 0.5 * M_PI - pos_msg[i].position;

      // Bound wing rotation angle
      Bound(eff_sched_var.wing_rotation_rad, 0, 0.5 * M_PI);
    }
  }
}

void ctrl_eff_sched_rot_wing_init(void)
{
  // Initialize variables to quad values
  eff_sched_var.Ixx               = eff_sched_p.Ixx_body + eff_sched_p.Ixx_wing;             
  eff_sched_var.Iyy               = eff_sched_p.Iyy_body + eff_sched_p.Iyy_wing;
  eff_sched_var.wing_rotation_rad = 0; // ABI input
  eff_sched_var.wing_rotation_deg = 0;
  eff_sched_var.cosr              = 1;             
  eff_sched_var.sinr              = 0;              
  eff_sched_var.cosr2             = 1;              
  eff_sched_var.sinr2             = 0;
  eff_sched_var.sinr3             = 0;         

  // Set moment derivative variables
  eff_sched_var.pitch_motor_dMdpprz = ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH;
  eff_sched_var.roll_motor_dMdpprz  = ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL;

  eff_sched_var.cmd_elevator = 0;
  eff_sched_var.cmd_pusher_scaled = 0;
  eff_sched_var.cmd_T_mean_scaled = 0;

  eff_sched_var.airspeed = 0;
  eff_sched_var.airspeed2 = 0;

  // Get wing angle
  AbiBindMsgACT_FEEDBACK(WING_ROTATION_CAN_ROT_WING_ID, &wing_position_ev, wing_position_cb);
}

void ctrl_eff_sched_rot_wing_periodic(void)
{
  // your periodic code here.
  // freq = 10.0 Hz
  ctrl_eff_sched_rot_wing_update_wing_angle();
  ctrl_eff_sched_rot_wing_update_MMOI();
  ctrl_eff_sched_rot_wing_update_cmd();
  ctrl_eff_sched_rot_wing_update_airspeed();

  // Update the effectiveness values
  ctrl_eff_sched_rot_wing_update_hover_motor_effectiveness();
  ctrl_eff_sched_rot_wing_update_elevator_effectiveness();
  ctrl_eff_sched_rot_wing_update_rudder_effectiveness();
  ctrl_eff_sched_rot_wing_update_aileron_effectiveness();
  ctrl_eff_sched_rot_wing_update_flaperon_effectiveness();
  ctrl_eff_sched_rot_wing_update_pusher_effectiveness();
  ctrl_eff_sched_rot_wing_schedule_liftd();
}

void ctrl_eff_sched_rot_wing_update_wing_angle(void)
{
  // Calculate sin and cosines of rotation
  eff_sched_var.wing_rotation_deg = eff_sched_var.wing_rotation_rad / M_PI * 180.;

  eff_sched_var.cosr = cosf(eff_sched_var.wing_rotation_rad);
  eff_sched_var.sinr = sinf(eff_sched_var.wing_rotation_rad);

  eff_sched_var.cosr2 = eff_sched_var.cosr * eff_sched_var.cosr;
  eff_sched_var.sinr2 = eff_sched_var.sinr * eff_sched_var.sinr;

  eff_sched_var.sinr3 = eff_sched_var.sinr2 * eff_sched_var.sinr;

}

void ctrl_eff_sched_rot_wing_update_MMOI(void)
{
  eff_sched_var.Ixx = eff_sched_p.Ixx_body + eff_sched_var.cosr2 * eff_sched_p.Ixx_wing + eff_sched_var.sinr2 * eff_sched_p.Iyy_wing;
  eff_sched_var.Iyy = eff_sched_p.Iyy_body + eff_sched_var.sinr2 * eff_sched_p.Ixx_wing + eff_sched_var.cosr2 * eff_sched_p.Iyy_wing; 

  // Bound inertia
  Bound(eff_sched_var.Ixx, 0.01, 100.);
  Bound(eff_sched_var.Iyy, 0.01, 100.);
}

void ctrl_eff_sched_rot_wing_update_cmd(void)
{
  eff_sched_var.cmd_elevator = actuators_pprz[SERVO_SERVO_ELEVATOR];
  eff_sched_var.cmd_pusher_scaled = actuators_pprz[SERVO_MOTOR_PUSH] * 0.000853229; // Scaled with 8181 / 9600 / 1000
  eff_sched_var.cmd_T_mean_scaled = (actuators_pprz[SERVO_MOTOR_FRONT] + actuators_pprz[SERVO_MOTOR_RIGHT] + actuators_pprz[SERVO_MOTOR_BACK] + actuators_pprz[SERVO_MOTOR_LEFT]) / 4. * 0.000853229; // Scaled with 8181 / 9600 / 1000
}

void ctrl_eff_sched_rot_wing_update_airspeed(void)
{
  eff_sched_var.airspeed = stateGetAirspeed_f();
  Bound(eff_sched_var.airspeed, 0. , 30.);
  eff_sched_var.airspeed2 = eff_sched_var.airspeed * eff_sched_var.airspeed;
  Bound(eff_sched_var.airspeed2, 0. , 900.);
}

void ctrl_eff_sched_rot_wing_update_hover_motor_effectiveness(void)
{
  // Pitch motor effectiveness 

  float pitch_motor_q_eff = eff_sched_var.pitch_motor_dMdpprz / eff_sched_var.Iyy;

  // Roll motor effectiveness

  float roll_motor_p_eff = (eff_sched_var.roll_motor_dMdpprz * eff_sched_var.cosr + eff_sched_p.hover_roll_roll_coef[0] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.airspeed * eff_sched_var.cosr) / eff_sched_var.Ixx;
  Bound(roll_motor_p_eff, 0.00001, 1);
  float roll_motor_airspeed_compensation = eff_sched_p.hover_roll_roll_coef[1] * eff_sched_var.airspeed * eff_sched_var.cosr / eff_sched_var.Ixx;
  float roll_motor_p_eff_compensated = roll_motor_p_eff + roll_motor_airspeed_compensation;
  Bound(roll_motor_p_eff_compensated, 0.00001, 1);

  float roll_motor_q_eff = (eff_sched_p.hover_roll_pitch_coef[0] * eff_sched_var.wing_rotation_rad + eff_sched_p.hover_roll_pitch_coef[1] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.sinr) / eff_sched_var.Iyy;
  Bound(roll_motor_q_eff, 0, 1);

  // Update front pitch motor q effectiveness
  g1g2[1][0] = pitch_motor_q_eff;   // pitch effectiveness front motor

  // Update back motor q effectiveness
  g1g2[1][2] = -pitch_motor_q_eff;  // pitch effectiveness back motor

  // Update right motor p and q effectiveness
  g1g2[0][1] = -roll_motor_p_eff;   // roll effectiveness right motor (no airspeed compensation)
  g1g2[1][1] = roll_motor_q_eff;    // pitch effectiveness right motor

  // Update left motor p and q effectiveness
  g1g2[0][3] = roll_motor_p_eff_compensated;  // roll effectiveness left motor
  g1g2[1][3] = -roll_motor_q_eff;   // pitch effectiveness left motor
}

void ctrl_eff_sched_rot_wing_update_elevator_effectiveness(void)
{
  float de = eff_sched_p.k_elevator_deflection[0] + eff_sched_p.k_elevator_deflection[1] * eff_sched_var.cmd_elevator;

  float dMyde = (eff_sched_p.k_elevator[0] * de * eff_sched_var.airspeed2 +
                 eff_sched_p.k_elevator[1] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_pusher_scaled * eff_sched_var.airspeed + 
                 eff_sched_p.k_elevator[2] * eff_sched_var.airspeed2) / 10000.;

  float dMydpprz = dMyde * eff_sched_p.k_elevator_deflection[1];

  // Convert moment to effectiveness
  float eff_y_elev = dMydpprz / eff_sched_var.Iyy;

  Bound(eff_y_elev, 0.00001, 0.1);

  g1g2[1][5] = eff_y_elev;
}

void ctrl_eff_sched_rot_wing_update_rudder_effectiveness(void)
{
  float dMzdr = (eff_sched_p.k_rudder[0] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_T_mean_scaled + 
                 eff_sched_p.k_rudder[1] * eff_sched_var.cmd_T_mean_scaled * eff_sched_var.airspeed2 * eff_sched_var.cosr + 
                 eff_sched_p.k_rudder[2] * eff_sched_var.airspeed2) / 10000.;

  // Convert moment to effectiveness

  float dMzdpprz = dMzdr * eff_sched_p.d_rudder_d_pprz;

  // Convert moment to effectiveness
  float eff_z_rudder = dMzdpprz / eff_sched_p.Izz;

  Bound(eff_z_rudder, 0.00001, 0.1);

  g1g2[2][4] = eff_z_rudder;
}

void ctrl_eff_sched_rot_wing_update_aileron_effectiveness(void)
{
  float dMxdpprz = (eff_sched_p.k_aileron * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
  float eff_x_aileron = dMxdpprz / eff_sched_var.Ixx;
  Bound(eff_x_aileron, 0, 0.005)
  g1g2[0][6] = eff_x_aileron;
}

void ctrl_eff_sched_rot_wing_update_flaperon_effectiveness(void)
{
  float dMxdpprz = (eff_sched_p.k_flaperon * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
  float eff_x_flap_aileron = dMxdpprz / eff_sched_var.Ixx;
  Bound(eff_x_flap_aileron, 0, 0.005)
  g1g2[0][7] = eff_x_flap_aileron;
}

void ctrl_eff_sched_rot_wing_update_pusher_effectiveness(void)
{
  float rpmP = eff_sched_p.k_rpm_pprz_pusher[0] + eff_sched_p.k_rpm_pprz_pusher[1] * eff_sched_var.cmd_pusher_scaled + eff_sched_p.k_rpm_pprz_pusher[2] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_pusher_scaled;

  float dFxdrpmP = eff_sched_p.k_pusher[0]*rpmP + eff_sched_p.k_pusher[1] * eff_sched_var.airspeed;
  float drpmPdpprz = eff_sched_p.k_rpm_pprz_pusher[1] + 2. * eff_sched_p.k_rpm_pprz_pusher[2] * eff_sched_var.cmd_pusher_scaled;

  float eff_pusher = (dFxdrpmP * drpmPdpprz / eff_sched_p.m) / 10000.;

  Bound(eff_pusher, 0.00030, 0.0015);
  g1g2[4][8] = eff_pusher;
}

float ctrl_eff_sched_rot_wing_lift_d = 0.0f;

void ctrl_eff_sched_rot_wing_schedule_liftd(void) 
{
  float lift_d_wing = (eff_sched_p.k_lift_wing[0] + eff_sched_p.k_lift_wing[1] * eff_sched_var.sinr2) * eff_sched_var.airspeed2 / eff_sched_p.m;
  float lift_d_fuselage = eff_sched_p.k_lift_fuselage * eff_sched_var.airspeed2 / eff_sched_p.m;
  float lift_d_tail = eff_sched_p.k_lift_tail * eff_sched_var.airspeed2 / eff_sched_p.m;

  float lift_d = lift_d_wing + lift_d_fuselage + lift_d_tail;
  if (eff_sched_var.wing_rotation_deg < 60.) {
    lift_d = 0.0;
  }
  Bound(lift_d, -130., 0.);
  ctrl_eff_sched_rot_wing_lift_d = lift_d;
}

// Override standard LIFT_D function
float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED) {
  return ctrl_eff_sched_rot_wing_lift_d;
}

void stabilization_indi_set_wls_settings(float use_increment)
{
   // Calculate the min and max increments
    for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
      du_min_stab_indi[i] = -MAX_PPRZ * act_is_servo[i] - use_increment*actuator_state_filt_vect[i];
      du_max_stab_indi[i] = MAX_PPRZ - use_increment*actuator_state_filt_vect[i];
      du_pref_stab_indi[i] = act_pref[i] - use_increment*actuator_state_filt_vect[i];
      if (i == 5) {
        du_pref_stab_indi[i] = 0; // Set change in prefered state to 0 for elevator
        du_min_stab_indi[i] = - use_increment*actuator_state_filt_vect[i]; // cmd 0 is lowest position for elevator
      }
  }

  #if ROTWING_V3B
  float min_pprz_cmd_ail = -MAX_PPRZ;
  if (eff_sched_var.wing_rotation_deg < 15) {
    min_pprz_cmd_ail = 0;
  }
  Bound(min_pprz_cmd_ail, -9600, 0);

  du_min_stab_indi[6] = min_pprz_cmd_ail - use_increment*actuators_pprz[6];

  float min_pprz_cmd_flap_ail = -MAX_PPRZ;
  if (eff_sched_var.wing_rotation_deg < 38) {
    min_pprz_cmd_flap_ail = -1000;
  } if (eff_sched_var.wing_rotation_deg > 50) {
    min_pprz_cmd_flap_ail = -9600;
  } else {
    min_pprz_cmd_flap_ail = -5.596578906693223 * eff_sched_var.wing_rotation_deg * eff_sched_var.wing_rotation_deg * eff_sched_var.wing_rotation_deg + 654.186408367317 * eff_sched_var.wing_rotation_deg * eff_sched_var.wing_rotation_deg - 25577.0135504177 * eff_sched_var.wing_rotation_deg + 333307.855118805;
  }

  Bound(min_pprz_cmd_flap_ail, -9600, 0);

  du_min_stab_indi[7] = min_pprz_cmd_flap_ail - use_increment*actuators_pprz[7];
  #endif //ROTWING_V3B
}
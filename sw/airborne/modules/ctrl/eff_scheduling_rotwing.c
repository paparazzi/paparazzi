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

/** @file "modules/ctrl/eff_scheduling_rotwing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#include "modules/ctrl/eff_scheduling_rotwing.h"

#include "generated/airframe.h"
#include "state.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

#include "autopilot.h"

#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"

#ifndef SERVO_ROTATION_MECH_IDX
#error ctrl_eff_sched_rotwing requires a servo named ROTATION_MECH_IDX
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

#ifndef ROTWING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH
#error "NO ROTWING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH defined"
#endif

#ifndef ROTWING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL
#error "NO ROTWING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL defined"
#endif

#ifndef ROTWING_EFF_SCHED_HOVER_ROLL_PITCH_COEF
#error "NO ROTWING_EFF_SCHED_HOVER_ROLL_PITCH_COEF defined"
#endif

#ifndef ROTWING_EFF_SCHED_HOVER_ROLL_ROLL_COEF
#error "NO ROTWING_EFF_SCHED_HOVER_ROLL_ROLL_COEF defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_ELEVATOR
#error "NO ROTWING_EFF_SCHED_K_ELEVATOR defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_RUDDER
#error "NO ROTWING_EFF_SCHED_K_RUDDER defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_AILERON
#error "NO ROTWING_EFF_SCHED_K_AILERON defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_FLAPERON
#error "NO ROTWING_EFF_SCHED_K_FLAPERON defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_PUSHER
#error "NO ROTWING_EFF_SCHED_K_PUSHER defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_ELEVATOR_DEFLECTION
#error "NO ROTWING_EFF_SCHED_K_ELEVATOR_DEFLECTION defined"
#endif

#ifndef ROTWING_EFF_SCHED_D_RUDDER_D_PPRZ
#error "NO ROTWING_EFF_SCHED_D_RUDDER_D_PPRZ defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_RPM_PPRZ_PUSHER
#error "NO ROTWING_EFF_SCHED_K_RPM_PPRZ_PUSHER defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_LIFT_WING
#error "NO ROTWING_EFF_SCHED_K_LIFT_WING defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_LIFT_FUSELAGE
#error "NO ROTWING_EFF_SCHED_K_LIFT_FUSELAGE defined"
#endif

#ifndef ROTWING_EFF_SCHED_K_LIFT_TAIL
#error "NO ROTWING_EFF_SCHED_K_LIFT_TAIL defined"
#endif

struct rotwing_eff_sched_param_t eff_sched_p = {
  .Ixx_body                 = ROTWING_EFF_SCHED_IXX_BODY,
  .Iyy_body                 = ROTWING_EFF_SCHED_IYY_BODY,
  .Izz                      = ROTWING_EFF_SCHED_IZZ,
  .Ixx_wing                 = ROTWING_EFF_SCHED_IXX_WING,
  .Iyy_wing                 = ROTWING_EFF_SCHED_IYY_WING,
  .m                        = ROTWING_EFF_SCHED_M,
  .DMdpprz_hover_pitch      = ROTWING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH,
  .DMdpprz_hover_roll       = ROTWING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL,
  .hover_roll_pitch_coef    = ROTWING_EFF_SCHED_HOVER_ROLL_PITCH_COEF,
  .hover_roll_roll_coef     = ROTWING_EFF_SCHED_HOVER_ROLL_ROLL_COEF,
  .k_elevator               = ROTWING_EFF_SCHED_K_ELEVATOR,
  .k_rudder                 = ROTWING_EFF_SCHED_K_RUDDER,
  .k_aileron                = ROTWING_EFF_SCHED_K_AILERON,
  .k_flaperon               = ROTWING_EFF_SCHED_K_FLAPERON,
  .k_pusher                 = ROTWING_EFF_SCHED_K_PUSHER,
  .k_elevator_deflection    = ROTWING_EFF_SCHED_K_ELEVATOR_DEFLECTION,
  .d_rudder_d_pprz          = ROTWING_EFF_SCHED_D_RUDDER_D_PPRZ,
  .k_rpm_pprz_pusher        = ROTWING_EFF_SCHED_K_RPM_PPRZ_PUSHER,
  .k_lift_wing              = ROTWING_EFF_SCHED_K_LIFT_WING,
  .k_lift_fuselage          = ROTWING_EFF_SCHED_K_LIFT_FUSELAGE,
  .k_lift_tail              = ROTWING_EFF_SCHED_K_LIFT_TAIL
};

int32_t rw_flap_offset = 0;

// for negative values, still should be low_lim < up_lim
inline float bound_or_zero(float value, float low_lim, float up_lim) {
  float output = value;
  if (low_lim > 0.f) {
    if (value < low_lim) {
      output = 0.f;
    } else if (value > up_lim) {
      output = up_lim;
    }
  } else {
    if (value > up_lim) {
      output = 0.f;
    } else if (value < low_lim) {
      output = low_lim;
    }
  }
  return output;
}

float eff_sched_pusher_time = 0.002;

struct rotwing_eff_sched_var_t eff_sched_var;

inline void eff_scheduling_rotwing_update_wing_angle(void);
inline void eff_scheduling_rotwing_update_MMOI(void);
inline void eff_scheduling_rotwing_update_cmd(void);
inline void eff_scheduling_rotwing_update_airspeed(void);
inline void eff_scheduling_rotwing_update_hover_motor_effectiveness(void);
inline void eff_scheduling_rotwing_update_elevator_effectiveness(void);
inline void eff_scheduling_rotwing_update_rudder_effectiveness(void);
inline void eff_scheduling_rotwing_update_aileron_effectiveness(void);
inline void eff_scheduling_rotwing_update_flaperon_effectiveness(void);
inline void eff_scheduling_rotwing_update_pusher_effectiveness(void);
inline void eff_scheduling_rotwing_schedule_liftd(void);

inline float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED);
void stabilization_indi_set_wls_settings(void);


/** ABI binding wing position data.
 */
#ifndef WING_ROTATION_CAN_ROTWING_ID
#define WING_ROTATION_CAN_ROTWING_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(WING_ROTATION_CAN_ROTWING_ID)
static abi_event wing_position_ev;

static void wing_position_cb(uint8_t sender_id UNUSED, struct act_feedback_t *pos_msg, uint8_t num_act)
{
  for (int i=0; i<num_act; i++){
    if (pos_msg[i].set.position && (pos_msg[i].idx == SERVO_ROTATION_MECH_IDX))
    {
      // Get wing rotation angle from sensor
      eff_sched_var.wing_rotation_rad = 0.5 * M_PI - pos_msg[i].position;

      // Bound wing rotation angle
      Bound(eff_sched_var.wing_rotation_rad, 0, 0.5 * M_PI);
    }
  }
}

void eff_scheduling_rotwing_init(void)
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
  float hover_thrust = 6000;
  eff_sched_var.pitch_motor_dMdpprz = (eff_sched_p.DMdpprz_hover_pitch[0] + 2*hover_thrust * eff_sched_p.DMdpprz_hover_pitch[1]) / 10000.; // Dmdpprz hover pitch for hover thrust
  eff_sched_var.roll_motor_dMdpprz  = (eff_sched_p.DMdpprz_hover_roll[0] + 2*hover_thrust * eff_sched_p.DMdpprz_hover_roll[1]) / 10000.; // Dmdpprz hover roll for hover thrust

  eff_sched_var.cmd_elevator = 0;
  eff_sched_var.cmd_pusher = 0;
  eff_sched_var.cmd_pusher_scaled = 0;
  eff_sched_var.cmd_T_mean_scaled = 0;

  eff_sched_var.airspeed = 0;
  eff_sched_var.airspeed2 = 0;

  // Get wing angle
  AbiBindMsgACT_FEEDBACK(WING_ROTATION_CAN_ROTWING_ID, &wing_position_ev, wing_position_cb);
}

void eff_scheduling_rotwing_periodic(void)
{
  // your periodic code here.
  // freq = 10.0 Hz
  eff_scheduling_rotwing_update_wing_angle();
  eff_scheduling_rotwing_update_MMOI();
  eff_scheduling_rotwing_update_cmd();
  eff_scheduling_rotwing_update_airspeed();

  // Update the effectiveness values
  eff_scheduling_rotwing_update_hover_motor_effectiveness();
  eff_scheduling_rotwing_update_elevator_effectiveness();
  eff_scheduling_rotwing_update_rudder_effectiveness();
  eff_scheduling_rotwing_update_aileron_effectiveness();
  eff_scheduling_rotwing_update_flaperon_effectiveness();
  eff_scheduling_rotwing_update_pusher_effectiveness();
  eff_scheduling_rotwing_schedule_liftd();
}

void eff_scheduling_rotwing_update_wing_angle(void)
{
  // Calculate sin and cosines of rotation
  eff_sched_var.wing_rotation_deg = eff_sched_var.wing_rotation_rad / M_PI * 180.;

  eff_sched_var.cosr = cosf(eff_sched_var.wing_rotation_rad);
  eff_sched_var.sinr = sinf(eff_sched_var.wing_rotation_rad);

  eff_sched_var.cosr2 = eff_sched_var.cosr * eff_sched_var.cosr;
  eff_sched_var.sinr2 = eff_sched_var.sinr * eff_sched_var.sinr;

  eff_sched_var.sinr3 = eff_sched_var.sinr2 * eff_sched_var.sinr;

}

void eff_scheduling_rotwing_update_MMOI(void)
{
  eff_sched_var.Ixx = eff_sched_p.Ixx_body + eff_sched_var.cosr2 * eff_sched_p.Ixx_wing + eff_sched_var.sinr2 * eff_sched_p.Iyy_wing;
  eff_sched_var.Iyy = eff_sched_p.Iyy_body + eff_sched_var.sinr2 * eff_sched_p.Ixx_wing + eff_sched_var.cosr2 * eff_sched_p.Iyy_wing;

  // Bound inertia
  Bound(eff_sched_var.Ixx, 0.01, 100.);
  Bound(eff_sched_var.Iyy, 0.01, 100.);
}

void eff_scheduling_rotwing_update_cmd(void)
{
  // These indexes depend on the INDI sequence, not the actuator IDX
  eff_sched_var.cmd_elevator = actuator_state_filt_vect[5];
  eff_sched_var.cmd_pusher = actuator_state_filt_vect[8];
  eff_sched_var.cmd_pusher_scaled = actuator_state_filt_vect[8] * 0.000853229; // Scaled with 8181 / 9600 / 1000
  eff_sched_var.cmd_T_mean_scaled = (actuator_state_filt_vect[0] + actuator_state_filt_vect[1] + actuator_state_filt_vect[2] + actuator_state_filt_vect[3]) / 4. * 0.000853229; // Scaled with 8181 / 9600 / 1000
}

void eff_scheduling_rotwing_update_airspeed(void)
{
  eff_sched_var.airspeed = stateGetAirspeed_f();
  Bound(eff_sched_var.airspeed, 0. , 30.);
  eff_sched_var.airspeed2 = eff_sched_var.airspeed * eff_sched_var.airspeed;
  Bound(eff_sched_var.airspeed2, 0. , 900.);
}

void eff_scheduling_rotwing_update_hover_motor_effectiveness(void)
{
  float cmd_quat[4];
  float dM_dpprz[4];
  // Quadratic thrust (and therefore moment) model of the hover propellers
  for (uint8_t i = 0; i < 4; i++) {
    cmd_quat[i] = actuator_state_filt_vect[i];
    Bound(cmd_quat[i], 2500, MAX_PPRZ);

    if(i==0 || i==2) { // pitch motors
      dM_dpprz[i] = (eff_sched_p.DMdpprz_hover_pitch[0] + 2*cmd_quat[i] * eff_sched_p.DMdpprz_hover_pitch[1]) / 10000.;
      // Bound dM_dpprz to half and 2 times the hover effectiveness
      Bound(dM_dpprz[i], eff_sched_var.pitch_motor_dMdpprz * 0.5, eff_sched_var.pitch_motor_dMdpprz * 2.0);
    } else { // roll motors
      dM_dpprz[i] = (eff_sched_p.DMdpprz_hover_roll[0] + 2*cmd_quat[i] * eff_sched_p.DMdpprz_hover_roll[1]) / 10000.;
      Bound(dM_dpprz[i], eff_sched_var.roll_motor_dMdpprz * 0.5, eff_sched_var.roll_motor_dMdpprz * 2.0);
    }
  }

  // Roll motor effectiveness
  float dM_dpprz_right  = dM_dpprz[1];
  float dM_dpprz_left   = dM_dpprz[3];;

  float roll_motor_p_eff_right = -(dM_dpprz_right * eff_sched_var.cosr + eff_sched_p.hover_roll_roll_coef[0] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.airspeed * eff_sched_var.cosr) / eff_sched_var.Ixx;
  roll_motor_p_eff_right = bound_or_zero(roll_motor_p_eff_right, -1.f, -0.00001f);

  float roll_motor_p_eff_left = (dM_dpprz_left * eff_sched_var.cosr + eff_sched_p.hover_roll_roll_coef[0] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.airspeed * eff_sched_var.cosr) / eff_sched_var.Ixx;
  if(autopilot.in_flight) {
    float roll_motor_airspeed_compensation = eff_sched_p.hover_roll_roll_coef[1] * eff_sched_var.airspeed * eff_sched_var.cosr / eff_sched_var.Ixx;
    roll_motor_p_eff_left += roll_motor_airspeed_compensation;
  }
  roll_motor_p_eff_left = bound_or_zero(roll_motor_p_eff_left, 0.00001f, 1.f);

  float roll_motor_q_eff = (eff_sched_p.hover_roll_pitch_coef[0] * eff_sched_var.wing_rotation_rad + eff_sched_p.hover_roll_pitch_coef[1] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.sinr) / eff_sched_var.Iyy;
  Bound(roll_motor_q_eff, 0, 1);

  // Update front pitch motor q effectiveness
  g1g2[1][0] = dM_dpprz[0] / eff_sched_var.Iyy;   // pitch effectiveness front motor

  // Update back motor q effectiveness
  g1g2[1][2] = - dM_dpprz[2] / eff_sched_var.Iyy;  // pitch effectiveness back motor

  // Update right motor p and q effectiveness
  g1g2[0][1] = roll_motor_p_eff_right;   // roll effectiveness right motor (no airspeed compensation)
  g1g2[1][1] = roll_motor_q_eff;    // pitch effectiveness right motor

  // Update left motor p and q effectiveness
  g1g2[0][3] = roll_motor_p_eff_left;  // roll effectiveness left motor
  g1g2[1][3] = -roll_motor_q_eff;   // pitch effectiveness left motor
}

void eff_scheduling_rotwing_update_elevator_effectiveness(void)
{
  float de = eff_sched_p.k_elevator_deflection[0] + eff_sched_p.k_elevator_deflection[1] * eff_sched_var.cmd_elevator;

  float dMyde = (eff_sched_p.k_elevator[0] * de * eff_sched_var.airspeed2 +
                 eff_sched_p.k_elevator[1] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_pusher_scaled * eff_sched_var.airspeed +
                 eff_sched_p.k_elevator[2] * eff_sched_var.airspeed2) / 10000.;

  // scale the effectiveness of the elevator down if it has a large deflection to encourage it to become flat quickly (pragmatic, not physically inpsired)
  float elevator_ineffectiveness_scaling = (50-de)/40;
  Bound(elevator_ineffectiveness_scaling, 0.5, 1.0);

  float dMydpprz = dMyde * eff_sched_p.k_elevator_deflection[1];

  // Convert moment to effectiveness
  float eff_y_elev = dMydpprz / eff_sched_var.Iyy;

  Bound(eff_y_elev, 0.00001, 0.1);

  g1g2[1][5] = eff_y_elev;
}

void eff_scheduling_rotwing_update_rudder_effectiveness(void)
{
  float dMzdr = (eff_sched_p.k_rudder[0] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_T_mean_scaled +
                 eff_sched_p.k_rudder[1] * eff_sched_var.cmd_T_mean_scaled * eff_sched_var.airspeed2 * eff_sched_var.cosr +
                 eff_sched_p.k_rudder[2] * eff_sched_var.airspeed2) / 10000.;

  // Convert moment to effectiveness

  float dMzdpprz = dMzdr * eff_sched_p.d_rudder_d_pprz;

  // Convert moment to effectiveness
  float eff_z_rudder = dMzdpprz / eff_sched_p.Izz;

  Bound(eff_z_rudder, 0.000001, 0.1);

  g1g2[2][4] = eff_z_rudder;
}

void eff_scheduling_rotwing_update_aileron_effectiveness(void)
{
  float dMxdpprz = (eff_sched_p.k_aileron * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
  float eff_x_aileron = dMxdpprz / eff_sched_var.Ixx;
  Bound(eff_x_aileron, 0, 0.005)
  g1g2[0][6] = eff_x_aileron;

  float dMydpprz = 4.0*(eff_sched_p.k_aileron * eff_sched_var.airspeed2 * eff_sched_var.sinr2 * eff_sched_var.cosr) / 1000000.;
  float eff_y_aileron = dMydpprz / eff_sched_var.Iyy;
  eff_y_aileron = bound_or_zero(eff_y_aileron, 0.00003f, 0.005f);
  g1g2[1][6] = eff_y_aileron;
}

void eff_scheduling_rotwing_update_flaperon_effectiveness(void)
{
  float dMxdpprz = (eff_sched_p.k_flaperon * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
  float eff_x_flap_aileron = dMxdpprz / eff_sched_var.Ixx;
  Bound(eff_x_flap_aileron, 0, 0.005)
  g1g2[0][7] = eff_x_flap_aileron;
}

void eff_scheduling_rotwing_update_pusher_effectiveness(void)
{
  float rpmP = eff_sched_p.k_rpm_pprz_pusher[0] + eff_sched_p.k_rpm_pprz_pusher[1] * eff_sched_var.cmd_pusher + eff_sched_p.k_rpm_pprz_pusher[2] * eff_sched_var.cmd_pusher * eff_sched_var.cmd_pusher;

  float dFxdrpmP = eff_sched_p.k_pusher[0]*rpmP + eff_sched_p.k_pusher[1] * eff_sched_var.airspeed;
  float drpmPdpprz = eff_sched_p.k_rpm_pprz_pusher[1] + 2. * eff_sched_p.k_rpm_pprz_pusher[2] * eff_sched_var.cmd_pusher;

  float eff_pusher = (dFxdrpmP * drpmPdpprz / eff_sched_p.m) / 10000.;

  Bound(eff_pusher, 0.00030, 0.0015);
  g1g2[4][8] = eff_pusher;
}

float eff_scheduling_rotwing_lift_d = 0.0f;

void eff_scheduling_rotwing_schedule_liftd(void)
{
  float lift_d_wing = (eff_sched_p.k_lift_wing[0] + eff_sched_p.k_lift_wing[1] * eff_sched_var.sinr2) * eff_sched_var.airspeed2 / eff_sched_p.m;
  float lift_d_fuselage = eff_sched_p.k_lift_fuselage * eff_sched_var.airspeed2 / eff_sched_p.m;
  float lift_d_tail = eff_sched_p.k_lift_tail * eff_sched_var.airspeed2 / eff_sched_p.m;

  float lift_d = lift_d_wing + lift_d_fuselage + lift_d_tail;
  if (eff_sched_var.wing_rotation_deg < 60.) {
    lift_d = 0.0;
  }
  Bound(lift_d, -130., 0.);
  eff_scheduling_rotwing_lift_d = lift_d;
}

// Override standard LIFT_D function
float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED) {
  return eff_scheduling_rotwing_lift_d;
}

void stabilization_indi_set_wls_settings(void)
{
   // Calculate the min and max increments
    for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
      u_min_stab_indi[i] = -MAX_PPRZ * act_is_servo[i];
      u_max_stab_indi[i] = MAX_PPRZ;
      u_pref_stab_indi[i] = act_pref[i];
      if (i == 5) { // elevator
        u_pref_stab_indi[i] = actuator_state_filt_vect[i]; // Set change in prefered state to 0 for elevator
        u_min_stab_indi[i] = 0; // cmd 0 is lowest position for elevator
      }
      if (i == 7) { // flaperons
        // If an offset is used, limit the max differential command to prevent unilateral saturation.
        int32_t flap_saturation_limit = MAX_PPRZ - abs(rw_flap_offset);
        BoundAbs(flap_saturation_limit, MAX_PPRZ);
        u_min_stab_indi[i] = -flap_saturation_limit;
        u_max_stab_indi[i] = flap_saturation_limit;
      }
      if (i==8) { // pusher
        // dt (min to max) MAX_PPRZ / (dt * f) dt_min == 0.002
        Bound(eff_sched_pusher_time, 0.002, 5.);
        float max_increment = MAX_PPRZ / (eff_sched_pusher_time * 500);
        wls_stab_p.u_min[i] = actuators_pprz[i] - max_increment;
        wls_stab_p.u_max[i] = actuators_pprz[i] + max_increment;

        Bound(wls_stab_p.u_min[i], 0, MAX_PPRZ);
        Bound(wls_stab_p.u_max[i], 0, MAX_PPRZ);
      }
  }
}

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

#include "modules/ctrl/eff_scheduling_rotwing_rpm_ctrl.h"

#include "generated/airframe.h"
#include "state.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "autopilot.h"

#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "modules/core/commands.h"

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

#ifndef ROTWING_EFF_SCHED_K_ELEVATOR_DEFLECTION
#error "NO ROTWING_EFF_SCHED_K_ELEVATOR_DEFLECTION defined"
#endif

#ifndef ROTWING_EFF_SCHED_D_RUDDER_D_PPRZ
#error "NO ROTWING_EFF_SCHED_D_RUDDER_D_PPRZ defined"
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

#ifndef ROTWING_EFF_SCHED_CT_HOVER
#error "NO ROTWING_EFF_SCHED_CT_HOVER defined"
#endif

#ifndef ROTWING_EFF_SCHED_CT_PUSHER
#error "NO ROTWING_EFF_SCHED_CT_PUSHER defined"
#endif

#ifndef ROTWING_EFF_SCHED_ROLL_ARM
#error "NO EFF_SCHEDULING_ROTWING_ROLL_ARM defined"
#endif

#ifndef ROTWING_EFF_SCHED_PITCH_ARM
#error "NO EFF_SCHEDULING_ROTWING_PITCH_ARM defined"
#endif

#ifndef ROTWING_EFF_SCHED_ROLL_ARM_PITCH_MOTORS
#error "NO ROTWING_EFF_SCHED_ROLL_ARM_PITCH_MOTORS defined"
#endif

#ifndef ROTWING_EFF_SCHED_HOVER_MAX_RPM
#error "NO ROTWING_EFF_SCHED_HOVER_MAX_RPM defined"
#endif

#ifndef ROTWING_EFF_SCHED_PUSHER_MAX_RPM
#error "NO ROTWING_EFF_SCHED_PUSHER_MAX_RPM defined"
#endif

#ifndef ROTWING_EFF_SCHED_RPM_CTRL_HOVER_LIM_COEF
#error "USING RPM CONTROL BUT NO ROTWING_EFF_SCHED_RPM_CTRL_HOVER_LIM_COEF defined"
#endif

#ifndef ROTWING_EFF_SCHED_RPM_CTRL_PUSH_LIM_COEF
#error "USING RPM CONTROL BUT NO ROTWING_EFF_SCHED_RPM_CTRL_PUSH_LIM_COEF defined"
#endif

#ifndef ROTWING_PITCH_MOTOR_TILT_ANGLE_DEG
#define ROTWING_PITCH_MOTOR_TILT_ANGLE_DEG 10.0
#endif

struct rotwing_eff_sched_param_t eff_sched_p = {
  .Ixx_body                 = ROTWING_EFF_SCHED_IXX_BODY,
  .Iyy_body                 = ROTWING_EFF_SCHED_IYY_BODY,
  .Izz                      = ROTWING_EFF_SCHED_IZZ,
  .Ixx_wing                 = ROTWING_EFF_SCHED_IXX_WING,
  .Iyy_wing                 = ROTWING_EFF_SCHED_IYY_WING,
  .m                        = ROTWING_EFF_SCHED_M,
  .CT_hover                 = ROTWING_EFF_SCHED_CT_HOVER,
  .CT_pusher                = ROTWING_EFF_SCHED_CT_PUSHER,
  .hover_max_rpm_squared    = ROTWING_EFF_SCHED_HOVER_MAX_RPM * ROTWING_EFF_SCHED_HOVER_MAX_RPM,
  .pusher_max_rpm_squared   = ROTWING_EFF_SCHED_PUSHER_MAX_RPM * ROTWING_EFF_SCHED_PUSHER_MAX_RPM,
  .k_elevator               = ROTWING_EFF_SCHED_K_ELEVATOR,
  .k_rudder                 = ROTWING_EFF_SCHED_K_RUDDER,
  .k_aileron                = ROTWING_EFF_SCHED_K_AILERON,
  .k_flaperon               = ROTWING_EFF_SCHED_K_FLAPERON,
  .k_elevator_deflection    = ROTWING_EFF_SCHED_K_ELEVATOR_DEFLECTION,
  .d_rudder_d_pprz          = ROTWING_EFF_SCHED_D_RUDDER_D_PPRZ,
  .k_lift_wing              = ROTWING_EFF_SCHED_K_LIFT_WING,
  .k_lift_fuselage          = ROTWING_EFF_SCHED_K_LIFT_FUSELAGE,
  .k_lift_tail              = ROTWING_EFF_SCHED_K_LIFT_TAIL,
  .hover_rpm_lim_coef       = ROTWING_EFF_SCHED_RPM_CTRL_HOVER_LIM_COEF,
  .pusher_rpm_lim_coef      = ROTWING_EFF_SCHED_RPM_CTRL_PUSH_LIM_COEF
};

int32_t rw_flap_offset = 0;
float eff_scheduling_rotwing_lift_d = 0.0f;

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
float roll_eff_slider = 12.f;

static const float Wu_gih_original[GUIDANCE_INDI_HYBRID_U] = GUIDANCE_INDI_WLS_WU;

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
inline void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);


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
  // freq = 500.0 Hz
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
  // Scaling so CAN_cmd squared fits into MAX_PPRZ
  float scaling = eff_sched_p.hover_max_rpm_squared / MAX_PPRZ;

  // Calculate the change in moment per change in RPM squared
  float dMdu_pitch = ROTWING_EFF_SCHED_PITCH_ARM * eff_sched_p.CT_hover * scaling;
  float dMdu_roll = ROTWING_EFF_SCHED_ROLL_ARM * eff_sched_p.CT_hover * scaling;
  
  float front_motor_pitch_eff = dMdu_pitch / eff_sched_var.Iyy;
  float back_motor_pitch_eff  = -dMdu_pitch / eff_sched_var.Iyy;

  // pitch motor has roll effectiveness due to z offset from c.g. and tilt
  static const float roll_eff_pitch_motor_scaling = ROTWING_EFF_SCHED_ROLL_ARM_PITCH_MOTORS / ROTWING_EFF_SCHED_ROLL_ARM * sinf(RadOfDeg(ROTWING_PITCH_MOTOR_TILT_ANGLE_DEG)); 

  // Update front motor roll and pitch effectiveness
  g1g2[RW_aq][COMMAND_MOTOR_FRONT] = front_motor_pitch_eff;   // pitch effectiveness front motor
  g1g2[RW_ap][COMMAND_MOTOR_FRONT] = dMdu_pitch * roll_eff_pitch_motor_scaling / eff_sched_var.Ixx; // Roll effectiveness front motor
  
  // Update back motor roll and pitch effectiveness
  g1g2[RW_aq][COMMAND_MOTOR_BACK] = back_motor_pitch_eff; // pitch effectiveness back motor
  g1g2[RW_ap][COMMAND_MOTOR_BACK] = -dMdu_pitch * roll_eff_pitch_motor_scaling / eff_sched_var.Ixx; // Roll effectiveness back motor

  // Update right motor roll and pitch effectiveness
  float right_motor_roll_eff = -(dMdu_roll * eff_sched_var.cosr) / eff_sched_var.Ixx;
  right_motor_roll_eff = bound_or_zero(right_motor_roll_eff, -1.f, -0.00001f);
  
  float right_motor_pitch_eff = (dMdu_roll * eff_sched_var.sinr) / eff_sched_var.Iyy;
  right_motor_pitch_eff = bound_or_zero(right_motor_pitch_eff, 0.00001f, front_motor_pitch_eff);
  
  g1g2[RW_ap][COMMAND_MOTOR_RIGHT] = right_motor_roll_eff; // roll effectiveness right motor
  g1g2[RW_aq][COMMAND_MOTOR_RIGHT] = right_motor_pitch_eff; // pitch effectiveness right motor

  // Update left motor roll and pitch effectiveness
  float left_motor_roll_eff = (dMdu_roll * eff_sched_var.cosr) / eff_sched_var.Ixx;
  left_motor_roll_eff = bound_or_zero(left_motor_roll_eff, 0.00001f, 1.f);
    
  float left_motor_pitch_eff = -(dMdu_roll * eff_sched_var.sinr) / eff_sched_var.Iyy;
  left_motor_pitch_eff = bound_or_zero(left_motor_pitch_eff, back_motor_pitch_eff, -0.00001f);

  g1g2[RW_ap][COMMAND_MOTOR_LEFT] = left_motor_roll_eff;  // roll effectiveness left motor
  g1g2[RW_aq][COMMAND_MOTOR_LEFT] = left_motor_pitch_eff;   // pitch effectiveness left motor
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

  g1g2[RW_aq][COMMAND_ELEVATOR] = eff_y_elev;
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

  g1g2[RW_ar][COMMAND_RUDDER] = eff_z_rudder;
}

void eff_scheduling_rotwing_update_aileron_effectiveness(void)
{
  float dMxdpprz = (eff_sched_p.k_aileron * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
  float eff_x_aileron = dMxdpprz / eff_sched_var.Ixx;
  Bound(eff_x_aileron, 0, 0.005);
  g1g2[RW_ap][COMMAND_AILERONS] = eff_x_aileron;

  float dMydpprz = 4.0*(eff_sched_p.k_aileron * eff_sched_var.airspeed2 * eff_sched_var.sinr2 * eff_sched_var.cosr) / 1000000.;
  float eff_y_aileron = dMydpprz / eff_sched_var.Iyy;
  eff_y_aileron = bound_or_zero(eff_y_aileron, 0.00003f, 0.005f);
  g1g2[RW_aq][COMMAND_AILERONS] = eff_y_aileron;
}

void eff_scheduling_rotwing_update_flaperon_effectiveness(void)
{
  float dMxdpprz = (eff_sched_p.k_flaperon * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
  float eff_x_flap_aileron = dMxdpprz / eff_sched_var.Ixx;
  Bound(eff_x_flap_aileron, 0, 0.005);
  g1g2[RW_ap][COMMAND_FLAPS] = eff_x_flap_aileron;
}

void eff_scheduling_rotwing_update_pusher_effectiveness(void)
{
  // Scaling so CAN_cmd squared fits into MAX_PPRZ
  float scaling = eff_sched_p.pusher_max_rpm_squared / MAX_PPRZ;
  float drpmPdpprz = eff_sched_p.CT_pusher * scaling;
  float eff_pusher = (drpmPdpprz / eff_sched_p.m);

  // Bound effectiveness (shouldn't be necessary with the new structure but just in case)
  Bound(eff_pusher, 0.00030, 0.0015);

  g1g2[RW_aX][COMMAND_THRUST_X] = eff_pusher;
}

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
  int32_t flap_saturation_limit;

  // Calculate the min and max increments
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    wls_stab_p.u_min[i] = -MAX_PPRZ * act_is_servo[i];
    wls_stab_p.u_max[i] = MAX_PPRZ;
    wls_stab_p.u_pref[i] = act_pref[i];

    switch(i) {
      // With RPM control we need to limit the max command to the hover motors based on the battery voltage
      case COMMAND_MOTOR_FRONT:
        wls_stab_p.u_max[i] = eff_sched_p.hover_rpm_lim_coef[0] + electrical.vsupply * eff_sched_p.hover_rpm_lim_coef[1];
        wls_stab_p.u_min[i] = (int16_t)((float)SERVO_MOTOR_FRONT_NEUTRAL * ((float)SERVO_MOTOR_FRONT_NEUTRAL * ((float)MAX_PPRZ / (float)SERVO_MOTOR_FRONT_MAX) / (float)SERVO_MOTOR_FRONT_MAX));
        break;
      case COMMAND_MOTOR_RIGHT:
        wls_stab_p.u_max[i] = eff_sched_p.hover_rpm_lim_coef[0] + electrical.vsupply * eff_sched_p.hover_rpm_lim_coef[1];
        wls_stab_p.u_min[i] = (int16_t)((float)SERVO_MOTOR_RIGHT_NEUTRAL * ((float)SERVO_MOTOR_RIGHT_NEUTRAL * ((float)MAX_PPRZ / (float)SERVO_MOTOR_RIGHT_MAX) / (float)SERVO_MOTOR_RIGHT_MAX));
        break;
      case COMMAND_MOTOR_BACK:
        wls_stab_p.u_max[i] = eff_sched_p.hover_rpm_lim_coef[0] + electrical.vsupply * eff_sched_p.hover_rpm_lim_coef[1];
        wls_stab_p.u_min[i] = (int16_t)((float)SERVO_MOTOR_BACK_NEUTRAL * ((float)SERVO_MOTOR_BACK_NEUTRAL * ((float)MAX_PPRZ / (float)SERVO_MOTOR_BACK_MAX) / (float)SERVO_MOTOR_BACK_MAX));
        break;
      case COMMAND_MOTOR_LEFT:
        wls_stab_p.u_max[i] = eff_sched_p.hover_rpm_lim_coef[0] + electrical.vsupply * eff_sched_p.hover_rpm_lim_coef[1];
        wls_stab_p.u_min[i] = (int16_t)((float)SERVO_MOTOR_LEFT_NEUTRAL * ((float)SERVO_MOTOR_LEFT_NEUTRAL * ((float)MAX_PPRZ / (float)SERVO_MOTOR_LEFT_MAX) / (float)SERVO_MOTOR_LEFT_MAX));
        break;
      case COMMAND_ELEVATOR:
        wls_stab_p.u_pref[i] = actuator_state_filt_vect[i]; // Set change in prefered state to 0 for elevator
        wls_stab_p.u_min[i] = 0; // cmd 0 is lowest position for elevator
        break;
      case COMMAND_FLAPS:
        // If an offset is used, limit the max differential command to prevent unilateral saturation.
        flap_saturation_limit = MAX_PPRZ - abs(rw_flap_offset);
        wls_stab_p.u_min[i] = -flap_saturation_limit;
        wls_stab_p.u_max[i] = flap_saturation_limit;
        break;
      case COMMAND_THRUST_X:
        // dt (min to max) MAX_PPRZ / (dt * f) dt_min == 0.002
        Bound(eff_sched_pusher_time, 0.002, 5.);
        float max_increment = MAX_PPRZ / (eff_sched_pusher_time * 500);
        wls_stab_p.u_min[i] = actuators_pprz[i] - max_increment;
        wls_stab_p.u_max[i] = actuators_pprz[i] + max_increment;

        // With RPM control we need to limit the max command to the pusher motor based on the battery voltage
        wls_stab_p.u_min[i] = Max((int16_t)((float)SERVO_MOTOR_PUSH_NEUTRAL * ((float)SERVO_MOTOR_PUSH_NEUTRAL * ((float)MAX_PPRZ / (float)SERVO_MOTOR_PUSH_MAX) / (float)SERVO_MOTOR_PUSH_MAX)), wls_stab_p.u_min[i]);
        wls_stab_p.u_max[i] = Min(eff_sched_p.pusher_rpm_lim_coef[0] + electrical.vsupply * eff_sched_p.pusher_rpm_lim_coef[1], wls_stab_p.u_max[i]);
        break;
      default:
        break;
    }  

    Bound(wls_stab_p.u_min[i], -MAX_PPRZ*act_is_servo[i], MAX_PPRZ);
    Bound(wls_stab_p.u_max[i], -MAX_PPRZ*act_is_servo[i], MAX_PPRZ);
  }
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  // adjust weights
  float fixed_wing_percentile = (rotwing_state_hover_motors_idling())? 1:0; // TODO: when hover props go below 40%, ...
  Bound(fixed_wing_percentile, 0, 1);
#define AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT 16

  float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;

  // Increase importance of forward acceleration in forward flight
  wls_guid_p.Wv[0] = Wv_original[0] * (1.0f + fixed_wing_percentile *
                                         AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT); // stall n low hover motor_off (weight 16x more important than vertical weight)

  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  float du_min_thrust_z = ((MAX_PPRZ - actuator_state_filt_vect[COMMAND_MOTOR_FRONT]) * g1g2[RW_aZ][COMMAND_MOTOR_FRONT]
                           + (MAX_PPRZ - actuator_state_filt_vect[COMMAND_MOTOR_RIGHT]) * g1g2[RW_aZ][COMMAND_MOTOR_RIGHT]
                           + (MAX_PPRZ - actuator_state_filt_vect[COMMAND_MOTOR_BACK]) * g1g2[RW_aZ][COMMAND_MOTOR_BACK] 
                           + (MAX_PPRZ - actuator_state_filt_vect[COMMAND_MOTOR_LEFT]) * g1g2[RW_aZ][COMMAND_MOTOR_LEFT]) * rotwing_state_hover_motors_running();
  Bound(du_min_thrust_z, -50., 0.);
  float du_max_thrust_z = -(actuator_state_filt_vect[COMMAND_MOTOR_FRONT] * g1g2[RW_aZ][COMMAND_MOTOR_FRONT] 
                            + actuator_state_filt_vect[COMMAND_MOTOR_RIGHT] * g1g2[RW_aZ][COMMAND_MOTOR_RIGHT] 
                            + actuator_state_filt_vect[COMMAND_MOTOR_BACK] * g1g2[RW_aZ][COMMAND_MOTOR_BACK] 
                            + actuator_state_filt_vect[COMMAND_MOTOR_LEFT] * g1g2[RW_aZ][COMMAND_MOTOR_LEFT]);
  Bound(du_max_thrust_z, 0., 50.);

  float roll_limit_rad = guidance_indi_max_bank;
  float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  float fwd_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float quad_pitch_limit_rad = RadOfDeg(5.0);

  float airspeed = stateGetAirspeed_f();

  float scheduled_pitch_angle = 0.f;
  float pitch_angle_range = 3.;
  float meas_skew_angle = rotwing_state.meas_skew_angle_deg;
  Bound(meas_skew_angle, 0, 90); // Bound to prevent errors
  if (meas_skew_angle < ROTWING_SKEW_ANGLE_STEP) {
    scheduled_pitch_angle = ROTWING_QUAD_PREF_PITCH;
    wls_guid_p.Wu[1] = Wu_gih_original[1];
    max_pitch_limit_rad = quad_pitch_limit_rad;
  } else {
    float pitch_progression = (airspeed - rotwing_state.fw_min_airspeed) / 2.f;
    Bound(pitch_progression, 0.f, 1.f);
    scheduled_pitch_angle = pitch_angle_range * pitch_progression + ROTWING_QUAD_PREF_PITCH*(1.f-pitch_progression);
    wls_guid_p.Wu[1] = Wu_gih_original[1] * (1.f - pitch_progression*0.99);
    max_pitch_limit_rad = quad_pitch_limit_rad + (fwd_pitch_limit_rad - quad_pitch_limit_rad) * pitch_progression;
  }
  if (!rotwing_state_hover_motors_running()) {
    scheduled_pitch_angle = 8.;
    max_pitch_limit_rad = fwd_pitch_limit_rad;
  }
  Bound(scheduled_pitch_angle, -5., 8.);
  guidance_indi_pitch_pref_deg = scheduled_pitch_angle;

  float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Set lower limits
  wls_guid_p.u_min[0] = -roll_limit_rad - roll_angle; //roll
  wls_guid_p.u_min[1] = min_pitch_limit_rad - pitch_angle; // pitch

  // Set upper limits limits
  wls_guid_p.u_max[0] = roll_limit_rad - roll_angle; //roll
  wls_guid_p.u_max[1] = max_pitch_limit_rad - pitch_angle; // pitch

  if(rotwing_state_hover_motors_running()) {
    wls_guid_p.u_min[2] = du_min_thrust_z;
    wls_guid_p.u_max[2] = du_max_thrust_z;
  } else {
    wls_guid_p.u_min[2] = 0.;
    wls_guid_p.u_max[2] = 0.;
  }

  if(rotwing_state_pusher_motor_running()) {
    wls_guid_p.u_min[3] = (-actuator_state_filt_vect[8] * g1g2[4][8]);
    wls_guid_p.u_max[3] = 9.0; // Hacky value to prevent drone from pitching down in transition
  } else {
    wls_guid_p.u_min[3] = 0.;
    wls_guid_p.u_max[3] = 0.;
  }

  // Set prefered states
  wls_guid_p.u_pref[0] = 0; // prefered delta roll angle
  wls_guid_p.u_pref[1] = -pitch_angle + pitch_pref_rad;// prefered delta pitch angle
  wls_guid_p.u_pref[2] = wls_guid_p.u_max[2]; // Low thrust better for efficiency
  wls_guid_p.u_pref[3] = body_v[0]; // solve the body acceleration
}

/**
 * Convert u^2 to PPRZ command
 * @param u_squared Value in squared domain
 * @param max Motor max value (e.g., SERVO_MOTOR_FRONT_MAX)
 * @param neutral Motor neutral value
 * @param travel_up Motor travel up value
 * @return PPRZ cmd
 */
static float u_squared_to_pprz_cmd(float u_squared, float max, float neutral, float travel_up)
{
  float u = (u_squared > 0 ? 1 : (u_squared < 0 ? -1 : 0)) * sqrtf(fabsf(u_squared) / (float)MAX_PPRZ * max * max);
  float u_unbounded = (u - neutral) / travel_up;
  return (u_unbounded < 0) ? 0 : u_unbounded;
}

/**
 * Convert PPRZ command back to u^2
 * @param pprz_cmd PPRZ command
 * @param max Motor max value (e.g., SERVO_MOTOR_FRONT_MAX)
 * @param neutral Motor neutral value
 * @param travel_up Motor travel up value
 * @return u^2
 */
static float pprz_cmd_to_u_squared(float pprz_cmd, float max, float neutral, float travel_up)
{
  float u_can_cmd = pprz_cmd * travel_up + neutral;
  return (u_can_cmd * u_can_cmd) / (max * max) * (float)MAX_PPRZ;
}

void stabilization_indi_get_actuator_state(void)
{ 
  float* indi_u = stabilization_indi_get_indi_u();
  float* actuator_state = stabilization_indi_get_act_state();
  float* act_dyn_discrete = stabilization_indi_get_act_dyn();

  float u_pprz_cmd;
  float act_pprz;

  //actuator dynamics
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    switch(i) {
      // With RPM control we have to do a conversion step from allocation back to actuator state
      case COMMAND_MOTOR_FRONT:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_FRONT_MAX, SERVO_MOTOR_FRONT_NEUTRAL, SERVO_MOTOR_FRONT_TRAVEL_UP);
        act_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_FRONT_MAX, SERVO_MOTOR_FRONT_NEUTRAL, SERVO_MOTOR_FRONT_TRAVEL_UP);
        actuator_state[i] = act_pprz + act_dyn_discrete[i] * (u_pprz_cmd - act_pprz);
        actuator_state[i] = pprz_cmd_to_u_squared(actuator_state[i], SERVO_MOTOR_FRONT_MAX, SERVO_MOTOR_FRONT_NEUTRAL, SERVO_MOTOR_FRONT_TRAVEL_UP);
        break;
      case COMMAND_MOTOR_RIGHT:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_RIGHT_MAX, SERVO_MOTOR_RIGHT_NEUTRAL, SERVO_MOTOR_RIGHT_TRAVEL_UP);
        act_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_RIGHT_MAX, SERVO_MOTOR_RIGHT_NEUTRAL, SERVO_MOTOR_RIGHT_TRAVEL_UP);
        actuator_state[i] = act_pprz + act_dyn_discrete[i] * (u_pprz_cmd - act_pprz);
        actuator_state[i] = pprz_cmd_to_u_squared(actuator_state[i], SERVO_MOTOR_RIGHT_MAX, SERVO_MOTOR_RIGHT_NEUTRAL, SERVO_MOTOR_RIGHT_TRAVEL_UP);
        break;
      case COMMAND_MOTOR_BACK:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_BACK_MAX, SERVO_MOTOR_BACK_NEUTRAL, SERVO_MOTOR_BACK_TRAVEL_UP);
        act_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_BACK_MAX, SERVO_MOTOR_BACK_NEUTRAL, SERVO_MOTOR_BACK_TRAVEL_UP);
        actuator_state[i] = act_pprz + act_dyn_discrete[i] * (u_pprz_cmd - act_pprz);
        actuator_state[i] = pprz_cmd_to_u_squared(actuator_state[i], SERVO_MOTOR_BACK_MAX, SERVO_MOTOR_BACK_NEUTRAL, SERVO_MOTOR_BACK_TRAVEL_UP);
        break;
      case COMMAND_MOTOR_LEFT:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_LEFT_MAX, SERVO_MOTOR_LEFT_NEUTRAL, SERVO_MOTOR_LEFT_TRAVEL_UP);
        act_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_LEFT_MAX, SERVO_MOTOR_LEFT_NEUTRAL, SERVO_MOTOR_LEFT_TRAVEL_UP);
        actuator_state[i] = act_pprz + act_dyn_discrete[i] * (u_pprz_cmd - act_pprz);
        actuator_state[i] = pprz_cmd_to_u_squared(actuator_state[i], SERVO_MOTOR_LEFT_MAX, SERVO_MOTOR_LEFT_NEUTRAL, SERVO_MOTOR_LEFT_TRAVEL_UP);
        break;
      case COMMAND_THRUST_X:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_PUSH_MAX, SERVO_MOTOR_PUSH_NEUTRAL, SERVO_MOTOR_PUSH_TRAVEL_UP);
        act_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_PUSH_MAX, SERVO_MOTOR_PUSH_NEUTRAL, SERVO_MOTOR_PUSH_TRAVEL_UP);
        actuator_state[i] = act_pprz + act_dyn_discrete[i] * (u_pprz_cmd - act_pprz);
        actuator_state[i] = pprz_cmd_to_u_squared(actuator_state[i], SERVO_MOTOR_PUSH_MAX, SERVO_MOTOR_PUSH_NEUTRAL, SERVO_MOTOR_PUSH_TRAVEL_UP);
        break;
      default:
        actuator_state[i] = actuator_state[i] + act_dyn_discrete[i] * (indi_u[i] - actuator_state[i]);
        break;
    }
  }
}

void stabilization_indi_commit_actuator_cmd(int32_t *cmd)
{
  float* indi_u = stabilization_indi_get_indi_u();
  float* actuator_state = stabilization_indi_get_act_state();
  bool* act_is_thruster_z = stabilization_indi_get_act_is_thruster_z();
  uint8_t* act_to_commands = stabilization_indi_get_act_to_commands();

  float u_pprz_cmd;
  float act_state_pprz;

  cmd[COMMAND_THRUST] = 0;

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    switch(i) {
      // With RPM control we have to do a conversion step from allocation back to actuator state
      case COMMAND_MOTOR_FRONT:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_FRONT_MAX, SERVO_MOTOR_FRONT_NEUTRAL, SERVO_MOTOR_FRONT_TRAVEL_UP);
        act_state_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_FRONT_MAX, SERVO_MOTOR_FRONT_NEUTRAL, SERVO_MOTOR_FRONT_TRAVEL_UP);
        break;
      case COMMAND_MOTOR_RIGHT:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_RIGHT_MAX, SERVO_MOTOR_RIGHT_NEUTRAL, SERVO_MOTOR_RIGHT_TRAVEL_UP);
        act_state_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_RIGHT_MAX, SERVO_MOTOR_RIGHT_NEUTRAL, SERVO_MOTOR_RIGHT_TRAVEL_UP);
        break;
      case COMMAND_MOTOR_BACK:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_BACK_MAX, SERVO_MOTOR_BACK_NEUTRAL, SERVO_MOTOR_BACK_TRAVEL_UP);
        act_state_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_BACK_MAX, SERVO_MOTOR_BACK_NEUTRAL, SERVO_MOTOR_BACK_TRAVEL_UP);
        break;
      case COMMAND_MOTOR_LEFT:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_LEFT_MAX, SERVO_MOTOR_LEFT_NEUTRAL, SERVO_MOTOR_LEFT_TRAVEL_UP);
        act_state_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_LEFT_MAX, SERVO_MOTOR_LEFT_NEUTRAL, SERVO_MOTOR_LEFT_TRAVEL_UP);
        break;
      case COMMAND_THRUST_X:
        u_pprz_cmd = u_squared_to_pprz_cmd(indi_u[i], SERVO_MOTOR_PUSH_MAX, SERVO_MOTOR_PUSH_NEUTRAL, SERVO_MOTOR_PUSH_TRAVEL_UP);
        act_state_pprz = u_squared_to_pprz_cmd(actuator_state[i], SERVO_MOTOR_PUSH_MAX, SERVO_MOTOR_PUSH_NEUTRAL, SERVO_MOTOR_PUSH_TRAVEL_UP);
        break;
      default:
        u_pprz_cmd = indi_u[i];
        act_state_pprz = actuator_state[i];
        break;
    }

    actuators_pprz[i] = u_pprz_cmd;

    if (act_to_commands != NULL) {
      cmd[act_to_commands[i]] = actuators_pprz[i];
    }

    cmd[COMMAND_THRUST] += act_state_pprz * (int32_t) act_is_thruster_z[i];
  }
  cmd[COMMAND_THRUST] /= stabilization_indi_get_num_thrusters();
}
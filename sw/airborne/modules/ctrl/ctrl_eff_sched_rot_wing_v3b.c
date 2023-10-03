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

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing_v3b.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#include "modules/ctrl/ctrl_eff_sched_rot_wing_v3b.h"

#include "modules/rot_wing_drone/wing_rotation_controller_v3b.h"

#include "modules/actuators/actuators.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"


#include "state.h"
#include "filters/low_pass_filter.h"

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };
float rot_wing_side_motors_g1_p_0[2];
float rot_wing_side_motors_g1_q_90[2] = ROT_WING_SCHED_G1_Q_90; 

// Define polynomial constants for effectiveness values for aerodynamic surfaces
#ifndef ROT_WING_SCHED_G1_AERO_CONST_P
#error "ROT_WING_SCHED_AERO_CONST_P should be defined"
#endif

#ifndef ROT_WING_SCHED_G1_AERO_CONST_Q
#error "ROT_WING_SCHED_AERO_CONST_Q should be defined"
#endif

#ifndef ROT_WING_SCHED_G1_AERO_CONST_R
#error "ROT_WING_SCHED_AERO_CONST_R should be defined"
#endif

float rot_wing_aerodynamic_eff_const_g1_p[1] = ROT_WING_SCHED_G1_AERO_CONST_P;
float rot_wing_aerodynamic_eff_const_g1_q[1] = ROT_WING_SCHED_G1_AERO_CONST_Q;
float rot_wing_aerodynamic_eff_const_g1_r[1] = ROT_WING_SCHED_G1_AERO_CONST_R;

// Define settings to multiply initial control eff scheduling values
float lift_d_multiplier = 0.7;
float g1_p_multiplier = 1.;
float g1_q_multiplier = 1.;
float g1_r_multiplier = 1.;
float g1_t_multiplier = 1.;

bool wing_rotation_sched_activated = true;
bool pusher_sched_activated = true;

float sched_pitch_hover_deg = 0.;
float sched_pitch_forward_deg = 10.;
float sched_lower_hover_speed = 7.;
float sched_upper_hover_speed = 14.;

float pitch_angle_set = 0;
float pitch_angle_range = 3.;

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 0.1
#endif

Butterworth2LowPass airspeed_lowpass_filter;

// Define scheduling constants
const float k_elevator[3] = {0.4603,  -4.81466, -28.8464};
const float k_rudder[3] = {-26.1434, -0.336403, -1.16702 };
const float k_pusher[2] = {0.007777, -0.67521};

float I_xx = 0.1184;
float I_yy = 1.1552;
float I_zz = 1.18;
const float weight_sched = 6.94;

// Some moved outer loop settings. TODO: make struct
float pitch_priority_factor = 11.;
float roll_priority_factor = 10.;
float thrust_priority_factor = 7.;
float pusher_priority_factor = 30.;

bool hover_motors_active = true;
bool bool_disable_hover_motors = false;

float horizontal_accel_weight = 10.;
float vertical_accel_weight = 10.;

inline void update_inertia(float *cosr2, float *sinr2);
inline void update_hover_motor_effectiveness(float *sk, float *cosr, float *sinr, float *airspeed_f);
inline void update_elevator_effectiveness(int16_t *elev_pprz, float *airspeed, float *airspeed2, float *pp_scaled);
inline void update_rudder_effectiveness(float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *cosr);
//inline void update_left_aileron_effectiveness(float *airspeed2, float *sinr);
inline void update_aileron_effectiveness(float *airspeed2, float *sinr);
inline void update_flap_aileron_effectiveness(float *airspeed2, float *sinr);
inline void update_pusher_effectiveness(float *airspeed_f, float pusher_cmd_filt);
inline void schedule_pref_pitch_angle_deg(float wing_rot_deg);
inline void schedule_pitch_priority_factor(float wing_rot_deg);
inline void schedule_liftd(float *airspeed2, float *sinr, float wing_rot_deg);
inline void schedule_wls_guidance(void);


void init_eff_scheduling(void)
{
  // Copy initial effectiveness on roll for side motors
  rot_wing_side_motors_g1_p_0[0] = g1_startup[0][1];
  rot_wing_side_motors_g1_p_0[1] = g1_startup[0][3];

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  init_butterworth_2_low_pass(&airspeed_lowpass_filter, tau, sample_time, 0.0);
}

void event_eff_scheduling(void)
{
  // Update relevant states
  // Update airspeed
  float airspeed = stateGetAirspeed_f();
  update_butterworth_2_low_pass(&airspeed_lowpass_filter, airspeed);
  float airspeed2 = airspeed * airspeed;//airspeed_lowpass_filter.o[0] * airspeed_lowpass_filter.o[0];

  // Update skew angle and triogeometric variables of wing rotation
  float cosr;
  float sinr;
  float cosr2;
  float sinr2;
  float wing_rotation_deg = wing_rotation.wing_angle_deg;
  if (wing_rotation_sched_activated) {
    cosr = cosf(wing_rotation.wing_angle_rad);
    sinr = sinf(wing_rotation.wing_angle_rad);
    cosr2 = cosr * cosr;
    sinr2 = sinr * sinr;
  } else {
    cosr = 1;
    cosr2 = 1;
    sinr = 0;
    sinr2 = 0;
  }

  // Update actuator states
  int16_t *elev_pprz = &actuators_pprz[5];

  float pp_scaled = actuators_pprz[8] / MAX_PPRZ * 8191. / 1000.;
  float T_mean_scaled = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3]) / 4. / MAX_PPRZ * 8191. / 1000.;

  // Calculate deflection of elevator
  
  update_inertia(&cosr2, &sinr2);
  update_hover_motor_effectiveness(&wing_rotation.wing_angle_rad, &cosr, &sinr, &airspeed);
  update_rudder_effectiveness(&airspeed2, &pp_scaled, &T_mean_scaled, &cosr);
  update_elevator_effectiveness(elev_pprz, &airspeed, &airspeed2, &pp_scaled);
  // update_left_aileron_effectiveness(&airspeed2, &sinr);
  update_aileron_effectiveness(&airspeed2, &sinr);
  update_flap_aileron_effectiveness(&airspeed2, &sinr);
  update_pusher_effectiveness(&airspeed, actuator_state_filt_vect[8]);
  schedule_pref_pitch_angle_deg(wing_rotation_deg);
  schedule_liftd(&airspeed2, &sinr2, wing_rotation_deg);
  schedule_wls_guidance();

  // float g1_p_side_motors[2];
  // float g1_q_side_motors[2];

  // // Calculate roll and pitch effectiveness of the two roll side motors
  // g1_p_side_motors[0] = rot_wing_side_motors_g1_p_0[0] * cosr;
  // g1_p_side_motors[1] = rot_wing_side_motors_g1_p_0[1] * cosr;

  // g1_q_side_motors[0] = rot_wing_side_motors_g1_q_90[0] * sinr;
  // g1_q_side_motors[1] = rot_wing_side_motors_g1_q_90[1] * sinr;

  // // Update inner loop effectiveness matrix for motors
  // g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  // g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][1] = g1_p_side_motors[0] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][1] = g1_q_side_motors[0] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  // g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  // g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][3] = g1_p_side_motors[1] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][3] = g1_q_side_motors[1] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  // g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][4] = rot_wing_aerodynamic_eff_const_g1_p[0] * airspeed2 / INDI_G_SCALING;

  // g1g2[1][4] = rot_wing_aerodynamic_eff_const_g1_q[0] * airspeed2 / INDI_G_SCALING;

  // g1g2[2][4] = rot_wing_aerodynamic_eff_const_g1_r[0] * airspeed2 / INDI_G_SCALING;

}

void update_inertia(float *cosr2, float *sinr2)
{
  // Inertia with wing
  I_xx = 0.128887470778173 * *sinr2 - 0.00522080411150638 * *cosr2 + 0.123666666666667;
  I_yy = 0.299513844419477 * *sinr2 + 0.427819488913857 * *cosr2 + 0.727333333333334;
  Bound(I_xx, 0.0001, 100);
  Bound(I_yy, 0.0001, 100);
}

void update_hover_motor_effectiveness(float *sk, float *cosr, float *sinr, float *airspeed_f)
{
  float bounded_airspeed = *airspeed_f;
  Bound(bounded_airspeed, 0, 20);

  // Update inner loop effectiveness matrix for motors
  g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][1] = ((0.0000947144886298017 * *sk * *sk * bounded_airspeed * *cosr - 0.001684595 * *cosr) / I_xx) * g1_p_multiplier;
  Bound(g1g2[0][1], -1, -0.00001);
  g1g2[1][1] = ((-0.000732843248849 * *sk * *sk * *sinr + 0.001600837 * *sk) / I_yy) * g1_q_multiplier;
  Bound(g1g2[1][1], 0, 1);
  g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][3] = -((0.0000947144886298017 * *sk * *sk * bounded_airspeed * *cosr - 0.001684595 * *cosr + 0.000053954 * bounded_airspeed * *cosr) / I_xx) * g1_p_multiplier;
  Bound(g1g2[0][3], 0.00001, 1);
  g1g2[1][3] = -((-0.000732843248849 * *sk * *sk * *sinr + 0.001600837 * *sk) / I_yy) * g1_q_multiplier;
  Bound(g1g2[1][3], -1, 0);
  g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;
}

void update_elevator_effectiveness(int16_t *elev_pprz, float *airspeed, float *airspeed2, float *pp_scaled)
{
  // Calculate deflection angle in [deg]
  float de = -0.0063 * *elev_pprz + 50.0;
  float bounded_airspeed = *airspeed;
  float bounded_airspeed2 = *airspeed2; 
  Bound(bounded_airspeed, 0. ,20.);
  Bound(bounded_airspeed2, 0., 400.);

  float dMyde = (k_elevator[0] * de * bounded_airspeed2 +
                k_elevator[1] * *pp_scaled * *pp_scaled * bounded_airspeed + 
                k_elevator[2] * bounded_airspeed2) / 10000.;

  float dMydpprz = dMyde * -0.0063;
  
  // Convert moment to effectiveness
  float eff_y_elev = dMydpprz / I_yy;

  Bound(eff_y_elev, 0.00001, 0.1);

  g1g2[0][5] = 0;
  g1g2[1][5] = eff_y_elev;
  g1g2[2][5] = 0;
  g1g2[3][5] = 0;
  
}

void update_rudder_effectiveness(float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *cosr)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float dMzdr = (k_rudder[0] * *pp_scaled * *T_mean_scaled + 
                k_rudder[1] * *T_mean_scaled * bounded_airspeed2 * *cosr + 
                k_rudder[2] * bounded_airspeed2) / 10000.;

  // Convert moment to effectiveness

  float dMzdpprz = dMzdr * -0.0018;

  // Convert moment to effectiveness
  float eff_z_rudder = dMzdpprz / I_zz;

  Bound(eff_z_rudder, 0.00001, 0.1);

  g1g2[0][4] = 0 / INDI_G_SCALING;
  g1g2[1][4] = 0 / INDI_G_SCALING;
  g1g2[2][4] = eff_z_rudder;
  g1g2[3][4] = 0 / INDI_G_SCALING;
}

// void update_left_aileron_effectiveness(float *airspeed2, float *sinr)
// {
//   float bounded_airspeed2 = *airspeed2;
//   Bound(bounded_airspeed2, 0., 400.);
//   float dMxdpprz = 2.620421875e-6 * 0.53 * bounded_airspeed2 * *sinr * *sinr * *sinr;
//   float eff_x_left_aileron = dMxdpprz / I_xx;
//   Bound(eff_x_left_aileron, 0, 0.005);
//   g1g2[0][6] = eff_x_left_aileron;
// }

void update_aileron_effectiveness(float *airspeed2, float *sinr)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float dMxdpprz = 2.620421875e-6 * 1.06 * bounded_airspeed2 * *sinr * *sinr * *sinr;
  float eff_x_aileron = dMxdpprz / I_xx;
  Bound(eff_x_aileron, 0, 0.005)
  g1g2[0][6] = eff_x_aileron;
}

void update_flap_aileron_effectiveness(float *airspeed2, float *sinr)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float dMxdpprz = 2.620421875e-6 * 0.78 * bounded_airspeed2 * *sinr * *sinr * *sinr;
  float eff_x_flap_aileron = dMxdpprz / I_xx;
  Bound(eff_x_flap_aileron, 0, 0.005)
  g1g2[0][7] = eff_x_flap_aileron;
}

void update_pusher_effectiveness(float *airspeed_f, float pusher_cmd_filt)
{
  if (pusher_sched_activated)
  {
    float bounded_airspeed = *airspeed_f;
    Bound(bounded_airspeed, 0., 20.);

    float rpmP = -2.91178067445214e-5*pusher_cmd_filt*pusher_cmd_filt + 1.32098226269777*pusher_cmd_filt - 131.497033952591;
    float dFxdrpmP = k_pusher[0]*rpmP + k_pusher[1]* bounded_airspeed;
    float drpmPdpprz = 1.32098226269777 - 5.82356134890428e-5*pusher_cmd_filt;

    float eff_pusher = (dFxdrpmP * drpmPdpprz / weight_sched) / 10000.;

  Bound(eff_pusher, 0.00030, 0.0015);
  g1g2[4][8] = eff_pusher;
  } else {
    g1g2[4][8] = STABILIZATION_INDI_PUSHER_PROP_EFFECTIVENESS;
  }
}

void schedule_pref_pitch_angle_deg(float wing_rot_deg)
{
  float scheduled_pitch_angle = 0;
  if (wing_rot_deg < 55) {
    scheduled_pitch_angle = pitch_angle_set;
  } else {
    float pitch_progression = (wing_rot_deg - 55) / 35.;
    scheduled_pitch_angle = pitch_angle_range * pitch_progression;
  }
  Bound(scheduled_pitch_angle, -5., 7.);
  guidance_indi_pitch_pref_deg = scheduled_pitch_angle;
}

void schedule_pitch_priority_factor(float wing_rot_deg)
{
  float scheduled_pitch_priority;
  float hover_weight = 60;
  float forward_weight = 11;
  float pitch_priority_range = hover_weight - forward_weight;
  float transition_percentage = wing_rot_deg / 90.;
  Bound(transition_percentage, 0. ,1.);

  scheduled_pitch_priority = hover_weight - transition_percentage * pitch_priority_range;
  
  Bound(scheduled_pitch_priority, forward_weight, hover_weight);
  pitch_priority_factor = scheduled_pitch_priority;
}

float ctrl_eff_sched_rot_wing_lift_d = 0.0f;

void schedule_liftd(float *airspeed2, float *sinr2, float wing_rot_deg)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float lift_d_wing = (-0.74529194103945 * bounded_airspeed2 * *sinr2 - 0.4065513216373 * bounded_airspeed2) / weight_sched * 1.18;
  float lift_d_fuselage = -0.072362752875 * bounded_airspeed2 / weight_sched;
  float lift_d_tail = -0.1452739306305 * bounded_airspeed2 / weight_sched;

  float lift_d = (lift_d_wing + lift_d_fuselage + lift_d_tail) * lift_d_multiplier;
  if (wing_rot_deg < 60) {
    lift_d = 0.0;
  }
  Bound(lift_d, -130., 0.);
  ctrl_eff_sched_rot_wing_lift_d = lift_d;
}

// Override standard LIFT_D function
float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED) {
  return ctrl_eff_sched_rot_wing_lift_d;
}


void schedule_wls_guidance(void)
{
  // Set weights
  Wu_gih[0] = roll_priority_factor * 10.414;
  Wu_gih[1] = pitch_priority_factor * 27.53;
  Wu_gih[2] = thrust_priority_factor * 0.626;
  Wu_gih[3] = pusher_priority_factor * 1.0;

  // adjust weights
  float thrust_command = (actuator_state_filt_vect[0] + actuator_state_filt_vect[1] + actuator_state_filt_vect[2] + actuator_state_filt_vect[3]) / 4;
  Bound(thrust_command, 0, MAX_PPRZ);
  float fixed_wing_percentage = !hover_motors_active; // TODO: when hover props go below 40%, ...
  Bound(fixed_wing_percentage, 0, 1);
  #define AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT 16

  Wv_gih[0] = horizontal_accel_weight * (1.0f + fixed_wing_percentage * AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT); // stall n low hover motor_off (weight 16x more important than vertical weight)
  Wv_gih[1] = horizontal_accel_weight;
  Wv_gih[2] = vertical_accel_weight;
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

  float min_pprz_cmd_ail = -MAX_PPRZ;
  if (wing_rotation.wing_angle_deg < 15) {
    min_pprz_cmd_ail = 0;
  }
  Bound(min_pprz_cmd_ail, -9600, 0);

  du_min_stab_indi[6] = min_pprz_cmd_ail - use_increment*actuators_pprz[6];

  float min_pprz_cmd_flap_ail = -MAX_PPRZ;
  if (wing_rotation.wing_angle_deg < 38) {
    min_pprz_cmd_flap_ail = -1000;
  } if (wing_rotation.wing_angle_deg > 50) {
    min_pprz_cmd_flap_ail = -9600;
  } else {
    min_pprz_cmd_flap_ail = -5.596578906693223 * wing_rotation.wing_angle_deg * wing_rotation.wing_angle_deg * wing_rotation.wing_angle_deg + 654.186408367317 * wing_rotation.wing_angle_deg * wing_rotation.wing_angle_deg - 25577.0135504177 * wing_rotation.wing_angle_deg + 333307.855118805;
  }

  Bound(min_pprz_cmd_flap_ail, -9600, 0);

  du_min_stab_indi[7] = min_pprz_cmd_flap_ail - use_increment*actuators_pprz[7];
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  // Evaluate motors_on boolean
  if (!hover_motors_active) {
    if (stateGetAirspeed_f() < 15.) {
      hover_motors_active = true;
      bool_disable_hover_motors = false;
    } else if (eulers_zxy.theta > RadOfDeg(15.0)) {
      hover_motors_active = true;
      bool_disable_hover_motors = false;
    }
  } else {
    bool_disable_hover_motors = false;
  }

  float du_min_thrust_z = ((MAX_PPRZ - actuator_state_filt_vect[0]) * g1g2[3][0] + (MAX_PPRZ - actuator_state_filt_vect[1]) * g1g2[3][1] + (MAX_PPRZ - actuator_state_filt_vect[2]) * g1g2[3][2] + (MAX_PPRZ - actuator_state_filt_vect[3]) * g1g2[3][3]) * hover_motors_active;
  Bound(du_min_thrust_z, -50., 0.);
  float du_max_thrust_z = -(actuator_state_filt_vect[0]*g1g2[3][0] + actuator_state_filt_vect[1]*g1g2[3][1] + actuator_state_filt_vect[2]*g1g2[3][2] + actuator_state_filt_vect[3]*g1g2[3][3]);
  Bound(du_max_thrust_z, 0., 50.);

  float roll_limit_rad = GUIDANCE_H_MAX_BANK;
  float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Set lower limits
  du_min_gih[0] = -roll_limit_rad - roll_angle; //roll
  du_min_gih[1] = min_pitch_limit_rad - pitch_angle; // pitch
  du_min_gih[2] = du_min_thrust_z;
  du_min_gih[3] = (-actuator_state_filt_vect[8]*g1g2[4][8]);

  // Set upper limits limits
  du_max_gih[0] = roll_limit_rad - roll_angle; //roll
  du_max_gih[1] = max_pitch_limit_rad - pitch_angle; // pitch
  du_max_gih[2] = du_max_thrust_z;
  du_max_gih[3] = 9.0; // Hacky value to prevent drone from pitching down in transition

  // Set prefered states
  du_pref_gih[0] = 0; // prefered delta roll angle
  du_pref_gih[1] = -pitch_angle + pitch_pref_rad;// prefered delta pitch angle
  du_pref_gih[2] = du_max_gih[2]; // Low thrust better for efficiency
  du_pref_gih[3] = body_v[0]; // solve the body acceleration
}

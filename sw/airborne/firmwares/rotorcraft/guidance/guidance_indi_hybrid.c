/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to IROS2016 to learn more!
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"


// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifndef GUIDANCE_INDI_SPEED_GAIN
#define GUIDANCE_INDI_SPEED_GAIN 1.8
#define GUIDANCE_INDI_SPEED_GAINZ 1.8
#endif

#ifndef GUIDANCE_INDI_POS_GAIN
#define GUIDANCE_INDI_POS_GAIN 0.5
#define GUIDANCE_INDI_POS_GAINZ 0.5
#endif

#ifndef GUIDANCE_INDI_MIN_PITCH
#define GUIDANCE_INDI_MIN_PITCH -120
#define GUIDANCE_INDI_MAX_PITCH 25
#endif

#ifndef GUIDANCE_INDI_LIFTD_ASQ
#define GUIDANCE_INDI_LIFTD_ASQ 0.20
#endif

/* If lift effectiveness at low airspeed not defined,
 * just make one interpolation segment that connects to
 * the quadratic part from 12 m/s onward
 */
#ifndef GUIDANCE_INDI_LIFTD_P50
#define GUIDANCE_INDI_LIFTD_P80 (GUIDANCE_INDI_LIFTD_ASQ*12*12)
#define GUIDANCE_INDI_LIFTD_P50 (GUIDANCE_INDI_LIFTD_P80/2)
#endif

struct guidance_indi_hybrid_params gih_params = {
  .pos_gain = GUIDANCE_INDI_POS_GAIN,
  .pos_gainz = GUIDANCE_INDI_POS_GAINZ,

  .speed_gain = GUIDANCE_INDI_SPEED_GAIN,
  .speed_gainz = GUIDANCE_INDI_SPEED_GAINZ,

  .heading_bank_gain = GUIDANCE_INDI_HEADING_BANK_GAIN,
  .liftd_asq = GUIDANCE_INDI_LIFTD_ASQ, // coefficient of airspeed squared
  .liftd_p80 = GUIDANCE_INDI_LIFTD_P80,
  .liftd_p50 = GUIDANCE_INDI_LIFTD_P50,
};

#ifndef GUIDANCE_INDI_MAX_AIRSPEED
#error "You must have an airspeed sensor to use this guidance"
#endif
float guidance_indi_max_airspeed = GUIDANCE_INDI_MAX_AIRSPEED;

// Tell the guidance that the airspeed needs to be zeroed.
// Recomended to also put GUIDANCE_INDI_NAV_SPEED_MARGIN low in this case.
#ifndef GUIDANCE_INDI_ZERO_AIRSPEED
#define GUIDANCE_INDI_ZERO_AIRSPEED FALSE
#endif

/*Airspeed threshold where making a turn is "worth it"*/
#ifndef TURN_AIRSPEED_TH
#define TURN_AIRSPEED_TH 10.0
#endif

/*Boolean to force the heading to a static value (only use for specific experiments)*/
bool take_heading_control = false;

bool force_forward = false;

struct FloatVect3 sp_accel = {0.0,0.0,0.0};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS
#ifndef STABILIZATION_INDI_ACT_DYN_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS STABILIZATION_INDI_ACT_DYN_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS

#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

float inv_eff[4];

// Max bank angle in radians
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass accely_filt;

struct FloatVect2 desired_airspeed;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

float guidance_indi_hybrid_heading_sp = 0.f;
struct FloatEulers guidance_euler_cmd;
float thrust_in;

struct FloatVect3 gi_speed_sp = {0.0, 0.0, 0.0};

#ifndef GUIDANCE_INDI_VEL_SP_ID
#define GUIDANCE_INDI_VEL_SP_ID ABI_BROADCAST
#endif
abi_event vel_sp_ev;
static void vel_sp_cb(uint8_t sender_id, struct FloatVect3 *vel_sp);
struct FloatVect3 indi_vel_sp = {0.0, 0.0, 0.0};
float time_of_vel_sp = 0.0;

void guidance_indi_propagate_filters(void);
static void guidance_indi_calcg_wing(struct FloatMat33 *Gmat);
static float guidance_indi_get_liftd(float pitch, float theta);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_guidance_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
                              &sp_accel.x,
                              &sp_accel.y,
                              &sp_accel.z,
                              &euler_cmd.x,
                              &euler_cmd.y,
                              &euler_cmd.z,
                              &filt_accel_ned[0].o[0],
                              &filt_accel_ned[1].o[0],
                              &filt_accel_ned[2].o[0],
                              &gi_speed_sp.x,
                              &gi_speed_sp.y,
                              &gi_speed_sp.z);
}
#endif

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  /*AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);*/
  AbiBindMsgVEL_SP(GUIDANCE_INDI_VEL_SP_ID, &vel_sp_ev, vel_sp_cb);

  float tau = 1.0/(2.0*M_PI*filter_cutoff);
  float sample_time = 1.0/PERIODIC_FREQUENCY;
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_guidance_indi_hybrid);
#endif
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void) {
  thrust_in = stabilization_cmd[COMMAND_THRUST];
  thrust_act = thrust_in;
  guidance_indi_hybrid_heading_sp = stateGetNedToBodyEulers_f()->psi;

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);
}

#include "firmwares/rotorcraft/navigation.h"
/**
 * @param accel_sp accel setpoint in NED frame [m/s^2]
 * @param heading_sp the desired heading [rad]
 * @return stabilization setpoint structure
 *
 * main indi guidance function
 */
struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accel_sp, float heading_sp)
{
  // set global accel sp variable FIXME clean this
  sp_accel = *accel_sp;

  /* Obtain eulers with zxy rotation order */
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  /* Calculate the transition percentage so that the ctrl_effecitveness scheduling works */
  transition_percentage = BFP_OF_REAL((eulers_zxy.theta/RadOfDeg(-75.0f))*100,INT32_PERCENTAGE_FRAC);
  Bound(transition_percentage,0,BFP_OF_REAL(100.0f,INT32_PERCENTAGE_FRAC));
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage <<
        (INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100, max_offset, INT32_ANGLE_FRAC);

  // filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters();

#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
  // for rc control horizontal, rotate from body axes to NED
  float psi = eulers_zxy.psi;
  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  // for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
#endif

  // Calculate matrix of partial derivatives
  guidance_indi_calcg_wing(&Ga);
  // Invert this matrix
  MAT33_INV(Ga_inv, Ga);

  struct FloatVect3 accel_filt;
  accel_filt.x = filt_accel_ned[0].o[0];
  accel_filt.y = filt_accel_ned[1].o[0];
  accel_filt.z = filt_accel_ned[2].o[0];

  struct FloatVect3 a_diff;
  a_diff.x = sp_accel.x - accel_filt.x;
  a_diff.y = sp_accel.y - accel_filt.y;
  a_diff.z = sp_accel.z - accel_filt.z;

  // Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  // If the thrust to specific force ratio has been defined, include vertical control
  // else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
  a_diff.z = 0.0;
#endif
#endif

  //Calculate roll,pitch and thrust command
  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

  //printf("abi thrust %f\n", euler_cmd.z);
  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, euler_cmd.z);

  // Coordinated turn
  // feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega;
  const float max_phi = RadOfDeg(60.0f);
#if GUIDANCE_INDI_ZERO_AIRSPEED
  float airspeed_turn = 0.f;
#else
  float airspeed_turn = stateGetAirspeed_f();
#endif
  // We are dividing by the airspeed, so a lower bound is important
  Bound(airspeed_turn, 10.0f, 30.0f);

  guidance_euler_cmd.phi = roll_filt.o[0] + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt.o[0] + euler_cmd.y;

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, RadOfDeg(GUIDANCE_INDI_MIN_PITCH), RadOfDeg(GUIDANCE_INDI_MAX_PITCH));

  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;

  if( (guidance_euler_cmd.theta > 0.0f) && ( fabs(guidance_euler_cmd.phi) < guidance_euler_cmd.theta)) {
    coordinated_turn_roll = ((guidance_euler_cmd.phi > 0.0f) - (guidance_euler_cmd.phi < 0.0f)) * guidance_euler_cmd.theta;
  }

  if (fabsf(coordinated_turn_roll) < max_phi) {
    omega = 9.81f / airspeed_turn * tanf(coordinated_turn_roll);
  } else { //max 60 degrees roll
    omega = 9.81f / airspeed_turn * 1.72305f * ((coordinated_turn_roll > 0.0f) - (coordinated_turn_roll < 0.0f));
  }

#ifdef FWD_SIDESLIP_GAIN
  // Add sideslip correction
  omega -= accely_filt.o[0]*FWD_SIDESLIP_GAIN;
#endif

  // For a hybrid it is important to reduce the sideslip, which is done by changing the heading.
  // For experiments, it is possible to fix the heading to a different value.
  if (take_heading_control) {
    // heading is fixed by nav
    guidance_euler_cmd.psi = heading_sp;
  }
  else {
    // heading is free and controlled by guidance
    guidance_indi_hybrid_heading_sp += omega / PERIODIC_FREQUENCY;
    FLOAT_ANGLE_NORMALIZE(guidance_indi_hybrid_heading_sp);
    // limit heading setpoint to be within bounds of current heading
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    float delta_limit = STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    float heading = stabilization_attitude_get_heading_f();
    float delta_psi = guidance_indi_hybrid_heading_sp - heading;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit) {
      guidance_indi_hybrid_heading_sp = heading + delta_limit;
    } else if (delta_psi < -delta_limit) {
      guidance_indi_hybrid_heading_sp = heading - delta_limit;
    }
    FLOAT_ANGLE_NORMALIZE(guidance_indi_hybrid_heading_sp);
#endif
    guidance_euler_cmd.psi = guidance_indi_hybrid_heading_sp;
  }

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  guidance_indi_filter_thrust();

  // Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
  thrust_in = thrust_filt.o[0] + euler_cmd.z * guidance_indi_specific_force_gain;
  Bound(thrust_in, GUIDANCE_INDI_MIN_THROTTLE, 9600);

#if GUIDANCE_INDI_RC_DEBUG
  if (radio_control.values[RADIO_THROTTLE] < 300) {
    thrust_in = 0;
  }
#endif

  // Overwrite the thrust command from guidance_v
  stabilization_cmd[COMMAND_THRUST] = thrust_in;
#endif

  // Set the quaternion setpoint from eulers_zxy
  struct FloatQuat sp_quat;
  float_quat_of_eulers_zxy(&sp_quat, &guidance_euler_cmd);
  float_quat_normalize(&sp_quat);

  return stab_sp_from_quat_f(&sp_quat);
}

// compute accel setpoint from speed setpoint (use global variables ! FIXME)
static struct FloatVect3 compute_accel_from_speed_sp(void)
{
  struct FloatVect3 accel_sp = { 0.f, 0.f, 0.f };

  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  //for rc control horizontal, rotate from body axes to NED
  float psi = eulers_zxy.psi;
  float cpsi = cosf(psi);
  float spsi = sinf(psi);
  float speed_sp_b_x =  cpsi * gi_speed_sp.x + spsi * gi_speed_sp.y;
  float speed_sp_b_y = -spsi * gi_speed_sp.x + cpsi * gi_speed_sp.y;

  // Get airspeed or zero it
#if GUIDANCE_INDI_ZERO_AIRSPEED
  float airspeed = 0.f;
#else
  float airspeed = stateGetAirspeed_f();
#endif
  struct NedCoor_f *groundspeed = stateGetSpeedNed_f();
  struct FloatVect2 airspeed_v = { cpsi * airspeed, spsi * airspeed };
  struct FloatVect2 windspeed;
  VECT2_DIFF(windspeed, *groundspeed, airspeed_v);

  VECT2_DIFF(desired_airspeed, gi_speed_sp, windspeed); // Use 2d part of gi_speed_sp
  float norm_des_as = FLOAT_VECT2_NORM(desired_airspeed);

  // Make turn instead of straight line
  if ((airspeed > TURN_AIRSPEED_TH) && (norm_des_as > (TURN_AIRSPEED_TH+2.0f))) {

    // Give the wind cancellation priority.
    if (norm_des_as > guidance_indi_max_airspeed) {
      float groundspeed_factor = 0.0f;

      // if the wind is faster than we can fly, just fly in the wind direction
      if (FLOAT_VECT2_NORM(windspeed) < guidance_indi_max_airspeed) {
        float av = gi_speed_sp.x * gi_speed_sp.x + gi_speed_sp.y * gi_speed_sp.y;
        float bv = -2.f * (windspeed.x * gi_speed_sp.x + windspeed.y * gi_speed_sp.y);
        float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - guidance_indi_max_airspeed * guidance_indi_max_airspeed;

        float dv = bv * bv - 4.0f * av * cv;

        // dv can only be positive, but just in case
        if (dv < 0.0f) {
          dv = fabsf(dv);
        }
        float d_sqrt = sqrtf(dv);

        groundspeed_factor = (-bv + d_sqrt)  / (2.0f * av);
      }

      desired_airspeed.x = groundspeed_factor * gi_speed_sp.x - windspeed.x;
      desired_airspeed.y = groundspeed_factor * gi_speed_sp.y - windspeed.y;

      speed_sp_b_x = guidance_indi_max_airspeed;
    }

    // desired airspeed can not be larger than max airspeed
    speed_sp_b_x = Min(norm_des_as, guidance_indi_max_airspeed);

    if (force_forward) {
      speed_sp_b_x = guidance_indi_max_airspeed;
    }

    // Calculate accel sp in body axes, because we need to regulate airspeed
    struct FloatVect2 sp_accel_b;
    // In turn acceleration proportional to heading diff
    sp_accel_b.y = atan2f(desired_airspeed.y, desired_airspeed.x) - psi;
    FLOAT_ANGLE_NORMALIZE(sp_accel_b.y);
    sp_accel_b.y *= gih_params.heading_bank_gain;

    // Control the airspeed
    sp_accel_b.x = (speed_sp_b_x - airspeed) * gih_params.speed_gain;

    accel_sp.x = cpsi * sp_accel_b.x - spsi * sp_accel_b.y;
    accel_sp.y = spsi * sp_accel_b.x + cpsi * sp_accel_b.y;
    accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  }
  else { // Go somewhere in the shortest way

    if (airspeed > 10.f) {
      // Groundspeed vector in body frame
      float groundspeed_x = cpsi * stateGetSpeedNed_f()->x + spsi * stateGetSpeedNed_f()->y;
      float speed_increment = speed_sp_b_x - groundspeed_x;

      // limit groundspeed setpoint to max_airspeed + (diff gs and airspeed)
      if ((speed_increment + airspeed) > guidance_indi_max_airspeed) {
        speed_sp_b_x = guidance_indi_max_airspeed + groundspeed_x - airspeed;
      }
    }

    gi_speed_sp.x = cpsi * speed_sp_b_x - spsi * speed_sp_b_y;
    gi_speed_sp.y = spsi * speed_sp_b_x + cpsi * speed_sp_b_y;

    accel_sp.x = (gi_speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    accel_sp.y = (gi_speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
    accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  }

  // Bound the acceleration setpoint
  float accelbound = 3.0f + airspeed / guidance_indi_max_airspeed * 5.0f; // FIXME remove hard coded values
  float_vect3_bound_in_2d(&accel_sp, accelbound);
  /*BoundAbs(sp_accel.x, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  /*BoundAbs(sp_accel.y, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  BoundAbs(accel_sp.z, 3.0);

  //printf("accel_sp %f %f %f\n", accel_sp.x, accel_sp.y, accel_sp.z);
  return accel_sp;
}

struct StabilizationSetpoint guidance_indi_run_pos(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv)
{
  struct FloatVect3 pos_err;
  struct FloatVect3 accel_sp;

  //Linear controller to find the acceleration setpoint from position and velocity
  pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
  pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
  pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;

  // First check for velocity setpoint from module // FIXME should be called like this
  float dt = get_sys_time_float() - time_of_vel_sp;
  // If the input command is not updated after a timeout, switch back to flight plan control
  if (dt < 0.5) {
    gi_speed_sp.x = indi_vel_sp.x;
    gi_speed_sp.y = indi_vel_sp.y;
    gi_speed_sp.z = indi_vel_sp.z;
  } else {
    gi_speed_sp.x = pos_err.x * gih_params.pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
    gi_speed_sp.y = pos_err.y * gih_params.pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    gi_speed_sp.z = pos_err.z * gih_params.pos_gainz + SPEED_FLOAT_OF_BFP(gv->zd_ref);
  }

  // Bound vertical speed setpoint
  if (stateGetAirspeed_f() > 13.f) {
    Bound(gi_speed_sp.z, -4.0f, 4.0f); // FIXME no harcoded values
  } else {
    Bound(gi_speed_sp.z, nav.descend_vspeed, nav.climb_vspeed); // FIXME don't use nav settings
  }

  accel_sp = compute_accel_from_speed_sp(); // compute accel sp

  return guidance_indi_run(&accel_sp, gh->sp.heading);
}

struct StabilizationSetpoint guidance_indi_run_speed(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv)
{
  struct FloatVect3 accel_sp;

  // First check for velocity setpoint from module // FIXME should be called like this
  float dt = get_sys_time_float() - time_of_vel_sp;
  // If the input command is not updated after a timeout, switch back to flight plan control
  if (dt < 0.5) {
    gi_speed_sp.x = indi_vel_sp.x;
    gi_speed_sp.y = indi_vel_sp.y;
    gi_speed_sp.z = indi_vel_sp.z;
  } else {
    gi_speed_sp.x = SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
    gi_speed_sp.y = SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
  }

  // Bound vertical speed setpoint
  if (stateGetAirspeed_f() > 13.f) {
    Bound(gi_speed_sp.z, -4.0f, 4.0f); // FIXME no harcoded values
  } else {
    Bound(gi_speed_sp.z, nav.descend_vspeed, nav.climb_vspeed); // FIXME don't use nav settings
  }

  accel_sp = compute_accel_from_speed_sp(); // compute accel sp

  return guidance_indi_run(&accel_sp, gh->sp.heading);
}

struct StabilizationSetpoint guidance_indi_run_accel(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv)
{
  struct FloatVect3 accel_sp;

  gi_speed_sp.x = 0.f;
  gi_speed_sp.y = 0.f;
  gi_speed_sp.z = 0.f;

  accel_sp.x = (gi_speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
  accel_sp.y = (gi_speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
  accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz + ACCEL_FLOAT_OF_BFP(gv->zdd_ref);

  return guidance_indi_run(&accel_sp, gh->sp.heading);
}

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);

  // same filter as for the acceleration
  update_butterworth_2_low_pass(&thrust_filt, thrust_act);
}
#endif

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_propagate_filters(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers_zxy.phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers_zxy.theta);

  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);
}

/**
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations, taking into account the lift of a wing that is
 * horizontal at -90 degrees pitch
 *
 * @param Gmat array to write the matrix to [3x3]
 */
void guidance_indi_calcg_wing(struct FloatMat33 *Gmat) {

  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  /*Amount of lift produced by the wing*/
  float pitch_lift = eulers_zxy.theta;
  Bound(pitch_lift,-M_PI_2,0);
  float lift = sinf(pitch_lift)*9.81;
  float T = cosf(pitch_lift)*-9.81;

  // get the derivative of the lift wrt to theta
  float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);

  RMAT_ELMT(*Gmat, 0, 0) =  cphi*ctheta*spsi*T + cphi*spsi*lift;
  RMAT_ELMT(*Gmat, 1, 0) = -cphi*ctheta*cpsi*T - cphi*cpsi*lift;
  RMAT_ELMT(*Gmat, 2, 0) = -sphi*ctheta*T -sphi*lift;
  RMAT_ELMT(*Gmat, 0, 1) = (ctheta*cpsi - sphi*stheta*spsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
  RMAT_ELMT(*Gmat, 1, 1) = (ctheta*spsi + sphi*stheta*cpsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
  RMAT_ELMT(*Gmat, 2, 1) = -cphi*stheta*T*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
  RMAT_ELMT(*Gmat, 0, 2) = stheta*cpsi + sphi*ctheta*spsi;
  RMAT_ELMT(*Gmat, 1, 2) = stheta*spsi - sphi*ctheta*cpsi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
}

/**
 * @brief Get the derivative of lift w.r.t. pitch.
 *
 * @param airspeed The airspeed says most about the flight condition
 *
 * @return The derivative of lift w.r.t. pitch
 */
float guidance_indi_get_liftd(float airspeed, float theta) {
  float liftd = 0.0;

  if(airspeed < 12) {
  /* Assume the airspeed is too low to be measured accurately
    * Use scheduling based on pitch angle instead.
    * You can define two interpolation segments
    */
    float pitch_interp = DegOfRad(theta);
    const float min_pitch = -80.0;
    const float middle_pitch = -50.0;
    const float max_pitch = -20.0;

    Bound(pitch_interp, min_pitch, max_pitch);
    if (pitch_interp > middle_pitch) {
      float ratio = (pitch_interp - max_pitch)/(middle_pitch - max_pitch);
      liftd = -gih_params.liftd_p50*ratio;
    } else {
      float ratio = (pitch_interp - middle_pitch)/(min_pitch - middle_pitch);
      liftd = -(gih_params.liftd_p80-gih_params.liftd_p50)*ratio - gih_params.liftd_p50;
    }
  } else {
    liftd = -gih_params.liftd_asq*airspeed*airspeed;
  }

  //TODO: bound liftd
  return liftd;
}

/**
 * ABI callback that obtains the velocity setpoint from a module
  */
static void vel_sp_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *vel_sp)
{
  indi_vel_sp.x = vel_sp->x;
  indi_vel_sp.y = vel_sp->y;
  indi_vel_sp.z = vel_sp->z;
  time_of_vel_sp = get_sys_time_float();
}


#if GUIDANCE_INDI_HYBRID_USE_AS_DEFAULT
// guidance indi control function is implementing the default functions of guidance

void guidance_h_run_enter(void)
{
  guidance_indi_enter();
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_pos(in_flight, gh, &guidance_v);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_speed(in_flight, gh, &guidance_v);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_accel(in_flight, gh, &guidance_v);
}

int32_t guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv UNUSED)
{
  return (int32_t)thrust_in; // nothing to do
}

int32_t guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv UNUSED)
{
  return (int32_t)thrust_in; // nothing to do
}

int32_t guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv UNUSED)
{
  return (int32_t)thrust_in; // nothing to do
}

#endif


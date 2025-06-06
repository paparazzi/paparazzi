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
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "state.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/navigation.h"


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

#ifndef GUIDANCE_INDI_LIFTD_ASQ
#define GUIDANCE_INDI_LIFTD_ASQ 0.20
#endif

#ifndef GUIDANCE_INDI_MAX_PUSHER_INCREMENT
#define GUIDANCE_INDI_MAX_PUSHER_INCREMENT MAX_PPRZ
#endif

/* If lift effectiveness at low airspeed not defined,
 * just make one interpolation segment that connects to
 * the quadratic part from 12 m/s onward
 */
#ifndef GUIDANCE_INDI_LIFTD_P50
#define GUIDANCE_INDI_LIFTD_P80 (GUIDANCE_INDI_LIFTD_ASQ*12*12)
#define GUIDANCE_INDI_LIFTD_P50 (GUIDANCE_INDI_LIFTD_P80/2)
#endif

#ifndef GUIDANCE_INDI_MAX_AIRSPEED
#error "You must have an airspeed sensor to use this guidance"
#endif

#ifndef GUIDANCE_INDI_MIN_AIRSPEED
#define GUIDANCE_INDI_MIN_AIRSPEED -10.f
#endif

/**
 * Climb speed when navigation is making turns instead of direct lines
 */
#ifndef GUIDANCE_INDI_FWD_CLIMB_SPEED
#define GUIDANCE_INDI_FWD_CLIMB_SPEED 4.0
#endif

/**
 * Descend speed when navigation is making turns instead of direct lines
 */
#ifndef GUIDANCE_INDI_FWD_DESCEND_SPEED
#define GUIDANCE_INDI_FWD_DESCEND_SPEED -4.0
#endif

/**
 * Climb speed when navigation is doing direct lines
 */
#ifndef GUIDANCE_INDI_QUAD_CLIMB_SPEED
#define GUIDANCE_INDI_QUAD_CLIMB_SPEED 2.0
#endif

/**
 * Descend speed when navigation is doing direct lines
 */
#ifndef GUIDANCE_INDI_QUAD_DESCEND_SPEED
#define GUIDANCE_INDI_QUAD_DESCEND_SPEED -2.0
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
  .min_airspeed = GUIDANCE_INDI_MIN_AIRSPEED,
  .max_airspeed = GUIDANCE_INDI_MAX_AIRSPEED,
  .stall_protect_gain = 1.5, // m/s^2 downward acceleration per m/s airspeed loss
  .climb_vspeed_fwd = GUIDANCE_INDI_FWD_CLIMB_SPEED,
  .descend_vspeed_fwd = GUIDANCE_INDI_FWD_DESCEND_SPEED,
  .climb_vspeed_quad = GUIDANCE_INDI_QUAD_CLIMB_SPEED,
  .descend_vspeed_quad = GUIDANCE_INDI_QUAD_DESCEND_SPEED,
};

// Quadplanes can hover at various pref pitch
float guidance_indi_pitch_pref_deg = 0;


// If using WLS, check that the matrix size is sufficient
#if GUIDANCE_INDI_HYBRID_USE_WLS
#if GUIDANCE_INDI_HYBRID_U > WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX too small: increase WLS_N_U_MAX in airframe file
#endif

#if GUIDANCE_INDI_HYBRID_V > WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX too small: increase WLS_N_V_MAX in airframe file
#endif
#endif


// Tell the guidance that the airspeed needs to be zeroed.
// Recomended to also put GUIDANCE_INDI_NAV_SPEED_MARGIN low in this case.
#ifndef GUIDANCE_INDI_ZERO_AIRSPEED
#define GUIDANCE_INDI_ZERO_AIRSPEED FALSE
#endif

/*Airspeed threshold where making a turn is "worth it"*/
#ifndef TURN_AIRSPEED_TH
#define TURN_AIRSPEED_TH 13.0
#endif

/*Boolean to force the heading to a static value (only use for specific experiments)*/
bool take_heading_control = false;

bool force_forward = false;

bool guidance_indi_airspeed_filtering = false;


struct FloatVect3 sp_accel = {0.0,0.0,0.0};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifdef GUIDANCE_INDI_THRUST_DYNAMICS
#warning GUIDANCE_INDI_THRUST_DYNAMICS is deprecated, use GUIDANCE_INDI_THRUST_DYNAMICS_FREQ instead.
#warning "The thrust dynamics are now specified in continuous time with the corner frequency of the first order model!"
#warning "define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ in rad/s"
#warning "Use -ln(1 - old_number) * PERIODIC_FREQUENCY to compute it from the old value."
#endif

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS_FREQ
#ifndef STABILIZATION_INDI_ACT_FREQ_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ STABILIZATION_INDI_ACT_FREQ_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS_FREQ

#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

#ifndef GUIDANCE_INDI_AIRSPEED_FILT_CUTOFF
#define GUIDANCE_INDI_AIRSPEED_FILT_CUTOFF 0.5
#endif

#ifndef GUIDANCE_INDI_MAX_LAT_ACCEL
#define GUIDANCE_INDI_MAX_LAT_ACCEL 9.81
#endif

#ifndef GUIDANCE_INDI_COORDINATED_TURN_MIN_AIRSPEED
#define GUIDANCE_INDI_COORDINATED_TURN_MIN_AIRSPEED 10.0
#endif

#ifndef GUIDANCE_INDI_COORDINATED_TURN_MAX_AIRSPEED
#define GUIDANCE_INDI_COORDINATED_TURN_MAX_AIRSPEED 30.0
#endif

#ifndef GUIDANCE_INDI_COORDINATED_TURN_AIRSPEED_MARGIN
#define GUIDANCE_INDI_COORDINATED_TURN_AIRSPEED_MARGIN 0.0
#endif

float inv_eff[4];

// Max bank angle in radians
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;
float guidance_indi_min_pitch = GUIDANCE_INDI_MIN_PITCH;

#if defined(ROTWING_STATE_FW_MAX_AIRSPEED) && defined(ROTWING_STATE_QUAD_MAX_AIRSPEED)
  float gih_coordinated_turn_min_airspeed = ROTWING_STATE_QUAD_MAX_AIRSPEED;
  float gih_coordinated_turn_max_airspeed = ROTWING_STATE_FW_MAX_AIRSPEED + GUIDANCE_INDI_COORDINATED_TURN_AIRSPEED_MARGIN;
#else
  float gih_coordinated_turn_min_airspeed = GUIDANCE_INDI_COORDINATED_TURN_MIN_AIRSPEED;
  float gih_coordinated_turn_max_airspeed = GUIDANCE_INDI_COORDINATED_TURN_MAX_AIRSPEED + GUIDANCE_INDI_COORDINATED_TURN_AIRSPEED_MARGIN;
#endif

bool coordinated_turn_use_accel = false;

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;

float thrust_dyn = 0.f;
float thrust_act = 0.f;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass accely_filt;
Butterworth2LowPass guidance_indi_airspeed_filt;

struct FloatVect2 desired_airspeed;
float gi_unbounded_airspeed_sp = 0.f;

float Ga[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U];
struct FloatVect3 euler_cmd;

float du_gih[GUIDANCE_INDI_HYBRID_U]; // = {0.0f, 0.0f, 0.0f};

#if GUIDANCE_INDI_HYBRID_USE_WLS
#include "math/wls/wls_alloc.h"
float *Bwls_gih[GUIDANCE_INDI_HYBRID_V];
struct WLS_t wls_guid_p = {
  .nu        = GUIDANCE_INDI_HYBRID_U,
  .nv        = GUIDANCE_INDI_HYBRID_V,
  .gamma_sq  = 100000.0,
  .v         = {0.0},
#ifdef GUIDANCE_INDI_WLS_PRIORITIES
  .Wv        =  GUIDANCE_INDI_WLS_PRIORITIES,
#else // X,Y accel, Z accel
  .Wv        =  { 100.f, 100.f, 1.f },
#endif  
#ifdef GUIDANCE_INDI_WLS_WU 
  .Wu        = GUIDANCE_INDI_WLS_WU,
#else
  .Wu        = {[0 ... GUIDANCE_INDI_HYBRID_U - 1] = 1.0},
#endif
  .u_pref    = {0.0},
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};
#endif
// The control objective
float v_gih[3];

// Filters
float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_airspeed_filt_cutoff = GUIDANCE_INDI_AIRSPEED_FILT_CUTOFF;

float guidance_indi_hybrid_heading_sp = 0.f;
struct FloatEulers guidance_euler_cmd;
struct ThrustSetpoint thrust_sp;
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

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_eff_mat_guid_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EFF_MAT_GUID(trans, dev, AC_ID, 
                GUIDANCE_INDI_HYBRID_U, Ga[0],
                GUIDANCE_INDI_HYBRID_U, Ga[1],
                GUIDANCE_INDI_HYBRID_U, Ga[2]);
}
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

#if GUIDANCE_INDI_HYBRID_USE_WLS
static void send_wls_v_guid(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_v("guid", &wls_guid_p, trans, dev); 
}
static void send_wls_u_guid(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_u("guid", &wls_guid_p, trans, dev); 
}
#endif // GUIDANCE_INDI_HYBRID_USE_WLS

#endif // PERIODIC_TELEMETRY

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  /*AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);*/
  AbiBindMsgVEL_SP(GUIDANCE_INDI_VEL_SP_ID, &vel_sp_ev, vel_sp_cb);

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifdef GUIDANCE_INDI_THRUST_DYNAMICS
  thrust_dyn = GUIDANCE_INDI_THRUST_DYNAMICS;
#else
  thrust_dyn = 1-exp(-GUIDANCE_INDI_THRUST_DYNAMICS_FREQ/PERIODIC_FREQUENCY);
#endif
#endif

  float tau = 1.0/(2.0*M_PI*filter_cutoff);
  float sample_time = 1.0/PERIODIC_FREQUENCY;
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);

  float tau_guidance_indi_airspeed = 1.0/(2.0*M_PI*guidance_indi_airspeed_filt_cutoff);
  init_butterworth_2_low_pass(&guidance_indi_airspeed_filt, tau_guidance_indi_airspeed, sample_time, 0.0);

#if GUIDANCE_INDI_HYBRID_USE_WLS
  for (int8_t i = 0; i < GUIDANCE_INDI_HYBRID_V; i++) {
    Bwls_gih[i] = Ga[i];
  }
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_guidance_indi_hybrid);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_GUID, send_eff_mat_guid_indi_hybrid);
#if GUIDANCE_INDI_HYBRID_USE_WLS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_guid);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_guid);
#endif
#endif
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void)
{
  /*Obtain eulers with zxy rotation order*/
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  nav.heading = eulers_zxy.psi;

  thrust_in = stabilization.cmd[COMMAND_THRUST];
  thrust_act = thrust_in;
  guidance_indi_hybrid_heading_sp = eulers_zxy.psi;

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }

  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, eulers_zxy.phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, eulers_zxy.theta);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);

  float tau_guidance_indi_airspeed = 1.0/(2.0*M_PI*guidance_indi_airspeed_filt_cutoff);
  init_butterworth_2_low_pass(&guidance_indi_airspeed_filt, tau_guidance_indi_airspeed, sample_time, 0.0);
}

void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed) {
  gih_params.min_airspeed = min_airspeed;
  gih_params.max_airspeed = max_airspeed;
}

void guidance_set_max_bank_angle(float max_bank) {
  guidance_indi_max_bank = max_bank;
}

void guidance_set_max_climb_speed(float max_climb_speed_quad, float max_climb_speed_fwd) {
  gih_params.climb_vspeed_quad = max_climb_speed_quad;
  gih_params.climb_vspeed_fwd = max_climb_speed_fwd;
}

void guidance_set_max_descend_speed(float max_descend_speed_quad, float max_descend_speed_fwd) {
  gih_params.descend_vspeed_quad = max_descend_speed_quad;
  gih_params.descend_vspeed_fwd = max_descend_speed_fwd;
}

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

  /* Calculate the transition ratio so that the ctrl_effecitveness scheduling works */
  stabilization.transition_ratio = eulers_zxy.theta / RadOfDeg(-75.0f);
  Bound(stabilization.transition_ratio, 0.f, 1.f);

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

  struct FloatVect3 accel_filt;
  accel_filt.x = filt_accel_ned[0].o[0];
  accel_filt.y = filt_accel_ned[1].o[0];
  accel_filt.z = filt_accel_ned[2].o[0];

  struct FloatVect3 a_diff;
  VECT3_DIFF(a_diff, sp_accel, accel_filt);

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

  // Calculate matrix of partial derivatives and control objective
  guidance_indi_calcg_wing(Ga, a_diff, v_gih);

#if GUIDANCE_INDI_HYBRID_USE_WLS

  // Calculate the maximum deflections
  guidance_indi_hybrid_set_wls_settings(v_gih, roll_filt.o[0], pitch_filt.o[0]);

  float du_gih[GUIDANCE_INDI_HYBRID_U]; // = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < GUIDANCE_INDI_HYBRID_V; i++) {
    wls_guid_p.v[i] = v_gih[i];
  }
  wls_alloc(&wls_guid_p, Bwls_gih, 0, 0, 10);
  for (int i = 0; i < GUIDANCE_INDI_HYBRID_U; i++) {
    du_gih[i] = wls_guid_p.u[i];
  }
  euler_cmd.x = du_gih[0];
  euler_cmd.y = du_gih[1];
  euler_cmd.z = du_gih[2];

#else
  // compute inverse matrix of Ga
  float Ga_inv[3][3] = {};
  float_mat_inv_3d(Ga_inv, Ga);
  // Calculate roll,pitch and thrust command
  float_mat3_mult(&euler_cmd, Ga_inv, a_diff);
#endif

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
  Bound(airspeed_turn, gih_coordinated_turn_min_airspeed, gih_coordinated_turn_max_airspeed);

  guidance_euler_cmd.phi = roll_filt.o[0] + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt.o[0] + euler_cmd.y;

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, RadOfDeg(guidance_indi_min_pitch), RadOfDeg(GUIDANCE_INDI_MAX_PITCH));

  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;

  // When tilting backwards (e.g. waypoint behind the drone), we have to yaw around to face the direction
  // of flight even when the drone is not rolling much (yet). Determine the shortest direction in which to yaw by
  // looking at the roll angle.
  if( (eulers_zxy.theta > 0.0f) && ( fabs(eulers_zxy.phi) < eulers_zxy.theta)) {
    if (eulers_zxy.phi > 0.0f) {
      coordinated_turn_roll = eulers_zxy.theta;
    } else {
      coordinated_turn_roll = -eulers_zxy.theta;
    }
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

  // We can pre-compute the required rates to achieve this turn rate:
  // NOTE: there *should* not be any problems possible with Euler singularities here
  struct FloatEulers *euler_zyx = stateGetNedToBodyEulers_f();

  struct FloatRates ff_rates;

  ff_rates.p = -sinf(euler_zyx->theta) * omega;
  ff_rates.q =  cosf(euler_zyx->theta) * sinf(euler_zyx->phi) * omega;
  ff_rates.r =  cosf(euler_zyx->theta) * cosf(euler_zyx->phi) * omega;

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

  // compute required thrust setpoint
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
  // return required thrust
  thrust_sp = th_sp_from_thrust_i(thrust_in, THRUST_AXIS_Z);

#else
  float thrust_vect[3];
#if GUIDANCE_INDI_HYBRID_U > 3
  thrust_vect[0] = du_gih[3];
  if (thrust_vect[0] > GUIDANCE_INDI_MAX_PUSHER_INCREMENT*g1g2[4][GUIDANCE_INDI_PUSHER_INDEX]) {
    thrust_vect[0] = GUIDANCE_INDI_MAX_PUSHER_INCREMENT*g1g2[4][GUIDANCE_INDI_PUSHER_INDEX];
  }
#else
  thrust_vect[0] = 0;
#endif
  thrust_vect[1] = 0;
  thrust_vect[2] = euler_cmd.z;
  // specific force not defined, return required increment
  thrust_sp = th_sp_from_incr_vect_f(thrust_vect);
#endif

  // Set the quaternion setpoint from eulers_zxy
  struct FloatQuat sp_quat;
  float_quat_of_eulers_zxy(&sp_quat, &guidance_euler_cmd);
  float_quat_normalize(&sp_quat);

  return stab_sp_from_quat_ff_rates_f(&sp_quat, &ff_rates);
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
  Bound(airspeed, 0.0f, 100.0f);
  if (guidance_indi_airspeed_filtering) {
    airspeed = guidance_indi_airspeed_filt.o[0];
  }
#endif
  struct NedCoor_f *groundspeed = stateGetSpeedNed_f();
  struct FloatVect2 airspeed_v = { cpsi * airspeed, spsi * airspeed };
  struct FloatVect2 windspeed;
  VECT2_DIFF(windspeed, *groundspeed, airspeed_v);

  VECT2_DIFF(desired_airspeed, gi_speed_sp, windspeed); // Use 2d part of gi_speed_sp
  float norm_des_as = FLOAT_VECT2_NORM(desired_airspeed);

  gi_unbounded_airspeed_sp = norm_des_as;

  // Check if some minimum airspeed is desired (e.g. to prevent stall)
  if (norm_des_as < gih_params.min_airspeed) {
     norm_des_as = gih_params.min_airspeed;
  }

  float gi_airspeed_sp = norm_des_as;

  // Make turn instead of straight line, control airspeed
  if ((airspeed > TURN_AIRSPEED_TH) && (norm_des_as > (TURN_AIRSPEED_TH+2.0f))) {

    // Give the wind cancellation priority.
    if (norm_des_as > gih_params.max_airspeed) {
      float groundspeed_factor = 0.0f;

      // if the wind is faster than we can fly, just fly in the wind direction
      if (FLOAT_VECT2_NORM(windspeed) < gih_params.max_airspeed) {
        float av = gi_speed_sp.x * gi_speed_sp.x + gi_speed_sp.y * gi_speed_sp.y;
        float bv = -2.f * (windspeed.x * gi_speed_sp.x + windspeed.y * gi_speed_sp.y);
        float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - gih_params.max_airspeed * gih_params.max_airspeed;

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

      gi_airspeed_sp = gih_params.max_airspeed;
    }

    if (force_forward) {
      gi_airspeed_sp = gih_params.max_airspeed;
    }

    // Calculate accel sp in body axes, because we need to regulate airspeed
    struct FloatVect2 sp_accel_b;
    // In turn acceleration proportional to heading diff
    sp_accel_b.y = atan2f(desired_airspeed.y, desired_airspeed.x) - psi;
    FLOAT_ANGLE_NORMALIZE(sp_accel_b.y);
    sp_accel_b.y *= gih_params.heading_bank_gain;

    BoundAbs(sp_accel_b.y, GUIDANCE_INDI_MAX_LAT_ACCEL);

    // Control the airspeed
    sp_accel_b.x = (gi_airspeed_sp - airspeed) * gih_params.speed_gain;

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
      if ((speed_increment + airspeed) > gih_params.max_airspeed) {
        speed_sp_b_x = gih_params.max_airspeed + groundspeed_x - airspeed;
      }
    }

    gi_speed_sp.x = cpsi * speed_sp_b_x - spsi * speed_sp_b_y;
    gi_speed_sp.y = spsi * speed_sp_b_x + cpsi * speed_sp_b_y;

    accel_sp.x = (gi_speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    accel_sp.y = (gi_speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
    accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  }

  // Bound the acceleration setpoint
  float accelbound = 3.0f + airspeed / gih_params.max_airspeed * 5.0f; // FIXME remove hard coded values
  float_vect3_bound_in_2d(&accel_sp, accelbound);
  BoundAbs(accel_sp.z, 3.0);

#ifdef ROTWING_FW_MIN_AIRSPEED
  if (!rotwing_state_pusher_motor_running() && !rotwing_state_hover_motors_running()) {
    accel_sp.z = gih_params.stall_protect_gain * (gi_airspeed_sp - airspeed);
    BoundAbs(accel_sp.z, 5.0);
  }
#endif

  return accel_sp;
}

static float bound_vz_sp(float vz_sp)
{
  // Bound vertical speed setpoint
  if (stateGetAirspeed_f() > TURN_AIRSPEED_TH) {
    Bound(vz_sp, -gih_params.climb_vspeed_fwd, -gih_params.descend_vspeed_fwd);
  } else {
    Bound(vz_sp, -gih_params.climb_vspeed_quad, -gih_params.descend_vspeed_quad);
  }
  return vz_sp;
}

struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndiHybrid_HMode h_mode, enum GuidanceIndiHybrid_VMode v_mode)
{
  struct FloatVect3 pos_err = { 0 };
  struct FloatVect3 accel_sp = { 0 };

  // First check for velocity setpoint from module // FIXME should be called like this
  float dt = get_sys_time_float() - time_of_vel_sp;
  // If the input command is not updated after a timeout, switch back to flight plan control
  if (dt < 0.5) {
    gi_speed_sp.x = indi_vel_sp.x;
    gi_speed_sp.y = indi_vel_sp.y;
    gi_speed_sp.z = indi_vel_sp.z;
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }

  if (h_mode == GUIDANCE_INDI_HYBRID_H_POS) {
    //Linear controller to find the acceleration setpoint from position and velocity
    pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
    pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
    gi_speed_sp.x = pos_err.x * gih_params.pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
    gi_speed_sp.y = pos_err.y * gih_params.pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    if (v_mode == GUIDANCE_INDI_HYBRID_V_POS) {
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      gi_speed_sp.z = bound_vz_sp(pos_err.z * gih_params.pos_gainz + SPEED_FLOAT_OF_BFP(gv->zd_ref));
    } else if (v_mode == GUIDANCE_INDI_HYBRID_V_SPEED) {
      gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    } else {
      gi_speed_sp.z = 0.f;
    }
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp
    if (v_mode == GUIDANCE_INDI_HYBRID_V_ACCEL) {
      accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz + ACCEL_FLOAT_OF_BFP(gv->zdd_ref); // overwrite accel
    }
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }
  else if (h_mode == GUIDANCE_INDI_HYBRID_H_SPEED) {
    gi_speed_sp.x = SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
    gi_speed_sp.y = SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    if (v_mode == GUIDANCE_INDI_HYBRID_V_POS) {
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      gi_speed_sp.z = bound_vz_sp(pos_err.z * gih_params.pos_gainz + SPEED_FLOAT_OF_BFP(gv->zd_ref));
    } else if (v_mode == GUIDANCE_INDI_HYBRID_V_SPEED) {
      gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    } else {
      gi_speed_sp.z = 0.f;
    }
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp
    if (v_mode == GUIDANCE_INDI_HYBRID_V_ACCEL) {
      accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz + ACCEL_FLOAT_OF_BFP(gv->zdd_ref); // overwrite accel
    }
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }
  else { // H_ACCEL
    gi_speed_sp.x = 0.f;
    gi_speed_sp.y = 0.f;
    if (v_mode == GUIDANCE_INDI_HYBRID_V_POS) {
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      gi_speed_sp.z = bound_vz_sp(pos_err.z * gih_params.pos_gainz + SPEED_FLOAT_OF_BFP(gv->zd_ref));
    } else if (v_mode == GUIDANCE_INDI_HYBRID_V_SPEED) {
      gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    } else {
      gi_speed_sp.z = 0.f;
    }
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp in case z control is required
    // overwrite accel X and Y
    accel_sp.x = (gi_speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
    accel_sp.y = (gi_speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
    if (v_mode == GUIDANCE_INDI_HYBRID_V_ACCEL) {
      accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz + ACCEL_FLOAT_OF_BFP(gv->zdd_ref); // overwrite accel
    }
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }
}

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + thrust_dyn * (thrust_in - thrust_act);

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
void guidance_indi_propagate_filters(void)
{
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers_zxy.phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers_zxy.theta);

  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);

  float airspeed = stateGetAirspeed_f();
  Bound(airspeed, 0.0f, 100.0f);
  update_butterworth_2_low_pass(&guidance_indi_airspeed_filt, airspeed);
}


/**
 * @brief Get the derivative of lift w.r.t. pitch.
 *
 * @param airspeed The airspeed says most about the flight condition
 *
 * @return The derivative of lift w.r.t. pitch
 */
float WEAK guidance_indi_get_liftd(float airspeed, float theta) {
  float liftd = 0.0f;

  if (airspeed < 12.f) {
  /* Assume the airspeed is too low to be measured accurately
    * Use scheduling based on pitch angle instead.
    * You can define two interpolation segments
    */
    float pitch_interp = DegOfRad(theta);
    const float min_pitch = -80.0f;
    const float middle_pitch = -50.0f;
    const float max_pitch = -20.0f;

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

static struct VerticalGuidance *_gv = &guidance_v;
static enum GuidanceIndiHybrid_VMode _v_mode = GUIDANCE_INDI_HYBRID_V_POS;

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_HYBRID_H_POS, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_HYBRID_H_SPEED, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_HYBRID_H_ACCEL, _v_mode);
}

struct ThrustSetpoint guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_HYBRID_V_POS;
  return thrust_sp;
}

struct ThrustSetpoint guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_HYBRID_V_SPEED;
  return thrust_sp;
}

struct ThrustSetpoint guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_HYBRID_V_ACCEL;
  return thrust_sp;
}

#endif


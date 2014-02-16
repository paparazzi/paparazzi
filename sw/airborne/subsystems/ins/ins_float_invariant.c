/*
 * Copyright (C) 2012-2013 Jean-Philippe Condomines, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */


/**
 * @file subsystems/ins/ins_float_invariant.c
 * @author Jean-Philippe Condomines <jp.condomines@gmail.com>
 *
 * INS using invariant filter.
 *
 * Only for fixedwing currenctly
 *
 */

#include "subsystems/ins/ins_float_invariant.h"

#include "subsystems/ahrs/ahrs_int_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ahrs.h"

#include "subsystems/ins.h"
#include "subsystems/gps.h"
#include "subsystems/imu.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/nav.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_rk_float.h"
#include "math/pprz_isa.h"

#include "subsystems/abi.h"
#include "state.h"

#include "led.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#if LOG_INVARIANT_FILTER
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
bool_t log_started = FALSE;
#endif

/*------------- =*= Invariant Observers =*= -------------*
 *
 *             State vector :
 *
 *  x = [q0 q1 q2 q3 vx vy vz px py pz wb1 wb2 wb3 hb as]'
 *
 *             Dynamic model (dim = 15) :
 *
 *  x_qdot      = 0.5 * x_quat * ( x_rates - x_bias );
 *  x_Vdot      = A + 1/as (q * am * (q)-1);
 *  x_Xdot      = V;
 *  x_bias_dot  = 0;
 *  x_asdot     = 0;
 *  x_hbdot     = 0;
 *
 *             Observation model (dim = 10):
 *  yv  = V;
 *  yx  = X;
 *  yh  = <X,e3> - hb;
 *  yb  = (q)-1 *B * q;  (B : magnetometers)
 *
 *------------------------------------------------------*/

// Default values for the tuning gains
// Tuning parameter of speed error on attitude (e-2)
#ifndef INS_INV_LV
#define INS_INV_LV 2.
#endif
// Tuning parameter of mag error on attitude (e-2)
#ifndef INS_INV_LB
#define INS_INV_LB 6.
#endif
// Tuning parameter of horizontal speed error on speed
#ifndef INS_INV_MV
#define INS_INV_MV 8.
#endif
// Tuning parameter of vertical speed error on speed
#ifndef INS_INV_MVZ
#define INS_INV_MVZ 15.
#endif
// Tuning parameter of baro error on vertical speed
#ifndef INS_INV_MH
#define INS_INV_MH 0.2
#endif
// Tuning parameter of horizontal position error on position
#ifndef INS_INV_NX
#define INS_INV_NX 0.8
#endif
// Tuning parameter of vertical position error on position
#ifndef INS_INV_NXZ
#define INS_INV_NXZ 0.5
#endif
// Tuning parameter of baro error on vertical position
#ifndef INS_INV_NH
#define INS_INV_NH 1.2
#endif
// Tuning parameter of speed error on gyro biases (e-3)
#ifndef INS_INV_OV
#define INS_INV_OV 1.2
#endif
// Tuning parameter of mag error on gyro biases (e-3)
#ifndef INS_INV_OB
#define INS_INV_OB 1.
#endif
// Tuning parameter of speed error on accel biases (e-2)
#ifndef INS_INV_RV
#define INS_INV_RV 4.
#endif
// Tuning parameter of baro error on accel biases (vertical projection) (e-8)
#ifndef INS_INV_RH
#define INS_INV_RH 8.
#endif
// Tuning parameter of baro error on baro bias
#ifndef INS_INV_SH
#define INS_INV_SH 0.01
#endif


// FIXME this is still needed for fixedwing integration
#if INS_UPDATE_FW_ESTIMATOR
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0.
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0.
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
#endif

struct InsFloatInv ins_impl;

/* integration time step */
static const float dt = (1./ ((float)AHRS_PROPAGATE_FREQUENCY));

/* earth gravity model */
static const struct FloatVect3 A = { 0.f, 0.f, 9.81f };

/* earth magnetic model */
static const struct FloatVect3 B = { (float)(INS_H_X), (float)(INS_H_Y), (float)(INS_H_Z) };

/* barometer */
bool_t ins_baro_initialised;
// Baro event on ABI
#ifndef INS_BARO_ID
#define INS_BARO_ID BARO_BOARD_SENDER_ID
#endif
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, const float *pressure);

/* error computation */
static inline void error_output(struct InsFloatInv * _ins);

/* propagation model (called by runge-kutta library) */
static inline void invariant_model(float * o, const float * x, const int n, const float * u, const int m);

/* init state and measurements */
static inline void init_invariant_state(void) {
  // init state
  FLOAT_QUAT_ZERO(ins_impl.state.quat);
  FLOAT_RATES_ZERO(ins_impl.state.bias);
  FLOAT_VECT3_ZERO(ins_impl.state.pos);
  FLOAT_VECT3_ZERO(ins_impl.state.speed);
  ins_impl.state.as = 1.;
  ins_impl.state.hb = 0.;

  // init measures
  FLOAT_VECT3_ZERO(ins_impl.meas.pos_gps);
  FLOAT_VECT3_ZERO(ins_impl.meas.speed_gps);
  ins_impl.meas.baro_alt = 0.;

  // init baro
  ins_baro_initialised = FALSE;
}

void ins_init() {

  // init position
#if INS_UPDATE_FW_ESTIMATOR
  struct UtmCoor_f utm0;
  utm0.north = (float)nav_utm_north0;
  utm0.east = (float)nav_utm_east0;
  utm0.alt = GROUND_ALT;
  utm0.zone = nav_utm_zone0;
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);
#else
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav0.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);
  ins_impl.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ltp_def);
#endif

  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);

  // init state and measurements
  init_invariant_state();

  // init gains
  ins_impl.gains.lv   = INS_INV_LV;
  ins_impl.gains.lb   = INS_INV_LB;
  ins_impl.gains.mv   = INS_INV_MV;
  ins_impl.gains.mvz  = INS_INV_MVZ;
  ins_impl.gains.mh   = INS_INV_MH;
  ins_impl.gains.nx   = INS_INV_NX;
  ins_impl.gains.nxz  = INS_INV_NXZ;
  ins_impl.gains.nh   = INS_INV_NH;
  ins_impl.gains.ov   = INS_INV_OV;
  ins_impl.gains.ob   = INS_INV_OB;
  ins_impl.gains.rv   = INS_INV_RV;
  ins_impl.gains.rh   = INS_INV_RH;
  ins_impl.gains.sh   = INS_INV_SH;

  ins.status = INS_UNINIT;
  ins.hf_realign = FALSE;
  ins.vf_realign = FALSE;

}

void ins_periodic(void) {}

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;
}


void ahrs_align(void)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ins_impl.state.quat, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);

  /* use average gyro as initial value for bias */
  struct FloatRates bias0;
  RATES_COPY(bias0, ahrs_aligner.lp_gyro);
  RATES_FLOAT_OF_BFP(ins_impl.state.bias, bias0);

  // ins and ahrs are now running
  ahrs.status = AHRS_RUNNING;
  ins.status = INS_RUNNING;
}

void ahrs_propagate(void) {
  struct FloatRates body_rates;
  struct FloatEulers eulers;

  // realign all the filter if needed
  // a complete init cycle is required
  if (ins.hf_realign || ins.vf_realign) {
    ins.status = INS_UNINIT;
    ahrs.status = AHRS_UNINIT;
    init_invariant_state();
    ins.hf_realign = FALSE;
    ins.vf_realign = FALSE;
  }

  // fill command vector
  RATES_FLOAT_OF_BFP(ins_impl.cmd.rates, imu.gyro);
  ACCELS_FLOAT_OF_BFP(ins_impl.cmd.accel, imu.accel);

  // update correction gains
  error_output(&ins_impl);

  // propagate model
  struct inv_state new_state;
  runge_kutta_4_float((float*)&new_state/*(float*)&ins_impl.state*/,
      (float*)&ins_impl.state, INV_STATE_DIM,
      (float*)&ins_impl.cmd, INV_COMMAND_DIM,
      invariant_model, dt);
  ins_impl.state = new_state;

  // normalize quaternion
  FLOAT_QUAT_NORMALIZE(ins_impl.state.quat);

  // set global state
  FLOAT_EULERS_OF_QUAT(eulers, ins_impl.state.quat);
#if INS_UPDATE_FW_ESTIMATOR
  // Some stupid lines of code for neutrals
  eulers.phi -= ins_roll_neutral;
  eulers.theta -= ins_pitch_neutral;
  stateSetNedToBodyEulers_f(&eulers);
#else
  stateSetNedToBodyQuat_f(&ins_impl.state.quat);
#endif
  RATES_DIFF(body_rates, ins_impl.cmd.rates, ins_impl.state.bias);
  stateSetBodyRates_f(&body_rates);
  stateSetPositionNed_f(&ins_impl.state.pos);
  stateSetSpeedNed_f(&ins_impl.state.speed);

  //------------------------------------------------------------//

  RunOnceEvery(3,{
      DOWNLINK_SEND_INV_FILTER(DefaultChannel, DefaultDevice,
        &ins_impl.state.quat.qi,
        &eulers.phi,
        &eulers.theta,
        &eulers.psi,
        &ins_impl.state.speed.x,
        &ins_impl.state.speed.y,
        &ins_impl.state.speed.z,
        &ins_impl.state.pos.x,
        &ins_impl.state.pos.y,
        &ins_impl.state.pos.z,
        &ins_impl.state.bias.p,
        &ins_impl.state.bias.q,
        &ins_impl.state.bias.r,
        &ins_impl.state.as,
        &ins_impl.state.hb,
        &ins_impl.meas.baro_alt,
        &ins_impl.meas.pos_gps.z)
      });

#if LOG_INVARIANT_FILTER
  if (pprzLogFile.fs != NULL) {
    if (!log_started) {
      // log file header
      sdLogWriteLog(&pprzLogFile, "p q r ax ay az gx gy gz gvx gvy gvz mx my mz b qi qx qy qz bp bq br vx vy vz px py pz hb as\n");
      log_started = TRUE;
    }
    else {
      sdLogWriteLog(&pprzLogFile, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
          ins_impl.cmd.rates.p,
          ins_impl.cmd.rates.q,
          ins_impl.cmd.rates.r,
          ins_impl.cmd.accel.x,
          ins_impl.cmd.accel.y,
          ins_impl.cmd.accel.z,
          ins_impl.meas.pos_gps.x,
          ins_impl.meas.pos_gps.y,
          ins_impl.meas.pos_gps.z,
          ins_impl.meas.speed_gps.x,
          ins_impl.meas.speed_gps.y,
          ins_impl.meas.speed_gps.z,
          ins_impl.meas.mag.x,
          ins_impl.meas.mag.y,
          ins_impl.meas.mag.z,
          ins_impl.meas.baro_alt,
          ins_impl.state.quat.qi,
          ins_impl.state.quat.qx,
          ins_impl.state.quat.qy,
          ins_impl.state.quat.qz,
          ins_impl.state.bias.p,
          ins_impl.state.bias.q,
          ins_impl.state.bias.r,
          ins_impl.state.speed.x,
          ins_impl.state.speed.y,
          ins_impl.state.speed.z,
          ins_impl.state.pos.x,
          ins_impl.state.pos.y,
          ins_impl.state.pos.z,
          ins_impl.state.hb,
          ins_impl.state.as);
    }
  }
#endif
}

void ahrs_update_gps(void) {

  if (gps.fix == GPS_FIX_3D && ins.status == INS_RUNNING) {

#if INS_UPDATE_FW_ESTIMATOR
    if (state.utm_initialized_f) {
      // position (local ned)
      ins_impl.meas.pos_gps.x = (gps.utm_pos.north / 100.) - state.utm_origin_f.north;
      ins_impl.meas.pos_gps.y = (gps.utm_pos.east / 100.) - state.utm_origin_f.east;
      ins_impl.meas.pos_gps.z = state.utm_origin_f.alt - (gps.hmsl / 1000.);
      // speed
      ins_impl.meas.speed_gps.x = gps.ned_vel.x / 100.;
      ins_impl.meas.speed_gps.y = gps.ned_vel.y / 100.;
      ins_impl.meas.speed_gps.z = gps.ned_vel.z / 100.;
    }
#else
    if (state.ned_initialized_f) {
      struct NedCoor_f gps_pos_cm_ned;
      ned_of_ecef_point_f(&gps_pos_cm_ned, &state.ned_origin_f, &gps.ecef_pos);
      VECT3_SDIV(ins_impl.meas.pos_gps, gps_pos_m_ned, 100.);
      struct NedCoor_f gps_speed_cm_s_ned;
      ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &state.ned_origin_f, &gps.ecef_vel);
      VECT3_SDIV(ins_impl.meas.speed_gps, gps_speed_cm_s_ned, 100.);
    }
#endif
  }

}

void ins_update_gps(void) {}

void ins_update_baro(void) {}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure) {
  static float ins_qfe = 101325.0;
  static float alpha = 10.;
  static int32_t i = 1;
  static float baro_moy = 0.;
  static float baro_prev = 0.;

  if (!ins_baro_initialised) {
    // try to find a stable qfe
    // TODO generic function in pprz_isa ?
    if (i == 1) {
      baro_moy = *pressure;
      baro_prev = *pressure;
    }
    baro_moy = (baro_moy*(i-1) + *pressure)/i;
    alpha = (10.*alpha + (baro_moy-baro_prev))/(11.);
    baro_prev = baro_moy;
    // test stop condition
    if (fabs(alpha) < 0.005) {
      ins_qfe = baro_moy;
      ins_baro_initialised = TRUE;
    }
    if (i == 250) {
      ins_qfe = *pressure;
      ins_baro_initialised = TRUE;
    }
    i++;
  }
  else { /* normal update with baro measurement */
    ins_impl.meas.baro_alt = -pprz_isa_height_of_pressure(*pressure, ins_qfe); // Z down
  }
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
  MAGS_FLOAT_OF_BFP(ins_impl.meas.mag, imu.mag);
}


/** Compute dynamic mode
 *
 * x_dot = evolution_model + (gain_matrix * error)
 */
static inline void invariant_model(float * o, const float * x, const int n, const float * u, const int m __attribute__((unused))) {

  const struct inv_state * s = (struct inv_state *)x;
  const struct inv_command * c = (struct inv_command *)u;
  struct inv_state s_dot;
  struct FloatRates rates;
  struct FloatVect3 tmp_vect;
  struct FloatQuat tmp_quat;
  float  norm;

  // test accel sensitivity
  if (fabs(s->as) < 0.1) {
    // too small, return x_dot = 0 to avoid division by 0
    float_vect_zero(o, n);
    // TODO set ins state to error
    return;
  }

  /* dot_q = 0.5 * q * (x_rates - x_bias) + LE * q + (1 - ||q||^2) * q */
  RATES_DIFF(rates, c->rates, s->bias);
  FLOAT_VECT3_ASSIGN(tmp_vect, rates.p, rates.q, rates.r);
  FLOAT_QUAT_VMUL_LEFT(s_dot.quat, s->quat, tmp_vect);
  FLOAT_QUAT_SMUL(s_dot.quat, s_dot.quat, 0.5);

  FLOAT_QUAT_VMUL_RIGHT(tmp_quat, s->quat, ins_impl.corr.LE);
  FLOAT_QUAT_ADD(s_dot.quat, tmp_quat);

  norm = FLOAT_QUAT_NORM(s->quat);
  norm = 1. - (norm*norm);
  FLOAT_QUAT_SMUL(tmp_quat, s->quat, norm);
  FLOAT_QUAT_ADD(s_dot.quat, tmp_quat);

  /* dot_V = A + (1/as) * (q * am * q-1) + ME */
  FLOAT_QUAT_RMAT_B2N(s_dot.speed, s->quat, c->accel);
  FLOAT_VECT3_SMUL(s_dot.speed, s_dot.speed, 1. / (s->as));
  FLOAT_VECT3_ADD(s_dot.speed, A);
  FLOAT_VECT3_ADD(s_dot.speed, ins_impl.corr.ME);

  /* dot_X = V + NE */
  FLOAT_VECT3_SUM(s_dot.pos, s->speed, ins_impl.corr.NE);

  /* bias_dot = q-1 * (OE) * q */
  FLOAT_QUAT_RMAT_N2B(tmp_vect, s->quat, ins_impl.corr.OE);
  RATES_ASSIGN(s_dot.bias, tmp_vect.x, tmp_vect.y, tmp_vect.z);

  /* as_dot = as * RE */
  s_dot.as = (s->as) * (ins_impl.corr.RE);

  /* hb_dot = SE */
  s_dot.hb = ins_impl.corr.SE;

  // set output
  memcpy(o, &s_dot, n*sizeof(float));
}

/** Compute correction vectors
 * E = ( ŷ - y )
 * LE, ME, NE, OE : ( gain matrix * error )
 */
static inline void error_output(struct InsFloatInv * _ins) {

  struct FloatVect3 YBt, I, Ev, Eb, Ex, Itemp, Ebtemp, Evtemp;
  float Eh;
  float temp;

  // test accel sensitivity
  if (fabs(_ins->state.as) < 0.1) {
    // too small, don't do anything to avoid division by 0
    return;
  }

  /* YBt = q * yB * q-1  */
  FLOAT_QUAT_RMAT_B2N(YBt, _ins->state.quat, _ins->meas.mag);

  FLOAT_QUAT_RMAT_B2N(I, _ins->state.quat, _ins->cmd.accel);
  FLOAT_VECT3_SMUL(I, I, 1. / (_ins->state.as));

  /*--------- E = ( ŷ - y ) ----------*/
  /* Eb = ( B - YBt ) */
  FLOAT_VECT3_DIFF(Eb, B, YBt);

  // pos and speed error only if GPS data are valid
  if (gps.fix == GPS_FIX_3D && ins.status == INS_RUNNING
#if INS_UPDATE_FW_ESTIMATOR
    && state.utm_initialized_f
#else
    && state.ned_initialized_f
#endif
    ) {
    /* Ev = (V - YV)   */
    FLOAT_VECT3_DIFF(Ev, _ins->state.speed, _ins->meas.speed_gps);
    /* Ex = (X - YX)  */
    FLOAT_VECT3_DIFF(Ex, _ins->state.pos, _ins->meas.pos_gps);
  }
  else {
    FLOAT_VECT3_ZERO(Ev);
    FLOAT_VECT3_ZERO(Ex);
  }
  /* Eh = < X,e3 > - hb - YH */
  Eh = _ins->state.pos.z - _ins->state.hb - _ins->meas.baro_alt;

  /*--------------Gains--------------*/

  /**** LvEv + LbEb = -lvIa x Ev +  lb < B x Eb, Ia > Ia *****/
  FLOAT_VECT3_SMUL(Itemp, I, -_ins->gains.lv/100.);
  FLOAT_VECT3_CROSS_PRODUCT(Evtemp, Itemp, Ev);

  FLOAT_VECT3_CROSS_PRODUCT(Ebtemp, B, Eb);
  temp = FLOAT_VECT3_DOT_PRODUCT(Ebtemp, I);
  temp = (_ins->gains.lb/100.) * temp;

  FLOAT_VECT3_SMUL(Ebtemp, I, temp);
  FLOAT_VECT3_ADD(Evtemp, Ebtemp);
  FLOAT_VECT3_COPY(_ins->corr.LE, Evtemp);

  /***** MvEv + MhEh = -mv * Ev + (-mh * <Eh,e3>)********/
  _ins->corr.ME.x = (-_ins->gains.mv) * Ev.x + 0.;
  _ins->corr.ME.y = (-_ins->gains.mv) * Ev.y + 0.;
  _ins->corr.ME.z = ((-_ins->gains.mvz) * Ev.z) + ((-_ins->gains.mh) * Eh);

  /****** NxEx + NhEh = -nx * Ex + (-nh * <Eh, e3>) ********/
  _ins->corr.NE.x = (-_ins->gains.nx) * Ex.x + 0.;
  _ins->corr.NE.y = (-_ins->gains.nx) * Ex.y + 0.;
  _ins->corr.NE.z = ((-_ins->gains.nxz) * Ex.z) + ((-_ins->gains.nh) * Eh);

  /****** OvEv + ObEb = ovIa x Ev - ob < B x Eb, Ia > Ia ********/
  FLOAT_VECT3_SMUL(Itemp, I, _ins->gains.ov/1000.);
  FLOAT_VECT3_CROSS_PRODUCT(Evtemp, Itemp, Ev);

  FLOAT_VECT3_CROSS_PRODUCT(Ebtemp, B, Eb);
  temp = FLOAT_VECT3_DOT_PRODUCT(Ebtemp, I);
  temp = (-_ins->gains.ob/1000.) * temp;

  FLOAT_VECT3_SMUL(Ebtemp, I, temp);
  FLOAT_VECT3_ADD(Evtemp, Ebtemp);
  FLOAT_VECT3_COPY(_ins->corr.OE, Evtemp);

  /* a scalar */
  /****** RvEv + RhEh = rv < Ia, Ev > + (-rhEh) **************/
  _ins->corr.RE = ((_ins->gains.rv/100.) * FLOAT_VECT3_DOT_PRODUCT(Ev, I)) + ((-_ins->gains.rh/10000.) * Eh);

  /****** ShEh ******/
  _ins->corr.SE = (_ins->gains.sh) * Eh;

}


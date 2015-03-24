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
 */

/**
 * @file subsystems/ins/ins_float_invariant.c
 * @author Jean-Philippe Condomines <jp.condomines@gmail.com>
 *
 * INS using invariant filter.
 *
 */

#include "subsystems/ins/ins_float_invariant.h"

#include "subsystems/ahrs/ahrs_int_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "subsystems/ins.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#if INS_FINV_USE_UTM
#include "firmwares/fixedwing/nav.h"
#endif

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_rk_float.h"
#include "math/pprz_isa.h"

#include "state.h"

// for debugging
#if SEND_INVARIANT_FILTER
#include "subsystems/datalink/telemetry.h"
#endif

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


struct InsFloatInv ins_float_inv;

/* earth gravity model */
static const struct FloatVect3 A = { 0.f, 0.f, 9.81f };

/* earth magnetic model */
//static const struct FloatVect3 B = { (float)(INS_H_X), (float)(INS_H_Y), (float)(INS_H_Z) };
#define B ins_float_inv.mag_h

/* barometer */
bool_t ins_baro_initialized;

/* gps */
bool_t ins_gps_fix_once;

/* error computation */
static inline void error_output(struct InsFloatInv *_ins);

/* propagation model (called by runge-kutta library) */
static inline void invariant_model(float *o, const float *x, const int n, const float *u, const int m);


/** Right multiplication by a quaternion.
 * vi * q
 */
void float_quat_vmul_right(struct FloatQuat *mright, const struct FloatQuat *q,
                           struct FloatVect3 *vi);


/* init state and measurements */
static inline void init_invariant_state(void)
{
  // init state
  float_quat_identity(&ins_float_inv.state.quat);
  FLOAT_RATES_ZERO(ins_float_inv.state.bias);
  FLOAT_VECT3_ZERO(ins_float_inv.state.pos);
  FLOAT_VECT3_ZERO(ins_float_inv.state.speed);
  ins_float_inv.state.as = 1.0f;
  ins_float_inv.state.hb = 0.0f;

  // init measures
  FLOAT_VECT3_ZERO(ins_float_inv.meas.pos_gps);
  FLOAT_VECT3_ZERO(ins_float_inv.meas.speed_gps);
  ins_float_inv.meas.baro_alt = 0.0f;

  // init baro
  ins_baro_initialized = FALSE;
  ins_gps_fix_once = FALSE;
}

void ins_float_invariant_init(void)
{

  // init position
#if INS_FINV_USE_UTM
  struct UtmCoor_f utm0;
  utm0.north = (float)nav_utm_north0;
  utm0.east = (float)nav_utm_east0;
  utm0.alt = GROUND_ALT;
  utm0.zone = nav_utm_zone0;
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);
#else
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);
  ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ltp_def);
#endif

  B.x = INS_H_X;
  B.y = INS_H_Y;
  B.z = INS_H_Z;

  // init state and measurements
  init_invariant_state();

  // init gains
  ins_float_inv.gains.lv   = INS_INV_LV;
  ins_float_inv.gains.lb   = INS_INV_LB;
  ins_float_inv.gains.mv   = INS_INV_MV;
  ins_float_inv.gains.mvz  = INS_INV_MVZ;
  ins_float_inv.gains.mh   = INS_INV_MH;
  ins_float_inv.gains.nx   = INS_INV_NX;
  ins_float_inv.gains.nxz  = INS_INV_NXZ;
  ins_float_inv.gains.nh   = INS_INV_NH;
  ins_float_inv.gains.ov   = INS_INV_OV;
  ins_float_inv.gains.ob   = INS_INV_OB;
  ins_float_inv.gains.rv   = INS_INV_RV;
  ins_float_inv.gains.rh   = INS_INV_RH;
  ins_float_inv.gains.sh   = INS_INV_SH;

  ins_float_inv.is_aligned = FALSE;
  ins_float_inv.reset = FALSE;
}


void ins_reset_local_origin(void)
{
#if INS_FINV_USE_UTM
  struct UtmCoor_f utm;
#ifdef GPS_USE_LATLONG
  /* Recompute UTM coordinates in this zone */
  struct LlaCoor_f lla;
  LLA_FLOAT_OF_BFP(lla, gps.lla_pos);
  utm.zone = (gps.lla_pos.lon / 1e7 + 180) / 6 + 1;
  utm_of_lla_f(&utm, &lla);
#else
  utm.zone = gps.utm_pos.zone;
  utm.east = gps.utm_pos.east / 100.0f;
  utm.north = gps.utm_pos.north / 100.0f;
#endif
  // ground_alt
  utm.alt = gps.hmsl / 1000.0f;
  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);
#else
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &gps.ecef_pos);
  ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ltp_def);
#endif
}

void ins_reset_altitude_ref(void)
{
#if INS_FINV_USE_UTM
  struct UtmCoor_f utm = state.utm_origin_f;
  utm.alt = gps.hmsl / 1000.0f;
  stateSetLocalUtmOrigin_f(&utm);
#else
  struct LlaCoor_i lla = {
    .lat = state.ned_origin_i.lla.lat,
    .lon = state.ned_origin_i.lla.lon,
    .alt = gps.lla_pos.alt
  };
  struct LtpDef_i ltp_def;
  ltp_def_from_lla_i(&ltp_def, &lla);
  ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ltp_def);
#endif
}

void ins_float_invariant_align(struct Int32Rates *lp_gyro,
                               struct Int32Vect3 *lp_accel,
                               struct Int32Vect3 *lp_mag)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ins_float_inv.state.quat, lp_accel, lp_mag);

  /* use average gyro as initial value for bias */
  struct FloatRates bias0;
  RATES_COPY(bias0, *lp_gyro);
  RATES_FLOAT_OF_BFP(ins_float_inv.state.bias, bias0);

  /* push initial values to state interface */
  stateSetNedToBodyQuat_f(&ins_float_inv.state.quat);

  // ins and ahrs are now running
  ins_float_inv.is_aligned = TRUE;
}

void ins_float_invariant_propagate(struct Int32Rates* gyro, struct Int32Vect3* accel, float dt)
{
  struct FloatRates body_rates;

  // realign all the filter if needed
  // a complete init cycle is required
  if (ins_float_inv.reset) {
    ins_float_inv.reset = FALSE;
    ins_float_inv.is_aligned = FALSE;
    init_invariant_state();
  }

  // fill command vector
  struct Int32Rates gyro_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ins_float_inv.body_to_imu);
  int32_rmat_transp_ratemult(&gyro_meas_body, body_to_imu_rmat, gyro);
  RATES_FLOAT_OF_BFP(ins_float_inv.cmd.rates, gyro_meas_body);
  struct Int32Vect3 accel_meas_body;
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  ACCELS_FLOAT_OF_BFP(ins_float_inv.cmd.accel, accel_meas_body);

  // update correction gains
  error_output(&ins_float_inv);

  // propagate model
  struct inv_state new_state;
  runge_kutta_4_float((float *)&new_state,
                      (float *)&ins_float_inv.state, INV_STATE_DIM,
                      (float *)&ins_float_inv.cmd, INV_COMMAND_DIM,
                      invariant_model, dt);
  ins_float_inv.state = new_state;

  // normalize quaternion
  FLOAT_QUAT_NORMALIZE(ins_float_inv.state.quat);

  // set global state
  stateSetNedToBodyQuat_f(&ins_float_inv.state.quat);
  RATES_DIFF(body_rates, ins_float_inv.cmd.rates, ins_float_inv.state.bias);
  stateSetBodyRates_f(&body_rates);
  stateSetPositionNed_f(&ins_float_inv.state.pos);
  stateSetSpeedNed_f(&ins_float_inv.state.speed);
  // untilt accel and remove gravity
  struct FloatQuat q_b2n;
  float_quat_invert(&q_b2n, &ins_float_inv.state.quat);
  struct FloatVect3 accel_n;
  float_quat_vmult(&accel_n, &q_b2n, &ins_float_inv.cmd.accel);
  VECT3_SMUL(accel_n, accel_n, 1. / (ins_float_inv.state.as));
  VECT3_ADD(accel_n, A);
  stateSetAccelNed_f((struct NedCoor_f *)&accel_n);

  //------------------------------------------------------------//

#if SEND_INVARIANT_FILTER
  struct FloatEulers eulers;
  FLOAT_EULERS_OF_QUAT(eulers, ins_float_inv.state.quat);
  RunOnceEvery(3,
               pprz_msg_send_INV_FILTER(&(DefaultChannel).trans_tx, &(DefaultDevice).device,
                                        AC_ID,
                                        &ins_float_inv.state.quat.qi,
                                        &eulers.phi,
                                        &eulers.theta,
                                        &eulers.psi,
                                        &ins_float_inv.state.speed.x,
                                        &ins_float_inv.state.speed.y,
                                        &ins_float_inv.state.speed.z,
                                        &ins_float_inv.state.pos.x,
                                        &ins_float_inv.state.pos.y,
                                        &ins_float_inv.state.pos.z,
                                        &ins_float_inv.state.bias.p,
                                        &ins_float_inv.state.bias.q,
                                        &ins_float_inv.state.bias.r,
                                        &ins_float_inv.state.as,
                                        &ins_float_inv.state.hb,
                                        &ins_float_inv.meas.baro_alt,
                                        &ins_float_inv.meas.pos_gps.z);
               );
#endif

#if LOG_INVARIANT_FILTER
  if (pprzLogFile.fs != NULL) {
    if (!log_started) {
      // log file header
      sdLogWriteLog(&pprzLogFile,
                    "p q r ax ay az gx gy gz gvx gvy gvz mx my mz b qi qx qy qz bp bq br vx vy vz px py pz hb as\n");
      log_started = TRUE;
    } else {
      sdLogWriteLog(&pprzLogFile,
                    "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    ins_float_inv.cmd.rates.p,
                    ins_float_inv.cmd.rates.q,
                    ins_float_inv.cmd.rates.r,
                    ins_float_inv.cmd.accel.x,
                    ins_float_inv.cmd.accel.y,
                    ins_float_inv.cmd.accel.z,
                    ins_float_inv.meas.pos_gps.x,
                    ins_float_inv.meas.pos_gps.y,
                    ins_float_inv.meas.pos_gps.z,
                    ins_float_inv.meas.speed_gps.x,
                    ins_float_inv.meas.speed_gps.y,
                    ins_float_inv.meas.speed_gps.z,
                    ins_float_inv.meas.mag.x,
                    ins_float_inv.meas.mag.y,
                    ins_float_inv.meas.mag.z,
                    ins_float_inv.meas.baro_alt,
                    ins_float_inv.state.quat.qi,
                    ins_float_inv.state.quat.qx,
                    ins_float_inv.state.quat.qy,
                    ins_float_inv.state.quat.qz,
                    ins_float_inv.state.bias.p,
                    ins_float_inv.state.bias.q,
                    ins_float_inv.state.bias.r,
                    ins_float_inv.state.speed.x,
                    ins_float_inv.state.speed.y,
                    ins_float_inv.state.speed.z,
                    ins_float_inv.state.pos.x,
                    ins_float_inv.state.pos.y,
                    ins_float_inv.state.pos.z,
                    ins_float_inv.state.hb,
                    ins_float_inv.state.as);
    }
  }
#endif
}

void ins_float_invariant_update_gps(struct GpsState *gps_s)
{

  if (gps_s->fix == GPS_FIX_3D && ins_float_inv.is_aligned) {
    ins_gps_fix_once = TRUE;

#if INS_FINV_USE_UTM
    if (state.utm_initialized_f) {
      // position (local ned)
      ins_float_inv.meas.pos_gps.x = (gps_s->utm_pos.north / 100.0f) - state.utm_origin_f.north;
      ins_float_inv.meas.pos_gps.y = (gps_s->utm_pos.east / 100.0f) - state.utm_origin_f.east;
      ins_float_inv.meas.pos_gps.z = state.utm_origin_f.alt - (gps_s->hmsl / 1000.0f);
      // speed
      ins_float_inv.meas.speed_gps.x = gps_s->ned_vel.x / 100.0f;
      ins_float_inv.meas.speed_gps.y = gps_s->ned_vel.y / 100.0f;
      ins_float_inv.meas.speed_gps.z = gps_s->ned_vel.z / 100.0f;
    }
#else
    if (state.ned_initialized_f) {
      struct EcefCoor_f ecef_pos, ecef_vel;
      ECEF_FLOAT_OF_BFP(ecef_pos, gps_s->ecef_pos);
      ned_of_ecef_point_f(&ins_float_inv.meas.pos_gps, &state.ned_origin_f, &ecef_pos);
      ECEF_FLOAT_OF_BFP(ecef_vel, gps_s->ecef_vel);
      ned_of_ecef_vect_f(&ins_float_inv.meas.speed_gps, &state.ned_origin_f, &ecef_vel);
    }
#endif
  }

}


void ins_float_invariant_update_baro(float pressure)
{
  static float ins_qfe = 101325.0f;
  static float alpha = 10.0f;
  static int32_t i = 1;
  static float baro_moy = 0.0f;
  static float baro_prev = 0.0f;

  if (!ins_baro_initialized) {
    // try to find a stable qfe
    // TODO generic function in pprz_isa ?
    if (i == 1) {
      baro_moy = pressure;
      baro_prev = pressure;
    }
    baro_moy = (baro_moy * (i - 1) + pressure) / i;
    alpha = (10.*alpha + (baro_moy - baro_prev)) / (11.0f);
    baro_prev = baro_moy;
    // test stop condition
    if (fabs(alpha) < 0.005f) {
      ins_qfe = baro_moy;
      ins_baro_initialized = TRUE;
    }
    if (i == 250) {
      ins_qfe = pressure;
      ins_baro_initialized = TRUE;
    }
    i++;
  } else { /* normal update with baro measurement */
    ins_float_inv.meas.baro_alt = -pprz_isa_height_of_pressure(pressure, ins_qfe); // Z down
  }
}

// assume mag is dead when values are not moving anymore
#define MAG_FROZEN_COUNT 30

void ins_float_invariant_update_mag(struct Int32Vect3* mag)
{
  static uint32_t mag_frozen_count = MAG_FROZEN_COUNT;
  static int32_t last_mx = 0;

  if (last_mx == mag->x) {
    mag_frozen_count--;
    if (mag_frozen_count == 0) {
      // if mag is dead, better set measurements to zero
      FLOAT_VECT3_ZERO(ins_float_inv.meas.mag);
      mag_frozen_count = MAG_FROZEN_COUNT;
    }
  } else {
    // values are moving
    struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ins_float_inv.body_to_imu);
    struct Int32Vect3 mag_meas_body;
    // new values in body frame
    int32_rmat_transp_vmult(&mag_meas_body, body_to_imu_rmat, mag);
    MAGS_FLOAT_OF_BFP(ins_float_inv.meas.mag, mag_meas_body);
    // reset counter
    mag_frozen_count = MAG_FROZEN_COUNT;
  }
  last_mx = mag->x;
}


/** Compute dynamic mode
 *
 * x_dot = evolution_model + (gain_matrix * error)
 */
static inline void invariant_model(float *o, const float *x, const int n, const float *u,
                                   const int m __attribute__((unused)))
{

#pragma GCC diagnostic push // require GCC 4.6
#pragma GCC diagnostic ignored "-Wcast-qual"
  struct inv_state *s = (struct inv_state *)x;
  struct inv_command *c = (struct inv_command *)u;
#pragma GCC diagnostic pop // require GCC 4.6
  struct inv_state s_dot;
  struct FloatRates rates_unbiased;
  struct FloatVect3 tmp_vect;
  struct FloatQuat tmp_quat;

  // test accel sensitivity
  if (fabs(s->as) < 0.1) {
    // too small, return x_dot = 0 to avoid division by 0
    float_vect_zero(o, n);
    // TODO set ins state to error
    return;
  }

  /* dot_q = 0.5 * q * (x_rates - x_bias) + LE * q + (1 - ||q||^2) * q */
  RATES_DIFF(rates_unbiased, c->rates, s->bias);
  /* qd = 0.5 * q * rates_unbiased = -0.5 * rates_unbiased * q */
  float_quat_derivative(&s_dot.quat, &rates_unbiased, &(s->quat));

  float_quat_vmul_right(&tmp_quat, &(s->quat), &ins_float_inv.corr.LE);
  QUAT_ADD(s_dot.quat, tmp_quat);

  float norm2_r = 1. - FLOAT_QUAT_NORM2(s->quat);
  QUAT_SMUL(tmp_quat, s->quat, norm2_r);
  QUAT_ADD(s_dot.quat, tmp_quat);

  /* dot_V = A + (1/as) * (q * am * q-1) + ME */
  struct FloatQuat q_b2n;
  float_quat_invert(&q_b2n, &(s->quat));
  float_quat_vmult((struct FloatVect3 *)&s_dot.speed, &q_b2n, &(c->accel));
  VECT3_SMUL(s_dot.speed, s_dot.speed, 1. / (s->as));
  VECT3_ADD(s_dot.speed, A);
  VECT3_ADD(s_dot.speed, ins_float_inv.corr.ME);

  /* dot_X = V + NE */
  VECT3_SUM(s_dot.pos, s->speed, ins_float_inv.corr.NE);

  /* bias_dot = q-1 * (OE) * q */
  float_quat_vmult(&tmp_vect, &(s->quat), &ins_float_inv.corr.OE);
  RATES_ASSIGN(s_dot.bias, tmp_vect.x, tmp_vect.y, tmp_vect.z);

  /* as_dot = as * RE */
  s_dot.as = (s->as) * (ins_float_inv.corr.RE);

  /* hb_dot = SE */
  s_dot.hb = ins_float_inv.corr.SE;

  // set output
  memcpy(o, &s_dot, n * sizeof(float));
}

/** Compute correction vectors
 * E = ( ŷ - y )
 * LE, ME, NE, OE : ( gain matrix * error )
 */
static inline void error_output(struct InsFloatInv *_ins)
{

  struct FloatVect3 YBt, I, Ev, Eb, Ex, Itemp, Ebtemp, Evtemp;
  float Eh;
  float temp;

  // test accel sensitivity
  if (fabs(_ins->state.as) < 0.1) {
    // too small, don't do anything to avoid division by 0
    return;
  }

  /* YBt = q * yB * q-1  */
  struct FloatQuat q_b2n;
  float_quat_invert(&q_b2n, &(_ins->state.quat));
  float_quat_vmult(&YBt, &q_b2n, &(_ins->meas.mag));

  float_quat_vmult(&I, &q_b2n, &(_ins->cmd.accel));
  VECT3_SMUL(I, I, 1. / (_ins->state.as));

  /*--------- E = ( ŷ - y ) ----------*/
  /* Eb = ( B - YBt ) */
  VECT3_DIFF(Eb, B, YBt);

  // pos and speed error only if GPS data are valid
  // or while waiting first GPS data to prevent diverging
  if ((gps.fix == GPS_FIX_3D && ins_float_inv.is_aligned
#if INS_FINV_USE_UTM
       && state.utm_initialized_f
#else
       && state.ned_initialized_f
#endif
      ) || !ins_gps_fix_once) {
    /* Ev = (V - YV)   */
    VECT3_DIFF(Ev, _ins->state.speed, _ins->meas.speed_gps);
    /* Ex = (X - YX)  */
    VECT3_DIFF(Ex, _ins->state.pos, _ins->meas.pos_gps);
  } else {
    FLOAT_VECT3_ZERO(Ev);
    FLOAT_VECT3_ZERO(Ex);
  }
  /* Eh = < X,e3 > - hb - YH */
  Eh = _ins->state.pos.z - _ins->state.hb - _ins->meas.baro_alt;

  /*--------------Gains--------------*/

  /**** LvEv + LbEb = -lvIa x Ev +  lb < B x Eb, Ia > Ia *****/
  VECT3_SMUL(Itemp, I, -_ins->gains.lv / 100.);
  VECT3_CROSS_PRODUCT(Evtemp, Itemp, Ev);

  VECT3_CROSS_PRODUCT(Ebtemp, B, Eb);
  temp = VECT3_DOT_PRODUCT(Ebtemp, I);
  temp = (_ins->gains.lb / 100.) * temp;

  VECT3_SMUL(Ebtemp, I, temp);
  VECT3_ADD(Evtemp, Ebtemp);
  VECT3_COPY(_ins->corr.LE, Evtemp);

  /***** MvEv + MhEh = -mv * Ev + (-mh * <Eh,e3>)********/
  _ins->corr.ME.x = (-_ins->gains.mv) * Ev.x + 0.;
  _ins->corr.ME.y = (-_ins->gains.mv) * Ev.y + 0.;
  _ins->corr.ME.z = ((-_ins->gains.mvz) * Ev.z) + ((-_ins->gains.mh) * Eh);

  /****** NxEx + NhEh = -nx * Ex + (-nh * <Eh, e3>) ********/
  _ins->corr.NE.x = (-_ins->gains.nx) * Ex.x + 0.;
  _ins->corr.NE.y = (-_ins->gains.nx) * Ex.y + 0.;
  _ins->corr.NE.z = ((-_ins->gains.nxz) * Ex.z) + ((-_ins->gains.nh) * Eh);

  /****** OvEv + ObEb = ovIa x Ev - ob < B x Eb, Ia > Ia ********/
  VECT3_SMUL(Itemp, I, _ins->gains.ov / 1000.);
  VECT3_CROSS_PRODUCT(Evtemp, Itemp, Ev);

  VECT3_CROSS_PRODUCT(Ebtemp, B, Eb);
  temp = VECT3_DOT_PRODUCT(Ebtemp, I);
  temp = (-_ins->gains.ob / 1000.) * temp;

  VECT3_SMUL(Ebtemp, I, temp);
  VECT3_ADD(Evtemp, Ebtemp);
  VECT3_COPY(_ins->corr.OE, Evtemp);

  /* a scalar */
  /****** RvEv + RhEh = rv < Ia, Ev > + (-rhEh) **************/
  _ins->corr.RE = ((_ins->gains.rv / 100.) * VECT3_DOT_PRODUCT(Ev, I)) + ((-_ins->gains.rh / 10000.) * Eh);

  /****** ShEh ******/
  _ins->corr.SE = (_ins->gains.sh) * Eh;

}


void float_quat_vmul_right(struct FloatQuat *mright, const struct FloatQuat *q,
                           struct FloatVect3 *vi)
{
  struct FloatVect3 qvec, v1, v2;
  float qi;

  FLOAT_QUAT_EXTRACT(qvec, *q);
  qi = - VECT3_DOT_PRODUCT(*vi, qvec);
  VECT3_CROSS_PRODUCT(v1, *vi, qvec);
  VECT3_SMUL(v2, *vi, q->qi);
  VECT3_ADD(v2, v1);
  QUAT_ASSIGN(*mright, qi, v2.x, v2.y, v2.z);
}

void ins_float_inv_set_body_to_imu_quat(struct FloatQuat *q_b2i)
{
  orientationSetQuat_f(&ins_float_inv.body_to_imu, q_b2i);

  if (!ins_float_inv.is_aligned) {
    /* Set ltp_to_imu so that body is zero */
    memcpy(&ins_float_inv.state.quat, q_b2i, sizeof(struct FloatQuat));
  }
}

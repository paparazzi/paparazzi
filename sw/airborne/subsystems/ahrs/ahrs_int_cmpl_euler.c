/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_int_cmpl_euler.c
 *
 * Complementary filter in euler representation (fixed-point).
 *
 * Estimate the attitude, heading and gyro bias.
 *
 */

#include "ahrs_int_cmpl_euler.h"

#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

#ifndef FACE_REINJ_1
#define FACE_REINJ_1 1024
#endif

#ifndef AHRS_MAG_OFFSET
#define AHRS_MAG_OFFSET 0.
#endif

#ifdef AHRS_UPDATE_FW_ESTIMATOR
// remotely settable (for FW)
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
#endif


struct AhrsIntCmplEuler ahrs_impl;

static inline void get_phi_theta_measurement_fom_accel(int32_t* phi_meas, int32_t* theta_meas, struct Int32Vect3 accel);
static inline void get_psi_measurement_from_mag(int32_t* psi_meas, int32_t phi_est, int32_t theta_est, struct Int32Vect3 mag);
static inline void set_body_state_from_euler(void);

#define F_UPDATE 512

#define PI_INTEG_EULER     (INT32_ANGLE_PI * F_UPDATE)
#define TWO_PI_INTEG_EULER (INT32_ANGLE_2_PI * F_UPDATE)
#define INTEG_EULER_NORMALIZE(_a) {                         \
    while (_a >  PI_INTEG_EULER)  _a -= TWO_PI_INTEG_EULER; \
    while (_a < -PI_INTEG_EULER)  _a += TWO_PI_INTEG_EULER; \
  }

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_filter(void) {
  DOWNLINK_SEND_FILTER(DefaultChannel, DefaultDevice,
      &ahrs_impl.ltp_to_imu_euler.phi,
      &ahrs_impl.ltp_to_imu_euler.theta,
      &ahrs_impl.ltp_to_imu_euler.psi,
      &ahrs_impl.measure.phi,
      &ahrs_impl.measure.theta,
      &ahrs_impl.measure.psi,
      &ahrs_impl.hi_res_euler.phi,
      &ahrs_impl.hi_res_euler.theta,
      &ahrs_impl.hi_res_euler.psi,
      &ahrs_impl.residual.phi,
      &ahrs_impl.residual.theta,
      &ahrs_impl.residual.psi,
      &ahrs_impl.gyro_bias.p,
      &ahrs_impl.gyro_bias.q,
      &ahrs_impl.gyro_bias.r);
}

static void send_euler(void) {
  struct Int32Eulers* eulers = stateGetNedToBodyEulers_i();
  DOWNLINK_SEND_AHRS_EULER_INT(DefaultChannel, DefaultDevice,
      &ahrs_impl.ltp_to_imu_euler.phi,
      &ahrs_impl.ltp_to_imu_euler.theta,
      &ahrs_impl.ltp_to_imu_euler.psi,
      &(eulers->phi),
      &(eulers->theta),
      &(eulers->psi));
}

static void send_bias(void) {
  DOWNLINK_SEND_AHRS_GYRO_BIAS_INT(DefaultChannel, DefaultDevice,
      &ahrs_impl.gyro_bias.p, &ahrs_impl.gyro_bias.q, &ahrs_impl.gyro_bias.r);
}
#endif

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  /* set ltp_to_imu so that body is zero */
  memcpy(&ahrs_impl.ltp_to_imu_euler, orientationGetEulers_i(&imu.body_to_imu),
         sizeof(struct Int32Eulers));
  INT_RATES_ZERO(ahrs_impl.imu_rate);

  INT_RATES_ZERO(ahrs_impl.gyro_bias);
  ahrs_impl.reinj_1 = FACE_REINJ_1;

  ahrs_impl.mag_offset = AHRS_MAG_OFFSET;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "FILTER", send_filter);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_EULER_INT", send_euler);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_GYRO_BIAS_INT", send_bias);
#endif
}

void ahrs_align(void) {

  get_phi_theta_measurement_fom_accel(&ahrs_impl.hi_res_euler.phi, &ahrs_impl.hi_res_euler.theta, ahrs_aligner.lp_accel);
  get_psi_measurement_from_mag(&ahrs_impl.hi_res_euler.psi,
                               ahrs_impl.hi_res_euler.phi/F_UPDATE, ahrs_impl.hi_res_euler.theta/F_UPDATE, ahrs_aligner.lp_mag);

  EULERS_COPY(ahrs_impl.measure, ahrs_impl.hi_res_euler);
  EULERS_COPY(ahrs_impl.measurement, ahrs_impl.hi_res_euler);

  /* Compute LTP to IMU eulers      */
  EULERS_SDIV(ahrs_impl.ltp_to_imu_euler, ahrs_impl.hi_res_euler, F_UPDATE);

  set_body_state_from_euler();

  RATES_COPY( ahrs_impl.gyro_bias, ahrs_aligner.lp_gyro);
  ahrs.status = AHRS_RUNNING;

}

//#define USE_NOISE_CUT 1
//#define USE_NOISE_FILTER 1
#define NOISE_FILTER_GAIN 50

#if USE_NOISE_CUT
#include "led.h"
static inline bool_t cut_rates (struct Int32Rates i1, struct Int32Rates i2, int32_t threshold) {
  struct Int32Rates diff;
  RATES_DIFF(diff, i1, i2);
  if (diff.p < -threshold || diff.p > threshold ||
      diff.q < -threshold || diff.q > threshold ||
      diff.r < -threshold || diff.r > threshold) {
    return TRUE;
  } else {
    return FALSE;
  }
}
#define RATE_CUT_THRESHOLD RATE_BFP_OF_REAL(1)

static inline bool_t cut_accel (struct Int32Vect3 i1, struct Int32Vect3 i2, int32_t threshold) {
  struct Int32Vect3 diff;
  VECT3_DIFF(diff, i1, i2);
  if (diff.x < -threshold || diff.x > threshold ||
      diff.y < -threshold || diff.y > threshold ||
      diff.z < -threshold || diff.z > threshold) {
    LED_ON(4);
    return TRUE;
  } else {
    LED_OFF(4);
    return FALSE;
  }
}
#define ACCEL_CUT_THRESHOLD ACCEL_BFP_OF_REAL(20)

#endif

/*
 *
 * fc = 1/(2*pi*tau)
 *
 * alpha = dt / ( tau + dt )
 *
 *
 *  y(i) = alpha x(i) + (1-alpha) y(i-1)
 *  or
 *  y(i) = y(i-1) + alpha * (x(i) - y(i-1))
 *
 *
 */

void ahrs_propagate(void) {

  /* unbias gyro             */
  struct Int32Rates uf_rate;
  RATES_DIFF(uf_rate, imu.gyro, ahrs_impl.gyro_bias);
#if USE_NOISE_CUT
  static struct Int32Rates last_uf_rate = { 0, 0, 0 };
  if (!cut_rates(uf_rate, last_uf_rate, RATE_CUT_THRESHOLD)) {
#endif
    /* low pass rate */
#if USE_NOISE_FILTER
    RATES_SUM_SCALED(ahrs_impl.imu_rate, ahrs_impl.imu_rate, uf_rate, NOISE_FILTER_GAIN);
    RATES_SDIV(ahrs_impl.imu_rate, ahrs_impl.imu_rate, NOISE_FILTER_GAIN+1);
#else
    RATES_ADD(ahrs_impl.imu_rate, uf_rate);
    RATES_SDIV(ahrs_impl.imu_rate, ahrs_impl.imu_rate, 2);
#endif
#if USE_NOISE_CUT
  }
  RATES_COPY(last_uf_rate, uf_rate);
#endif

  /* integrate eulers */
  struct Int32Eulers euler_dot;
  INT32_EULERS_DOT_OF_RATES(euler_dot, ahrs_impl.ltp_to_imu_euler, ahrs_impl.imu_rate);
  EULERS_ADD(ahrs_impl.hi_res_euler, euler_dot);

  /* low pass measurement */
  EULERS_ADD(ahrs_impl.measure, ahrs_impl.measurement);
  EULERS_SDIV(ahrs_impl.measure, ahrs_impl.measure, 2);

  /* compute residual */
  EULERS_DIFF(ahrs_impl.residual, ahrs_impl.measure, ahrs_impl.hi_res_euler);
  INTEG_EULER_NORMALIZE(ahrs_impl.residual.psi);

  struct Int32Eulers correction;
  /* compute a correction */
  EULERS_SDIV(correction, ahrs_impl.residual, ahrs_impl.reinj_1);
  /* correct estimation */
  EULERS_ADD(ahrs_impl.hi_res_euler, correction);
  INTEG_EULER_NORMALIZE(ahrs_impl.hi_res_euler.psi);


  /* Compute LTP to IMU eulers      */
  EULERS_SDIV(ahrs_impl.ltp_to_imu_euler, ahrs_impl.hi_res_euler, F_UPDATE);

  set_body_state_from_euler();

}

void ahrs_update_accel(void) {

#if USE_NOISE_CUT || USE_NOISE_FILTER
  static struct Int32Vect3 last_accel = { 0, 0, 0 };
#endif
#if USE_NOISE_CUT
  if (!cut_accel(imu.accel, last_accel, ACCEL_CUT_THRESHOLD)) {
#endif
#if USE_NOISE_FILTER
    VECT3_SUM_SCALED(imu.accel, imu.accel, last_accel, NOISE_FILTER_GAIN);
    VECT3_SDIV(imu.accel, imu.accel, NOISE_FILTER_GAIN+1);
#endif
    get_phi_theta_measurement_fom_accel(&ahrs_impl.measurement.phi, &ahrs_impl.measurement.theta, imu.accel);
#if USE_NOISE_CUT
  }
  VECT3_COPY(last_accel, imu.accel);
#endif

}


void ahrs_update_mag(void) {

  get_psi_measurement_from_mag(&ahrs_impl.measurement.psi, ahrs_impl.ltp_to_imu_euler.phi, ahrs_impl.ltp_to_imu_euler.theta, imu.mag);

}

/* measures phi and theta assuming no dynamic acceleration ?!! */
__attribute__ ((always_inline)) static inline void get_phi_theta_measurement_fom_accel(int32_t* phi_meas, int32_t* theta_meas, struct Int32Vect3 accel) {

  INT32_ATAN2(*phi_meas, -accel.y, -accel.z);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, *phi_meas);
  int32_t cphi_ax = -INT_MULT_RSHIFT(cphi, accel.x, INT32_TRIG_FRAC);
  INT32_ATAN2(*theta_meas, -cphi_ax, -accel.z);
  *phi_meas   *= F_UPDATE;
  *theta_meas *= F_UPDATE;

}

/* measure psi by projecting magnetic vector in local tangeant plan */
__attribute__ ((always_inline)) static inline void get_psi_measurement_from_mag(int32_t* psi_meas, int32_t phi_est, int32_t theta_est, struct Int32Vect3 mag) {

  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, phi_est);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, phi_est);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, theta_est);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, theta_est);

  int32_t sphi_stheta = (sphi*stheta)>>INT32_TRIG_FRAC;
  int32_t cphi_stheta = (cphi*stheta)>>INT32_TRIG_FRAC;
  //int32_t sphi_ctheta = (sphi*ctheta)>>INT32_TRIG_FRAC;
  //int32_t cphi_ctheta = (cphi*ctheta)>>INT32_TRIG_FRAC;

  const int32_t mn = ctheta * mag.x + sphi_stheta * mag.y + cphi_stheta * mag.z;
  const int32_t me = 0      * mag.x + cphi        * mag.y - sphi        * mag.z;
  //const int32_t md =
  //  -stheta     * imu.mag.x +
  //  sphi_ctheta * imu.mag.y +
  //  cphi_ctheta * imu.mag.z;
  float m_psi = -atan2(me, mn);
  *psi_meas = ((m_psi - ahrs_impl.mag_offset)*(float)(1<<(INT32_ANGLE_FRAC))*F_UPDATE);

}

/* Rotate angles and rates from imu to body frame and set state */
static void set_body_state_from_euler(void) {
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  struct Int32RMat ltp_to_imu_rmat, ltp_to_body_rmat;
  /* Compute LTP to IMU rotation matrix */
  INT32_RMAT_OF_EULERS(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_euler);
  /* Compute LTP to BODY rotation matrix */
  INT32_RMAT_COMP_INV(ltp_to_body_rmat, ltp_to_imu_rmat, *body_to_imu_rmat);
  /* Set state */
#ifdef AHRS_UPDATE_FW_ESTIMATOR
  struct Int32Eulers ltp_to_body_euler;
  INT32_EULERS_OF_RMAT(ltp_to_body_euler, ltp_to_body_rmat);
  ltp_to_body_euler.phi -= ANGLE_BFP_OF_REAL(ins_roll_neutral);
  ltp_to_body_euler.theta -= ANGLE_BFP_OF_REAL(ins_pitch_neutral);
  stateSetNedToBodyEulers_i(&ltp_to_body_euler);
#else
  stateSetNedToBodyRMat_i(&ltp_to_body_rmat);
#endif

  struct Int32Rates body_rate;
  /* compute body rates */
  INT32_RMAT_TRANSP_RATEMULT(body_rate, *body_to_imu_rmat, ahrs_impl.imu_rate);
  /* Set state */
  stateSetBodyRates_i(&body_rate);

}


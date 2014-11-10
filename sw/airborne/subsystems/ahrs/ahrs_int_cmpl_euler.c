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
#include "subsystems/abi.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

#ifndef FACE_REINJ_1
#define FACE_REINJ_1 1024
#endif

#ifndef AHRS_MAG_OFFSET
#define AHRS_MAG_OFFSET 0.
#endif

struct AhrsIntCmplEuler ahrs_ice;

static inline void get_phi_theta_measurement_fom_accel(int32_t* phi_meas, int32_t* theta_meas, struct Int32Vect3* accel);
static inline void get_psi_measurement_from_mag(int32_t* psi_meas, int32_t phi_est, int32_t theta_est, struct Int32Vect3* mag);
static inline void set_body_state_from_euler(void);

#define F_UPDATE 512

#define PI_INTEG_EULER     (INT32_ANGLE_PI * F_UPDATE)
#define TWO_PI_INTEG_EULER (INT32_ANGLE_2_PI * F_UPDATE)
#define INTEG_EULER_NORMALIZE(_a) {                         \
    while (_a >  PI_INTEG_EULER)  _a -= TWO_PI_INTEG_EULER; \
    while (_a < -PI_INTEG_EULER)  _a += TWO_PI_INTEG_EULER; \
  }


/** ABI binding for IMU data.
 * Used for gyro, accel and mag ABI messages.
 */
#ifndef AHRS_ICE_IMU_ID
#define AHRS_ICE_IMU_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;

static abi_event aligner_ev;

static void gyro_cb(uint8_t __attribute__((unused)) sender_id, const uint32_t* stamp,
                    const struct Int32Rates* gyro)
{
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_ice.status == AHRS_ICE_RUNNING) {
    float dt = (float)(*stamp - last_stamp) * 1e-6;
    ahrs_ice_propagate((struct Int32Rates*)gyro, dt);
  }
  last_stamp = *stamp;
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id, const uint32_t* stamp,
                     const struct Int32Vect3* accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_euler propagation.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_ice.status == AHRS_ICE_RUNNING) {
    float dt = (float)(*stamp - last_stamp) * 1e-6;
    ahrs_ice_update_accel((struct Int32Vect3*)accel, dt);
  }
  last_stamp = *stamp;
#else
PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS int_cmpl_euler propagation.")
PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_ice.status == AHRS_ICE_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_ice_propagate((struct Int32Rates*)gyro, dt);
  }
#endif
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id, const uint32_t* stamp,
                   const struct Int32Vect3* mag)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_euler accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_ice.status == AHRS_ICE_RUNNING) {
    float dt = (float)(*stamp - last_stamp) * 1e-6;
    ahrs_ice_update_mag((struct Int32Vect3*)mag, dt);
  }
  last_stamp = *stamp;
#else
PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS int_cmpl_quat accel update.")
PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_ice.status == AHRS_ICE_RUNNING) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_ice_update_accel((struct Int32Rates*)accel, dt);
  }
#endif
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       const uint32_t* stamp __attribute__((unused)),
                       const struct Int32Rates* lp_gyro, const struct Int32Vect3* lp_accel,
                       const struct Int32Vect3* lp_mag)
{
  if (ahrs_ice.status != AHRS_ICE_RUNNING) {
    ahrs_ice_align((struct Int32Rates*)lp_gyro, (struct Int32Vect3*)lp_accel,
                   (struct Int32Vect3*)lp_mag);
  }
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_filter(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_FILTER(trans, dev, AC_ID,
      &ahrs_ice.ltp_to_imu_euler.phi,
      &ahrs_ice.ltp_to_imu_euler.theta,
      &ahrs_ice.ltp_to_imu_euler.psi,
      &ahrs_ice.measure.phi,
      &ahrs_ice.measure.theta,
      &ahrs_ice.measure.psi,
      &ahrs_ice.hi_res_euler.phi,
      &ahrs_ice.hi_res_euler.theta,
      &ahrs_ice.hi_res_euler.psi,
      &ahrs_ice.residual.phi,
      &ahrs_ice.residual.theta,
      &ahrs_ice.residual.psi,
      &ahrs_ice.gyro_bias.p,
      &ahrs_ice.gyro_bias.q,
      &ahrs_ice.gyro_bias.r);
}

static void send_euler(struct transport_tx *trans, struct link_device *dev) {
  struct Int32Eulers* eulers = stateGetNedToBodyEulers_i();
  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
      &ahrs_ice.ltp_to_imu_euler.phi,
      &ahrs_ice.ltp_to_imu_euler.theta,
      &ahrs_ice.ltp_to_imu_euler.psi,
      &(eulers->phi),
      &(eulers->theta),
      &(eulers->psi));
}

static void send_bias(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
      &ahrs_ice.gyro_bias.p, &ahrs_ice.gyro_bias.q, &ahrs_ice.gyro_bias.r);
}
#endif

void ahrs_ice_register(void)
{
  ahrs_register_impl(ahrs_ice_init, NULL);
}

void ahrs_ice_init(struct OrientationReps* body_to_imu)
{
  /* save body_to_imu pointer */
  ahrs_ice.body_to_imu = body_to_imu;

  ahrs_ice.status = AHRS_ICE_UNINIT;

  /* set ltp_to_imu so that body is zero */
  memcpy(&ahrs_ice.ltp_to_imu_euler, orientationGetEulers_i(ahrs_ice.body_to_imu),
         sizeof(struct Int32Eulers));
  INT_RATES_ZERO(ahrs_ice.imu_rate);

  INT_RATES_ZERO(ahrs_ice.gyro_bias);
  ahrs_ice.reinj_1 = FACE_REINJ_1;

  ahrs_ice.mag_offset = AHRS_MAG_OFFSET;

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_ICE_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_ICE_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_ICE_IMU_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "FILTER", send_filter);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_EULER_INT", send_euler);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_GYRO_BIAS_INT", send_bias);
#endif
}

bool_t ahrs_ice_align(struct Int32Rates* lp_gyro, struct Int32Vect3* lp_accel,
                  struct Int32Vect3* lp_mag)
{

  get_phi_theta_measurement_fom_accel(&ahrs_ice.hi_res_euler.phi, &ahrs_ice.hi_res_euler.theta, lp_accel);
  get_psi_measurement_from_mag(&ahrs_ice.hi_res_euler.psi,
                               ahrs_ice.hi_res_euler.phi/F_UPDATE, ahrs_ice.hi_res_euler.theta/F_UPDATE, lp_mag);

  EULERS_COPY(ahrs_ice.measure, ahrs_ice.hi_res_euler);
  EULERS_COPY(ahrs_ice.measurement, ahrs_ice.hi_res_euler);

  /* Compute LTP to IMU eulers      */
  EULERS_SDIV(ahrs_ice.ltp_to_imu_euler, ahrs_ice.hi_res_euler, F_UPDATE);

  set_body_state_from_euler();

  RATES_COPY(ahrs_ice.gyro_bias, *lp_gyro);

  ahrs_ice.status = AHRS_ICE_RUNNING;

  return TRUE;
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

void ahrs_ice_propagate(struct Int32Rates* gyro, float dt __attribute__((unused))) {

  /* unbias gyro             */
  struct Int32Rates uf_rate;
  RATES_DIFF(uf_rate, *gyro, ahrs_ice.gyro_bias);
#if USE_NOISE_CUT
  static struct Int32Rates last_uf_rate = { 0, 0, 0 };
  if (!cut_rates(uf_rate, last_uf_rate, RATE_CUT_THRESHOLD)) {
#endif
    /* low pass rate */
#if USE_NOISE_FILTER
    RATES_SUM_SCALED(ahrs_ice.imu_rate, ahrs_ice.imu_rate, uf_rate, NOISE_FILTER_GAIN);
    RATES_SDIV(ahrs_ice.imu_rate, ahrs_ice.imu_rate, NOISE_FILTER_GAIN+1);
#else
    RATES_ADD(ahrs_ice.imu_rate, uf_rate);
    RATES_SDIV(ahrs_ice.imu_rate, ahrs_ice.imu_rate, 2);
#endif
#if USE_NOISE_CUT
  }
  RATES_COPY(last_uf_rate, uf_rate);
#endif

  /* integrate eulers */
  struct Int32Eulers euler_dot;
  int32_eulers_dot_of_rates(&euler_dot, &ahrs_ice.ltp_to_imu_euler, &ahrs_ice.imu_rate);
  EULERS_ADD(ahrs_ice.hi_res_euler, euler_dot);

  /* low pass measurement */
  EULERS_ADD(ahrs_ice.measure, ahrs_ice.measurement);
  EULERS_SDIV(ahrs_ice.measure, ahrs_ice.measure, 2);

  /* compute residual */
  EULERS_DIFF(ahrs_ice.residual, ahrs_ice.measure, ahrs_ice.hi_res_euler);
  INTEG_EULER_NORMALIZE(ahrs_ice.residual.psi);

  struct Int32Eulers correction;
  /* compute a correction */
  EULERS_SDIV(correction, ahrs_ice.residual, ahrs_ice.reinj_1);
  /* correct estimation */
  EULERS_ADD(ahrs_ice.hi_res_euler, correction);
  INTEG_EULER_NORMALIZE(ahrs_ice.hi_res_euler.psi);


  /* Compute LTP to IMU eulers      */
  EULERS_SDIV(ahrs_ice.ltp_to_imu_euler, ahrs_ice.hi_res_euler, F_UPDATE);

  set_body_state_from_euler();

}

void ahrs_ice_update_accel(struct Int32Vect3* accel, float dt __attribute__((unused))) {

#if USE_NOISE_CUT || USE_NOISE_FILTER
  static struct Int32Vect3 last_accel = { 0, 0, 0 };
#endif
#if USE_NOISE_CUT
  if (!cut_accel(*accel, last_accel, ACCEL_CUT_THRESHOLD)) {
#endif
#if USE_NOISE_FILTER
    VECT3_SUM_SCALED(*accel, *accel, last_accel, NOISE_FILTER_GAIN);
    VECT3_SDIV(*accel, *accel, NOISE_FILTER_GAIN+1);
#endif
    get_phi_theta_measurement_fom_accel(&ahrs_ice.measurement.phi, &ahrs_ice.measurement.theta, accel);
#if USE_NOISE_CUT
  }
  VECT3_COPY(last_accel, *accel);
#endif

}


void ahrs_ice_update_mag(struct Int32Vect3* mag, float dt __attribute__((unused))) {

  get_psi_measurement_from_mag(&ahrs_ice.measurement.psi, ahrs_ice.ltp_to_imu_euler.phi, ahrs_ice.ltp_to_imu_euler.theta, mag);

}

/* measures phi and theta assuming no dynamic acceleration ?!! */
__attribute__ ((always_inline)) static inline void get_phi_theta_measurement_fom_accel(int32_t* phi_meas, int32_t* theta_meas, struct Int32Vect3* accel) {

  *phi_meas = int32_atan2(-accel->y, -accel->z);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, *phi_meas);
  int32_t cphi_ax = -INT_MULT_RSHIFT(cphi, accel->x, INT32_TRIG_FRAC);
  *theta_meas = int32_atan2(-cphi_ax, -accel->z);
  *phi_meas   *= F_UPDATE;
  *theta_meas *= F_UPDATE;

}

/* measure psi by projecting magnetic vector in local tangeant plan */
__attribute__ ((always_inline)) static inline void get_psi_measurement_from_mag(int32_t* psi_meas, int32_t phi_est, int32_t theta_est, struct Int32Vect3* mag) {

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

  const int32_t mn = ctheta * mag->x + sphi_stheta * mag->y + cphi_stheta * mag->z;
  const int32_t me = 0      * mag->x + cphi        * mag->y - sphi        * mag->z;
  //const int32_t md =
  //  -stheta     * mag->x +
  //  sphi_ctheta * mag->y +
  //  cphi_ctheta * mag->z;
  float m_psi = -atan2(me, mn);
  *psi_meas = ((m_psi - ahrs_ice.mag_offset)*(float)(1<<(INT32_ANGLE_FRAC))*F_UPDATE);

}

/* Rotate angles and rates from imu to body frame and set state */
static void set_body_state_from_euler(void) {
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(ahrs_ice.body_to_imu);
  struct Int32RMat ltp_to_imu_rmat, ltp_to_body_rmat;
  /* Compute LTP to IMU rotation matrix */
  int32_rmat_of_eulers(&ltp_to_imu_rmat, &ahrs_ice.ltp_to_imu_euler);
  /* Compute LTP to BODY rotation matrix */
  int32_rmat_comp_inv(&ltp_to_body_rmat, &ltp_to_imu_rmat, body_to_imu_rmat);
  /* Set state */
  stateSetNedToBodyRMat_i(&ltp_to_body_rmat);

  struct Int32Rates body_rate;
  /* compute body rates */
  int32_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_ice.imu_rate);
  /* Set state */
  stateSetBodyRates_i(&body_rate);

}

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

#include "ahrs_int_cmpl_euler.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

#ifndef FACE_REINJ_1
#define FACE_REINJ_1 1024
#endif


struct AhrsIntCmplEuler ahrs_impl;

static inline void get_phi_theta_measurement_fom_accel(int32_t* phi_meas, int32_t* theta_meas, struct Int32Vect3 accel);
static inline void get_psi_measurement_from_mag(int32_t* psi_meas, int32_t phi_est, int32_t theta_est, struct Int32Vect3 mag);
static inline void compute_imu_quat_and_rmat_from_euler(void);
static inline void compute_body_orientation(void);

#define F_UPDATE 512

#define PI_INTEG_EULER     (INT32_ANGLE_PI * F_UPDATE)
#define TWO_PI_INTEG_EULER (INT32_ANGLE_2_PI * F_UPDATE)
#define INTEG_EULER_NORMALIZE(_a) {				\
    while (_a >  PI_INTEG_EULER)  _a -= TWO_PI_INTEG_EULER;	\
    while (_a < -PI_INTEG_EULER)  _a += TWO_PI_INTEG_EULER;	\
  }

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  /* set ltp_to_body to zero */
  INT_EULERS_ZERO(ahrs.ltp_to_body_euler);
  INT32_QUAT_ZERO(ahrs.ltp_to_body_quat);
  INT32_RMAT_ZERO(ahrs.ltp_to_body_rmat);
  INT_RATES_ZERO(ahrs.body_rate);

  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs.ltp_to_imu_quat, imu.body_to_imu_quat);
  RMAT_COPY(ahrs.ltp_to_imu_rmat, imu.body_to_imu_rmat);
  INT32_EULERS_OF_RMAT(ahrs.ltp_to_imu_euler, ahrs.ltp_to_imu_rmat);
  INT_RATES_ZERO(ahrs.imu_rate);

  INT_RATES_ZERO(ahrs_impl.gyro_bias);
  ahrs_impl.reinj_1 = FACE_REINJ_1;

#ifdef IMU_MAG_OFFSET
  ahrs_mag_offset = IMU_MAG_OFFSET;
#else
  ahrs_mag_offset = 0.;
#endif
}

void ahrs_align(void) {

  get_phi_theta_measurement_fom_accel(&ahrs_impl.hi_res_euler.phi, &ahrs_impl.hi_res_euler.theta, ahrs_aligner.lp_accel);
  get_psi_measurement_from_mag(&ahrs_impl.hi_res_euler.psi,
			       ahrs_impl.hi_res_euler.phi/F_UPDATE, ahrs_impl.hi_res_euler.theta/F_UPDATE, ahrs_aligner.lp_mag);

  EULERS_COPY(ahrs_impl.measure, ahrs_impl.hi_res_euler);
  EULERS_COPY(ahrs_impl.measurement, ahrs_impl.hi_res_euler);

  /* Compute LTP to IMU eulers      */
  EULERS_SDIV(ahrs.ltp_to_imu_euler, ahrs_impl.hi_res_euler, F_UPDATE);

  compute_imu_quat_and_rmat_from_euler();

  compute_body_orientation();

  RATES_COPY( ahrs_impl.gyro_bias, ahrs_aligner.lp_gyro);
  ahrs.status = AHRS_RUNNING;

}



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
  /* low pass rate */
  RATES_ADD(ahrs.imu_rate, uf_rate);
  RATES_SDIV(ahrs.imu_rate, ahrs.imu_rate, 2);

  /* integrate eulers */
  struct Int32Eulers euler_dot;
  INT32_EULERS_DOT_OF_RATES(euler_dot, ahrs.ltp_to_imu_euler, ahrs.imu_rate);
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
  EULERS_SDIV(ahrs.ltp_to_imu_euler, ahrs_impl.hi_res_euler, F_UPDATE);

  compute_imu_quat_and_rmat_from_euler();

  compute_body_orientation();

}

void ahrs_update_accel(void) {

  get_phi_theta_measurement_fom_accel(&ahrs_impl.measurement.phi, &ahrs_impl.measurement.theta, imu.accel);

}


void ahrs_update_mag(void) {

  get_psi_measurement_from_mag(&ahrs_impl.measurement.psi, ahrs.ltp_to_imu_euler.phi, ahrs.ltp_to_imu_euler.theta, imu.mag);

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
  *psi_meas = ((m_psi - RadOfDeg(ahrs_mag_offset))*(float)(1<<(INT32_ANGLE_FRAC))*F_UPDATE);

}

/* Compute ltp to imu rotation in quaternion and rotation matrice representation
   from the euler angle representation */
__attribute__ ((always_inline)) static inline void compute_imu_quat_and_rmat_from_euler(void) {

  /* Compute LTP to IMU quaternion */
  INT32_QUAT_OF_EULERS(ahrs.ltp_to_imu_quat, ahrs.ltp_to_imu_euler);
  /* Compute LTP to IMU rotation matrix */
  INT32_RMAT_OF_EULERS(ahrs.ltp_to_imu_rmat, ahrs.ltp_to_imu_euler);

}

__attribute__ ((always_inline)) static inline void compute_body_orientation(void) {

  /* Compute LTP to BODY quaternion */
  INT32_QUAT_COMP_INV(ahrs.ltp_to_body_quat, ahrs.ltp_to_imu_quat, imu.body_to_imu_quat);
  /* Compute LTP to BODY rotation matrix */
  INT32_RMAT_COMP_INV(ahrs.ltp_to_body_rmat, ahrs.ltp_to_imu_rmat, imu.body_to_imu_rmat);
  /* compute LTP to BODY eulers */
  INT32_EULERS_OF_RMAT(ahrs.ltp_to_body_euler, ahrs.ltp_to_body_rmat);
  /* compute body rates */
  INT32_RMAT_TRANSP_RATEMULT(ahrs.body_rate, imu.body_to_imu_rmat, ahrs.imu_rate);

}


#ifdef AHRS_UPDATE_FW_ESTIMATOR
// TODO use ahrs result directly
#include "estimator.h"
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
void ahrs_update_fw_estimator(void)
{
  struct FloatEulers att;
  // export results to estimator
  EULERS_FLOAT_OF_BFP(att, ahrs.ltp_to_body_euler);

  estimator_phi   = att.phi - ins_roll_neutral;
  estimator_theta = att.theta - ins_pitch_neutral;
  estimator_psi   = att.psi;

  struct FloatRates rates;
  RATES_FLOAT_OF_BFP(rates, ahrs.body_rate);
  estimator_p = rates.p;
  estimator_q = rates.q;

}
#endif //AHRS_UPDATE_FW_ESTIMATOR

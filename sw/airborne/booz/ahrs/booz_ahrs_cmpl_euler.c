/*
 * $Id:  $
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "booz_ahrs_cmpl_euler.h"

#include "booz_imu.h"
#include "booz_ahrs_aligner.h"

#include "airframe.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"


struct Int32Rates  booz2_face_gyro_bias;
struct Int32Eulers booz2_face_measure;
struct Int32Eulers booz2_face_residual;
struct Int32Eulers booz2_face_uncorrected;
struct Int32Eulers booz2_face_corrected;

struct Int32Eulers measurement;

int32_t booz2_face_reinj_1;

void booz_ahrs_init(void) {
  booz_ahrs.status = BOOZ_AHRS_UNINIT;
  INT_EULERS_ZERO(booz_ahrs.ltp_to_body_euler);
  INT_EULERS_ZERO(booz_ahrs.ltp_to_imu_euler);
  INT32_QUAT_ZERO(booz_ahrs.ltp_to_body_quat);
  INT32_QUAT_ZERO(booz_ahrs.ltp_to_imu_quat);
  INT_RATES_ZERO(booz_ahrs.body_rate);
  INT_RATES_ZERO(booz_ahrs.imu_rate);
  INT_RATES_ZERO(booz2_face_gyro_bias);
  booz2_face_reinj_1 = BOOZ2_FACE_REINJ_1;

  INT_EULERS_ZERO(booz2_face_uncorrected);
}

void booz_ahrs_align(void) {

  RATES_COPY( booz2_face_gyro_bias, booz_ahrs_aligner.lp_gyro);
  booz_ahrs.status = BOOZ_AHRS_RUNNING;

}


#define F_UPDATE 512

#define PI_INTEG_EULER     (INT32_ANGLE_PI * F_UPDATE)
#define TWO_PI_INTEG_EULER (INT32_ANGLE_2_PI * F_UPDATE)
#define INTEG_EULER_NORMALIZE(_a) {				\
    while (_a >  PI_INTEG_EULER)  _a -= TWO_PI_INTEG_EULER;	\
    while (_a < -PI_INTEG_EULER)  _a += TWO_PI_INTEG_EULER;	\
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

void booz_ahrs_propagate(void) {

  /* unbias gyro             */
  struct Int32Rates uf_rate;
  RATES_DIFF(uf_rate, booz_imu.gyro, booz2_face_gyro_bias);
  /* low pass rate */  
  RATES_ADD(booz_ahrs.imu_rate, uf_rate);
  RATES_SDIV(booz_ahrs.imu_rate, booz_ahrs.imu_rate, 2);

  /* integrate eulers */
  struct Int32Eulers euler_dot;
  INT32_EULERS_DOT_OF_RATES(euler_dot, booz_ahrs.ltp_to_imu_euler, booz_ahrs.imu_rate);
  EULERS_ADD(booz2_face_corrected, euler_dot);

  /* low pass measurement */
  EULERS_ADD(booz2_face_measure, measurement);
  EULERS_SDIV(booz2_face_measure, booz2_face_measure, 2);
  /* compute residual */
  EULERS_DIFF(booz2_face_residual, booz2_face_measure, booz2_face_corrected);
  INTEG_EULER_NORMALIZE(booz2_face_residual.psi);

  struct Int32Eulers correction;
  /* compute a correction */
  EULERS_SDIV(correction, booz2_face_residual, booz2_face_reinj_1);
  /* correct estimation */
  EULERS_ADD(booz2_face_corrected, correction);
  INTEG_EULER_NORMALIZE(booz2_face_corrected.psi);


  /* Compute LTP to IMU eulers      */
  EULERS_SDIV(booz_ahrs.ltp_to_imu_euler, booz2_face_corrected, F_UPDATE);
  /* Compute LTP to IMU quaternion */
  INT32_QUAT_OF_EULERS(booz_ahrs.ltp_to_imu_quat, booz_ahrs.ltp_to_imu_euler);
  /* Compute LTP to IMU rotation matrix */
  INT32_RMAT_OF_EULERS(booz_ahrs.ltp_to_imu_rmat, booz_ahrs.ltp_to_imu_euler);

  /* Compute LTP to BODY quaternion */
  INT32_QUAT_COMP_INV(booz_ahrs.ltp_to_body_quat, booz_ahrs.ltp_to_imu_quat, booz_imu.body_to_imu_quat);
  /* Compute LTP to BODY rotation matrix */
  INT32_RMAT_COMP_INV(booz_ahrs.ltp_to_body_rmat, booz_ahrs.ltp_to_imu_rmat, booz_imu.body_to_imu_rmat);
  /* compute LTP to BODY eulers */
  INT32_EULERS_OF_RMAT(booz_ahrs.ltp_to_body_euler, booz_ahrs.ltp_to_body_rmat);
  /* compute body rates */
  INT32_RMAT_TRANSP_RATEMULT(booz_ahrs.body_rate, booz_imu.body_to_imu_rmat, booz_ahrs.imu_rate);

}

void booz_ahrs_update_accel(void) {

  /* build a measurement assuming constant acceleration ?!! */
  INT32_ATAN2(measurement.phi, -booz_imu.accel.y, -booz_imu.accel.z);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, measurement.phi);
  int32_t cphi_ax = -INT_MULT_RSHIFT(cphi, booz_imu.accel.x, INT32_TRIG_FRAC);
  INT32_ATAN2(measurement.theta, -cphi_ax, -booz_imu.accel.z);
  measurement.phi *= F_UPDATE;
  measurement.theta *= F_UPDATE;

}

/* measure psi assuming magnetic vector is in earth plan (md = 0) */
void booz_ahrs_update_mag(void) {

  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, booz_ahrs.ltp_to_imu_euler.phi);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, booz_ahrs.ltp_to_imu_euler.phi);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, booz_ahrs.ltp_to_imu_euler.theta);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, booz_ahrs.ltp_to_imu_euler.theta);

  int32_t sphi_stheta = (sphi*stheta)>>INT32_TRIG_FRAC;
  int32_t cphi_stheta = (cphi*stheta)>>INT32_TRIG_FRAC;
  //int32_t sphi_ctheta = (sphi*ctheta)>>INT32_TRIG_FRAC;
  //int32_t cphi_ctheta = (cphi*ctheta)>>INT32_TRIG_FRAC;

  const int32_t mn =
    ctheta      * booz_imu.mag.x +
    sphi_stheta * booz_imu.mag.y +
    cphi_stheta * booz_imu.mag.z;
  const int32_t me =
    0           * booz_imu.mag.x +
    cphi        * booz_imu.mag.y +
    -sphi       * booz_imu.mag.z;
  //const int32_t md =
  //  -stheta     * booz_imu.mag.x +
  //  sphi_ctheta * booz_imu.mag.y +
  //  cphi_ctheta * booz_imu.mag.z;
  float m_psi = -atan2(me, mn);
  measurement.psi = ((m_psi)*(float)(1<<(INT32_ANGLE_FRAC))*F_UPDATE);

}


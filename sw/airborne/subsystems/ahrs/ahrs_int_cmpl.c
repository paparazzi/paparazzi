/*
 * $Id$
 *
 * Copyright (C) 2008-2011 The Paparazzi Team
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


// TODO
//
// gravity heuristic
// gps based gravity correction
// gps update for yaw on fixed wing ?
//

#include "subsystems/ahrs/ahrs_int_cmpl.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ahrs/ahrs_int_utils.h"

#include "subsystems/imu.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

//#include "../../test/pprz_algebra_print.h"

static inline void ahrs_update_mag_full(void);
static inline void ahrs_update_mag_2d(void);


/* in place quaternion first order integration with constante rotational velocity */
/*  */
#define INT32_QUAT_INTEGRATE_FI(_q, _hr, _omega, _f) {			\
    _hr.qi += -_omega.p*_q.qx - _omega.q*_q.qy - _omega.r*_q.qz;	\
    _hr.qx +=  _omega.p*_q.qi + _omega.r*_q.qy - _omega.q*_q.qz;	\
    _hr.qy +=  _omega.q*_q.qi - _omega.r*_q.qx + _omega.p*_q.qz;	\
    _hr.qz +=  _omega.r*_q.qi + _omega.q*_q.qx - _omega.p*_q.qy;	\
									\
    ldiv_t _div = ldiv(_hr.qi, ((1<<INT32_RATE_FRAC)*_f*2));		\
    _q.qi+= _div.quot;							\
    _hr.qi = _div.rem;							\
									\
    _div = ldiv(_hr.qx, ((1<<INT32_RATE_FRAC)*_f*2));			\
    _q.qx+= _div.quot;							\
    _hr.qx = _div.rem;							\
									\
    _div = ldiv(_hr.qy, ((1<<INT32_RATE_FRAC)*_f*2));			\
    _q.qy+= _div.quot;							\
    _hr.qy = _div.rem;							\
									\
    _div = ldiv(_hr.qz, ((1<<INT32_RATE_FRAC)*_f*2));			\
    _q.qz+= _div.quot;							\
    _hr.qz = _div.rem;							\
									\
  }



struct AhrsIntCmpl ahrs_impl;

static inline void compute_imu_quat_and_rmat_from_euler(void);
static inline void compute_imu_euler_and_rmat_from_quat(void);
static inline void compute_body_orientation(void);

void ahrs_init(void) {

  ahrs.status = AHRS_UNINIT;

  // FIXME: make ltp_to_imu and ltp_to_body coherent
  INT_EULERS_ZERO(ahrs.ltp_to_body_euler);
  INT_EULERS_ZERO(ahrs.ltp_to_imu_euler);
  INT32_QUAT_ZERO(ahrs.ltp_to_body_quat);
  INT32_QUAT_ZERO(ahrs.ltp_to_imu_quat);
  INT32_RMAT_ZERO(ahrs.ltp_to_body_rmat);
  INT_RATES_ZERO(ahrs.body_rate);
  INT_RATES_ZERO(ahrs.imu_rate);

  INT_RATES_ZERO(ahrs_impl.gyro_bias);
  INT_RATES_ZERO(ahrs_impl.rate_correction);
  INT_RATES_ZERO(ahrs_impl.high_rez_bias);

}

void ahrs_align(void) {


  /* Compute an initial orientation using euler angles */
  ahrs_int_get_euler_from_accel_mag(&ahrs.ltp_to_imu_euler, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);
  /* Convert initial orientation in quaternion and rotation matrice representations. */
  compute_imu_quat_and_rmat_from_euler();

  compute_body_orientation();

  /* Use low passed gyro value as initial bias */
  RATES_COPY( ahrs_impl.gyro_bias, ahrs_aligner.lp_gyro);
  RATES_COPY( ahrs_impl.high_rez_bias, ahrs_aligner.lp_gyro);
  INT_RATES_LSHIFT(ahrs_impl.high_rez_bias, ahrs_impl.high_rez_bias, 28);

  ahrs.status = AHRS_RUNNING;

}



/*
 *
 *
 *
 */
void ahrs_propagate(void) {

  /* unbias gyro             */
  struct Int32Rates omega;
  RATES_DIFF(omega, imu.gyro_prev, ahrs_impl.gyro_bias);

  /* low pass rate */
#ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  RATES_SMUL(ahrs.imu_rate, ahrs.imu_rate,2);
  RATES_ADD(ahrs.imu_rate, omega);
  RATES_SDIV(ahrs.imu_rate, ahrs.imu_rate, 3);
#else
  RATES_COPY(ahrs.imu_rate, omega);
#endif

  /* add correction     */
  RATES_ADD(omega, ahrs_impl.rate_correction);
  /* and zeros it */
  INT_RATES_ZERO(ahrs_impl.rate_correction);

  /* integrate quaternion */
  INT32_QUAT_INTEGRATE_FI(ahrs.ltp_to_imu_quat, ahrs_impl.high_rez_quat, omega, AHRS_PROPAGATE_FREQUENCY);
  INT32_QUAT_NORMALIZE(ahrs.ltp_to_imu_quat);

  compute_imu_euler_and_rmat_from_quat();

  compute_body_orientation();

}




void ahrs_update_accel(void) {

  struct Int32Vect3 c2 = { RMAT_ELMT(ahrs.ltp_to_imu_rmat, 0,2),
			   RMAT_ELMT(ahrs.ltp_to_imu_rmat, 1,2),
			   RMAT_ELMT(ahrs.ltp_to_imu_rmat, 2,2)};
  struct Int32Vect3 residual;
#ifdef AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  // FIXME: check overflow ?
  const struct Int32Vect3 Xdd_imu = {
    0,
     ((ahrs_impl.ltp_vel_norm>>INT32_ACCEL_FRAC) * ahrs.imu_rate.r)
    >>(INT32_SPEED_FRAC+INT32_RATE_FRAC-INT32_ACCEL_FRAC-INT32_ACCEL_FRAC),
    -((ahrs_impl.ltp_vel_norm>>INT32_ACCEL_FRAC) * ahrs.imu_rate.q)
    >>(INT32_SPEED_FRAC+INT32_RATE_FRAC-INT32_ACCEL_FRAC-INT32_ACCEL_FRAC)
  };
  struct Int32Vect3 corrected_gravity;
  VECT3_DIFF(corrected_gravity, imu.accel, Xdd_imu);
  INT32_VECT3_CROSS_PRODUCT(residual, corrected_gravity, c2);
#else
  INT32_VECT3_CROSS_PRODUCT(residual, imu.accel, c2);
#endif

  // residual FRAC : ACCEL_FRAC + TRIG_FRAC = 10 + 14 = 24
  // rate_correction FRAC = RATE_FRAC = 12
  // 2^12 / 2^24 * 5e-2 = 1/81920
  ahrs_impl.rate_correction.p += -residual.x/82000;
  ahrs_impl.rate_correction.q += -residual.y/82000;
  ahrs_impl.rate_correction.r += -residual.z/82000;

  // residual FRAC = ACCEL_FRAC + TRIG_FRAC = 10 + 14 = 24
  // high_rez_bias = RATE_FRAC+28 = 40
  // 2^40 / 2^24 * 5e-6 = 1/3.05

  //  ahrs_impl.high_rez_bias.p += residual.x*3;
  //  ahrs_impl.high_rez_bias.q += residual.y*3;
  //  ahrs_impl.high_rez_bias.r += residual.z*3;

  ahrs_impl.high_rez_bias.p += residual.x;
  ahrs_impl.high_rez_bias.q += residual.y;
  ahrs_impl.high_rez_bias.r += residual.z;


  /*                        */
  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);


}

void ahrs_update_mag(void) {
#ifdef AHRS_MAG_UPDATE_YAW_ONLY
  ahrs_update_mag_2d();
#else
  ahrs_update_mag_full();
#endif
}


static inline void ahrs_update_mag_full(void) {
  const struct Int32Vect3 expected_ltp = {MAG_BFP_OF_REAL(AHRS_H_X),
					  MAG_BFP_OF_REAL(AHRS_H_Y),
					  MAG_BFP_OF_REAL(AHRS_H_Z)};
  struct Int32Vect3 expected_imu;
  INT32_RMAT_VMULT(expected_imu, ahrs.ltp_to_imu_rmat, expected_ltp);

  struct Int32Vect3 residual;
  INT32_VECT3_CROSS_PRODUCT(residual, imu.mag, expected_imu);

  ahrs_impl.rate_correction.p += residual.x/32/16;
  ahrs_impl.rate_correction.q += residual.y/32/16;
  ahrs_impl.rate_correction.r += residual.z/32/16;


  ahrs_impl.high_rez_bias.p += -residual.x/32*1024;
  ahrs_impl.high_rez_bias.q += -residual.y/32*1024;
  ahrs_impl.high_rez_bias.r += -residual.z/32*1024;


  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);

}


static inline void ahrs_update_mag_2d(void) {

  const struct Int32Vect2 expected_ltp = {MAG_BFP_OF_REAL(AHRS_H_X),
					  MAG_BFP_OF_REAL(AHRS_H_Y)};

  struct Int32Vect3 measured_ltp;
  INT32_RMAT_TRANSP_VMULT(measured_ltp, ahrs.ltp_to_imu_rmat, imu.mag);

  struct Int32Vect3 residual_ltp =
    { 0,
      0,
      (measured_ltp.x * expected_ltp.y - measured_ltp.y * expected_ltp.x)/(1<<5)};

  struct Int32Vect3 residual_imu;
  INT32_RMAT_VMULT(residual_imu, ahrs.ltp_to_imu_rmat, residual_ltp);

  // residual_ltp FRAC = 2 * MAG_FRAC = 22
  // rate_correction FRAC = RATE_FRAC = 12
  // 2^12 / 2^22 * 2.5 = 1/410

  //  ahrs_impl.rate_correction.p += residual_imu.x*(1<<5)/410;
  //  ahrs_impl.rate_correction.q += residual_imu.y*(1<<5)/410;
  //  ahrs_impl.rate_correction.r += residual_imu.z*(1<<5)/410;

  ahrs_impl.rate_correction.p += residual_imu.x/16;
  ahrs_impl.rate_correction.q += residual_imu.y/16;
  ahrs_impl.rate_correction.r += residual_imu.z/16;


  // residual_ltp FRAC = 2 * MAG_FRAC = 22
  // high_rez_bias = RATE_FRAC+28 = 40
  // 2^40 / 2^22 * 2.5e-3 = 655

  //  ahrs_impl.high_rez_bias.p -= residual_imu.x*(1<<5)*655;
  //  ahrs_impl.high_rez_bias.q -= residual_imu.y*(1<<5)*655;
  //  ahrs_impl.high_rez_bias.r -= residual_imu.z*(1<<5)*655;

  ahrs_impl.high_rez_bias.p -= residual_imu.x*1024;
  ahrs_impl.high_rez_bias.q -= residual_imu.y*1024;
  ahrs_impl.high_rez_bias.r -= residual_imu.z*1024;


  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);

}


/* Compute ltp to imu rotation in quaternion and rotation matrice representation
   from the euler angle representation */
__attribute__ ((always_inline)) static inline void compute_imu_quat_and_rmat_from_euler(void) {

  /* Compute LTP to IMU quaternion */
  INT32_QUAT_OF_EULERS(ahrs.ltp_to_imu_quat, ahrs.ltp_to_imu_euler);
  /* Compute LTP to IMU rotation matrix */
  INT32_RMAT_OF_EULERS(ahrs.ltp_to_imu_rmat, ahrs.ltp_to_imu_euler);

}

/* Compute ltp to imu rotation in euler angles and rotation matrice representation
   from the quaternion representation */
__attribute__ ((always_inline)) static inline void compute_imu_euler_and_rmat_from_quat(void) {

  /* Compute LTP to IMU euler */
  INT32_EULERS_OF_QUAT(ahrs.ltp_to_imu_euler, ahrs.ltp_to_imu_quat);
  /* Compute LTP to IMU rotation matrix */
  INT32_RMAT_OF_QUAT(ahrs.ltp_to_imu_rmat, ahrs.ltp_to_imu_quat);

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
  EULERS_FLOAT_OF_BFP(att,ahrs.ltp_to_imu_euler);

  estimator_phi   = att.phi;
  estimator_theta = att.theta;
  estimator_psi   = att.psi;

  //estimator_p = Omega_Vector[0];
}

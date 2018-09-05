/*
 * Copyright (C) 2017 Marton Brossard <martin.brossard@mines-paristech.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/ins/ins_mekf_wind.cpp
 *
 * Multiplicative Extended Kalman Filter in rotation matrix formulation.
 *
 * Estimate attitude, ground speed, position, gyro bias, accelerometer bias and wind speed.
 *
 * Using Eigen library
 */


#include "modules/ins/ins_mekf_wind.h"
#include "generated/airframe.h"

#ifndef SITL
// Redifine Eigen assert so it doesn't use memory allocation
#define eigen_assert(_cond) { if (!(_cond)) { while(1) ; } }
#endif

// Eigen headers
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

using namespace Eigen;

/** Covariance matrix elements and size
 */
enum MekfWindCovVar {
  MEKF_WIND_qx, MEKF_WIND_qy, MEKF_WIND_qz,
  MEKF_WIND_vx, MEKF_WIND_vy, MEKF_WIND_vz,
  MEKF_WIND_px, MEKF_WIND_py, MEKF_WIND_pz,
  MEKF_WIND_rbp, MEKF_WIND_rbq, MEKF_WIND_rbr,
  MEKF_WIND_abx, MEKF_WIND_aby, MEKF_WIND_abz,
  MEKF_WIND_bb,
  MEKF_WIND_wx, MEKF_WIND_wy, MEKF_WIND_wz,
  MEKF_WIND_COV_SIZE
};

typedef Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_COV_SIZE> MEKFWCov;

/** Process noise elements and size
 */
enum MekfWindPNoiseVar {
  MEKF_WIND_qgp, MEKF_WIND_qgq, MEKF_WIND_qgr,
  MEKF_WIND_qax, MEKF_WIND_qay, MEKF_WIND_qaz,
  MEKF_WIND_qrbp, MEKF_WIND_qrbq, MEKF_WIND_qrbr,
  MEKF_WIND_qabx, MEKF_WIND_qaby, MEKF_WIND_qabz,
  MEKF_WIND_qbb,
  MEKF_WIND_qwx, MEKF_WIND_qwy, MEKF_WIND_qwz,
  MEKF_WIND_PROC_NOISE_SIZE
};

typedef Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, MEKF_WIND_PROC_NOISE_SIZE> MEKFWPNoise;

/** Measurement noise elements and size
 */
enum MekfWindMNoiseVar {
  MEKF_WIND_rvx, MEKF_WIND_rvy, MEKF_WIND_rvz,
  MEKF_WIND_rpx, MEKF_WIND_rpy, MEKF_WIND_rpz,
  MEKF_WIND_rmx, MEKF_WIND_rmy, MEKF_WIND_rmz,
  MEKF_WIND_rb,
  MEKF_WIND_ras, MEKF_WIND_raoa, MEKF_WIND_raos,
  MEKF_WIND_MEAS_NOISE_SIZE
};

typedef Matrix<float, MEKF_WIND_MEAS_NOISE_SIZE, MEKF_WIND_MEAS_NOISE_SIZE> MEKFWMNoise;

/** filter state vector
 */
struct MekfWindState {
	Quaternionf quat;
	Vector3f speed;
	Vector3f pos;
	Vector3f accel;
	Vector3f rates_bias;
	Vector3f accel_bias;
  float baro_bias;
	Vector3f wind;
};

/** filter command vector
 */
struct MekfWindInputs {
	Vector3f rates;
	Vector3f accel;
};

/** filter measurement vector
 */
struct MekfWindMeasurements {
	Vector3f speed;
	Vector3f pos;
	Vector3f mag;
	float baro_alt;
	float airspeed;
	float aoa;
	float aos;
};

/** private filter structure
 */
struct InsMekfWindPrivate {
  struct MekfWindState state;
  struct MekfWindInputs inputs;
  struct MekfWindMeasurements measurements;

  MEKFWCov P;
  MEKFWPNoise Q;
  MEKFWMNoise R;

  /* earth magnetic model */
  Vector3f mag_h;
};


// Initial covariance parameters
#ifndef INS_MEKF_WIND_P0_QUAT
#define INS_MEKF_WIND_P0_QUAT       0.007615f
#endif
#ifndef INS_MEKF_WIND_P0_SPEED
#define INS_MEKF_WIND_P0_SPEED      1.E+2f
#endif
#ifndef INS_MEKF_WIND_P0_POS
#define INS_MEKF_WIND_P0_POS        1.E+1f
#endif
#ifndef INS_MEKF_WIND_P0_RATES_BIAS
#define INS_MEKF_WIND_P0_RATES_BIAS 1.E-3f
#endif
#ifndef INS_MEKF_WIND_P0_ACCEL_BIAS
#define INS_MEKF_WIND_P0_ACCEL_BIAS 1.E-3f
#endif
#ifndef INS_MEKF_WIND_P0_BARO_BIAS
#define INS_MEKF_WIND_P0_BARO_BIAS  1.E-3f
#endif
#ifndef INS_MEKF_WIND_P0_WIND
#define INS_MEKF_WIND_P0_WIND       1.E-0f
#endif

// Initial process noise parameters
#ifndef INS_MEKF_WIND_Q_GYRO
#define INS_MEKF_WIND_Q_GYRO        1.E-2f
#endif
#ifndef INS_MEKF_WIND_Q_ACCEL
#define INS_MEKF_WIND_Q_ACCEL       1.E-2f
#endif
#ifndef INS_MEKF_WIND_Q_RATES_BIAS
#define INS_MEKF_WIND_Q_RATES_BIAS  1.E-6f
#endif
#ifndef INS_MEKF_WIND_Q_ACCEL_BIAS
#define INS_MEKF_WIND_Q_ACCEL_BIAS  1.E-6f
#endif
#ifndef INS_MEKF_WIND_Q_BARO_BIAS
#define INS_MEKF_WIND_Q_BARO_BIAS   1.E-3f
#endif
#ifndef INS_MEKF_WIND_Q_WIND
#define INS_MEKF_WIND_Q_WIND        1.E+1f
#endif

// Initial measurements noise parameters
#ifndef INS_MEKF_WIND_R_SPEED
#define INS_MEKF_WIND_R_SPEED       0.1f
#endif
#ifndef INS_MEKF_WIND_R_SPEED_Z
#define INS_MEKF_WIND_R_SPEED_Z     0.2f
#endif
#ifndef INS_MEKF_WIND_R_POS
#define INS_MEKF_WIND_R_POS         2.0f
#endif
#ifndef INS_MEKF_WIND_R_POS_Z
#define INS_MEKF_WIND_R_POS_Z       4.0f
#endif
#ifndef INS_MEKF_WIND_R_MAG
#define INS_MEKF_WIND_R_MAG         1.f
#endif
#ifndef INS_MEKF_WIND_R_BARO
#define INS_MEKF_WIND_R_BARO        2.f
#endif
#ifndef INS_MEKF_WIND_R_AIRSPEED
#define INS_MEKF_WIND_R_AIRSPEED    0.1f
#endif
#ifndef INS_MEKF_WIND_R_AOA
#define INS_MEKF_WIND_R_AOA         0.1f
#endif
#ifndef INS_MEKF_WIND_R_AOS
#define INS_MEKF_WIND_R_AOS         0.1f
#endif

// Disable wind estimation by default
#ifndef INS_MEKF_WIND_DISABLE_WIND
#define INS_MEKF_WIND_DISABLE_WIND true
#endif

// paramters
struct ins_mekf_wind_parameters ins_mekf_wind_params;

// internal structure
static struct InsMekfWindPrivate mekf_wind_private;
// short name
#define mwp mekf_wind_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, 9.81f );


/* init state and measurements */
static void init_mekf_state(void)
{
  // init state
  mekf_wind_private.state.quat = Quaternionf::Identity();
  mekf_wind_private.state.speed = Vector3f::Zero();
  mekf_wind_private.state.pos = Vector3f::Zero();
  mekf_wind_private.state.rates_bias = Vector3f::Zero();
  mekf_wind_private.state.accel_bias = Vector3f::Zero();
  mekf_wind_private.state.baro_bias = 0.f;
  mekf_wind_private.state.wind = Vector3f::Zero();

  // init measures
  mekf_wind_private.measurements.speed = Vector3f::Zero();
  mekf_wind_private.measurements.pos = Vector3f::Zero();
  mekf_wind_private.measurements.mag = Vector3f::Zero();
  mekf_wind_private.measurements.baro_alt = 0.f;
  mekf_wind_private.measurements.airspeed = 0.f;
  mekf_wind_private.measurements.aoa = 0.f;
  mekf_wind_private.measurements.aos = 0.f;

  // init input
  mekf_wind_private.inputs.rates = Vector3f::Zero();
  mekf_wind_private.inputs.accel = Vector3f::Zero();

  // init state covariance
  mekf_wind_private.P = MEKFWCov::Zero();
  mekf_wind_private.P(MEKF_WIND_qx,MEKF_WIND_qx) = INS_MEKF_WIND_P0_QUAT;
  mekf_wind_private.P(MEKF_WIND_qy,MEKF_WIND_qy) = INS_MEKF_WIND_P0_QUAT;
  mekf_wind_private.P(MEKF_WIND_qz,MEKF_WIND_qz) = INS_MEKF_WIND_P0_QUAT;
  mekf_wind_private.P(MEKF_WIND_vx,MEKF_WIND_vx) = INS_MEKF_WIND_P0_SPEED;
  mekf_wind_private.P(MEKF_WIND_vy,MEKF_WIND_vy) = INS_MEKF_WIND_P0_SPEED;
  mekf_wind_private.P(MEKF_WIND_vz,MEKF_WIND_vz) = INS_MEKF_WIND_P0_SPEED;
  mekf_wind_private.P(MEKF_WIND_px,MEKF_WIND_px) = INS_MEKF_WIND_P0_POS;
  mekf_wind_private.P(MEKF_WIND_py,MEKF_WIND_py) = INS_MEKF_WIND_P0_POS;
  mekf_wind_private.P(MEKF_WIND_pz,MEKF_WIND_pz) = INS_MEKF_WIND_P0_POS;
  mekf_wind_private.P(MEKF_WIND_rbp,MEKF_WIND_rbp) = INS_MEKF_WIND_P0_RATES_BIAS;
  mekf_wind_private.P(MEKF_WIND_rbq,MEKF_WIND_rbq) = INS_MEKF_WIND_P0_RATES_BIAS;
  mekf_wind_private.P(MEKF_WIND_rbr,MEKF_WIND_rbr) = INS_MEKF_WIND_P0_RATES_BIAS;
  mekf_wind_private.P(MEKF_WIND_abx,MEKF_WIND_abx) = INS_MEKF_WIND_P0_ACCEL_BIAS;
  mekf_wind_private.P(MEKF_WIND_aby,MEKF_WIND_aby) = INS_MEKF_WIND_P0_ACCEL_BIAS;
  mekf_wind_private.P(MEKF_WIND_abz,MEKF_WIND_abz) = INS_MEKF_WIND_P0_ACCEL_BIAS;
  mekf_wind_private.P(MEKF_WIND_bb,MEKF_WIND_bb) = INS_MEKF_WIND_P0_BARO_BIAS;
  mekf_wind_private.P(MEKF_WIND_wx,MEKF_WIND_wx) = INS_MEKF_WIND_P0_WIND;
  mekf_wind_private.P(MEKF_WIND_wy,MEKF_WIND_wy) = INS_MEKF_WIND_P0_WIND;
  mekf_wind_private.P(MEKF_WIND_wz,MEKF_WIND_wz) = INS_MEKF_WIND_P0_WIND;

  // init process and measurements noise
  ins_mekf_wind_update_params();
}

// Some matrix and quaternion utility functions
static Quaternionf quat_add(const Quaternionf& q1, const Quaternionf& q2) {
  return Quaternionf(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
}

static Quaternionf quat_smul(const Quaternionf& q1, float scal) {
  return Quaternionf(q1.w() * scal, q1.x() * scal, q1.y() * scal, q1.z() * scal);
}

/**
 * build skew symetric matrix from vector
 * m = [     0, -v(2),  v(1) ]
 *     [  v(2),     0, -v(0) ]
 *     [ -v(1),  v(0),     0 ]
 */
static Matrix3f skew_sym(const Vector3f& v) {
  Matrix3f m = Matrix3f::Zero();
  m(0,1) = -v(2);
  m(0,2) = v(1);
  m(1,0) = v(2);
  m(1,2) = -v(0);
  m(2,0) = -v(1);
  m(2,1) = v(0);
  return m;
}

/**
 * Init function
 */
void ins_mekf_wind_init(void)
{
  // init parameters
  ins_mekf_wind_params.Q_gyro       = INS_MEKF_WIND_Q_GYRO;
  ins_mekf_wind_params.Q_accel      = INS_MEKF_WIND_Q_ACCEL;
  ins_mekf_wind_params.Q_rates_bias = INS_MEKF_WIND_Q_RATES_BIAS;
  ins_mekf_wind_params.Q_accel_bias = INS_MEKF_WIND_Q_ACCEL_BIAS;
  ins_mekf_wind_params.Q_baro_bias  = INS_MEKF_WIND_Q_BARO_BIAS;
  ins_mekf_wind_params.Q_wind       = INS_MEKF_WIND_Q_WIND;
  ins_mekf_wind_params.R_speed      = INS_MEKF_WIND_R_SPEED;
  ins_mekf_wind_params.R_speed_z    = INS_MEKF_WIND_R_SPEED_Z;
  ins_mekf_wind_params.R_pos        = INS_MEKF_WIND_R_POS;
  ins_mekf_wind_params.R_pos_z      = INS_MEKF_WIND_R_POS_Z;
  ins_mekf_wind_params.R_mag        = INS_MEKF_WIND_R_MAG;
  ins_mekf_wind_params.R_baro       = INS_MEKF_WIND_R_BARO;
  ins_mekf_wind_params.R_airspeed   = INS_MEKF_WIND_R_AIRSPEED;
  ins_mekf_wind_params.R_aoa        = INS_MEKF_WIND_R_AOA;
  ins_mekf_wind_params.R_aos        = INS_MEKF_WIND_R_AOS;
  ins_mekf_wind_params.disable_wind = INS_MEKF_WIND_DISABLE_WIND;

  // init state and measurements
  init_mekf_state();

  // init local earth magnetic field
  mekf_wind_private.mag_h = Vector3f(1.0f, 0.f, 0.f);
}

void ins_mekf_wind_set_mag_h(const struct FloatVect3 *mag_h)
{
  // update local earth magnetic field
  mekf_wind_private.mag_h(0) = mag_h->x;
  mekf_wind_private.mag_h(1) = mag_h->y;
  mekf_wind_private.mag_h(2) = mag_h->z;
}

void ins_mekf_wind_reset(void)
{
  init_mekf_state();
}

/** Full INS propagation
 */
void ins_mekf_wind_propagate(struct FloatRates *gyro, struct FloatVect3 *acc, float dt)
{
  Quaternionf q_tmp;

  mekf_wind_private.inputs.rates = Vector3f(gyro->p, gyro->q, gyro->r);
  mekf_wind_private.inputs.accel = Vector3f(acc->x, acc->y, acc->z);

  const Vector3f gyro_unbiased = mwp.inputs.rates - mwp.state.rates_bias;
  const Vector3f accel_unbiased = mwp.inputs.accel - mwp.state.accel_bias;
  // propagate state
  // q_dot = 1/2 q * (rates - rates_bias)
  q_tmp.w() = 0.f;
  q_tmp.vec() = gyro_unbiased;
  const Quaternionf q_d = quat_smul(mwp.state.quat * q_tmp, 0.5f);
  // speed_d = q * (accel - accel_bias) * q^-1 + g
  q_tmp.vec() = accel_unbiased;
  // store NED accel
  mwp.state.accel = (mwp.state.quat * q_tmp * mwp.state.quat.inverse()).vec() + gravity;

  // Euler integration

  //mwp.state.quat = (mwp.state.quat + q_d * dt).normalize();
  mwp.state.quat = quat_add(mwp.state.quat, quat_smul(q_d, dt));
  mwp.state.quat.normalize();
  mwp.state.speed = mwp.state.speed + mwp.state.accel * dt;
  mwp.state.pos = mwp.state.pos + mwp.state.speed * dt;

  // propagate covariance
  const Matrix3f Rq = mwp.state.quat.toRotationMatrix();
  const Matrix3f Rqdt = Rq * dt;
  const Matrix3f RqA = skew_sym(Rq * accel_unbiased);
  const Matrix3f RqAdt = RqA * dt;
  const Matrix3f RqAdt2 = RqAdt * dt;

  MEKFWCov A = MEKFWCov::Identity();
  A.block<3,3>(MEKF_WIND_qx,MEKF_WIND_rbp) = -Rqdt;
  A.block<3,3>(MEKF_WIND_vx,MEKF_WIND_qx) = -RqAdt;
  A.block<3,3>(MEKF_WIND_vx,MEKF_WIND_rbp) = RqAdt2;
  A.block<3,3>(MEKF_WIND_vx,MEKF_WIND_abx) = -Rqdt;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_qx) = -RqAdt2;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_vx) = Matrix3f::Identity() * dt;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_rbp) = RqAdt2 * dt;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_abx) = -Rqdt * dt;

  Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_PROC_NOISE_SIZE> An;
  An.setZero();
  An.block<3,3>(MEKF_WIND_qx,MEKF_WIND_qgp) = Rq;
  An.block<3,3>(MEKF_WIND_vx,MEKF_WIND_qax) = Rq;
  An.block<3,3>(MEKF_WIND_rbp,MEKF_WIND_qrbp) = Matrix3f::Identity();
  An.block<3,3>(MEKF_WIND_abx,MEKF_WIND_qabx) = Matrix3f::Identity();
  An(MEKF_WIND_bb,MEKF_WIND_qbb) = 1.0f;
  An.block<3,3>(MEKF_WIND_wx,MEKF_WIND_qwx) = Matrix3f::Identity();

  MEKFWCov At(A);
  At.transposeInPlace();
  Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, MEKF_WIND_COV_SIZE> Ant;
  Ant = An.transpose();

  mwp.P = A * mwp.P * At + An * mwp.Q * Ant * dt;

  if (ins_mekf_wind_params.disable_wind) {
    mwp.P.block<3,MEKF_WIND_COV_SIZE>(MEKF_WIND_wx,0) = Matrix<float,3,MEKF_WIND_COV_SIZE>::Zero();
    mwp.P.block<MEKF_WIND_COV_SIZE-3,3>(0,MEKF_WIND_wx) = Matrix<float,MEKF_WIND_COV_SIZE-3,3>::Zero();
    mwp.P(MEKF_WIND_wx,MEKF_WIND_wx) = INS_MEKF_WIND_P0_WIND;
    mwp.P(MEKF_WIND_wy,MEKF_WIND_wy) = INS_MEKF_WIND_P0_WIND;
    mwp.P(MEKF_WIND_wz,MEKF_WIND_wz) = INS_MEKF_WIND_P0_WIND;
    mwp.state.wind = Vector3f::Zero();
  }
}

/** AHRS-only propagation + accel correction
 */
void ins_mekf_wind_propagate_ahrs(struct FloatRates *gyro, struct FloatVect3 *acc, float dt)
{
  Quaternionf q_tmp;

  mekf_wind_private.inputs.rates = Vector3f(gyro->p, gyro->q, gyro->r);
  mekf_wind_private.inputs.accel = Vector3f(acc->x, acc->y, acc->z);

  const Vector3f gyro_unbiased = mwp.inputs.rates - mwp.state.rates_bias;
  const Vector3f accel_unbiased = mwp.inputs.accel - mwp.state.accel_bias;
  // propagate state
  // q_dot = 1/2 q * (rates - rates_bias)
  q_tmp.w() = 0.f;
  q_tmp.vec() = gyro_unbiased;
  const Quaternionf q_d = quat_smul(mwp.state.quat * q_tmp, 0.5f);

  // Euler integration

  //mwp.state.quat = (mwp.state.quat + q_d * dt).normalize();
  mwp.state.quat = quat_add(mwp.state.quat, quat_smul(q_d, dt));
  mwp.state.quat.normalize();

  // propagate covariance
  const Matrix3f Rq = mwp.state.quat.toRotationMatrix();
  const Matrix3f Rqdt = Rq * dt;

  MEKFWCov A = MEKFWCov::Zero();
  A.block<3,3>(MEKF_WIND_qx,MEKF_WIND_qx) = Matrix3f::Identity();
  A.block<3,3>(MEKF_WIND_qx,MEKF_WIND_rbp) = -Rqdt;
  A.block<3,3>(MEKF_WIND_rbp,MEKF_WIND_rbp) = Matrix3f::Identity();

  Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_PROC_NOISE_SIZE> An;
  An.setZero();
  An.block<3,3>(MEKF_WIND_qx,MEKF_WIND_qgp) = Rq;
  An.block<3,3>(MEKF_WIND_rbp,MEKF_WIND_qrbp) = Matrix3f::Identity(); // TODO check index

  MEKFWCov At(A);
  At.transposeInPlace();
  Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, MEKF_WIND_COV_SIZE> Ant;
  Ant = An.transpose();

  mwp.P = A * mwp.P * At + An * mwp.Q * Ant * dt;

  // correction from accel measurements
  const Matrix3f Rqt = Rq.transpose();
  Matrix<float, 3, MEKF_WIND_COV_SIZE> H = Matrix<float, 3, MEKF_WIND_COV_SIZE>::Zero();
  H.block<3,3>(0,0) = - Rqt * skew_sym(gravity);
  Matrix<float, MEKF_WIND_COV_SIZE, 3> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix3f S = H * mwp.P * Ht + mwp.R.block<3,3>(MEKF_WIND_rmx,MEKF_WIND_rmx); // FIXME currently abusing mag noise ????
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 3> K = mwp.P * Ht * S.inverse();
  // Residual z_a - h(z)
  Vector3f res = accel_unbiased + (Rqt * gravity);
  // Update state
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,3>(MEKF_WIND_qx,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.rates_bias  += K.block<3,3>(MEKF_WIND_rbp,0) * res;
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}


void ins_mekf_wind_align(struct FloatRates *gyro_bias, struct FloatQuat *quat)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  mwp.state.quat.w() = quat->qi;
  mwp.state.quat.x() = quat->qx;
  mwp.state.quat.y() = quat->qy;
  mwp.state.quat.z() = quat->qz;

  /* use average gyro as initial value for bias */
  mwp.state.rates_bias(0) = gyro_bias->p;
  mwp.state.rates_bias(1) = gyro_bias->q;
  mwp.state.rates_bias(2) = gyro_bias->r;
}

void ins_mekf_wind_update_mag(struct FloatVect3* mag, bool attitude_only)
{
  mwp.measurements.mag(0) = mag->x;
  mwp.measurements.mag(1) = mag->y;
  mwp.measurements.mag(2) = mag->z;

  // H and Ht matrices
  const Matrix3f Rqt = mwp.state.quat.toRotationMatrix().transpose();
  Matrix<float, 3, MEKF_WIND_COV_SIZE> H = Matrix<float, 3, MEKF_WIND_COV_SIZE>::Zero();
  H.block<3,3>(0,0) = Rqt * skew_sym(mwp.mag_h);
  Matrix<float, MEKF_WIND_COV_SIZE, 3> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix3f S = H * mwp.P * Ht + mwp.R.block<3,3>(MEKF_WIND_rmx,MEKF_WIND_rmx);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 3> K = mwp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Vector3f res = mwp.measurements.mag - (Rqt * mwp.mag_h);
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,3>(MEKF_WIND_qx,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  if (attitude_only) {
    mwp.state.rates_bias  += K.block<3,3>(MEKF_WIND_rbp,0) * res;
  } else {
    mwp.state.speed       += K.block<3,3>(MEKF_WIND_vx,0) * res;
    mwp.state.pos         += K.block<3,3>(MEKF_WIND_px,0) * res;
    mwp.state.rates_bias  += K.block<3,3>(MEKF_WIND_rbp,0) * res;
    mwp.state.accel_bias  += K.block<3,3>(MEKF_WIND_abx,0) * res;
    mwp.state.baro_bias   += K.block<1,3>(MEKF_WIND_bb,0) * res;
    if (!ins_mekf_wind_params.disable_wind) {
      mwp.state.wind        += K.block<3,3>(MEKF_WIND_wx,0) * res;
    }
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_baro(float baro_alt)
{
  mwp.measurements.baro_alt = baro_alt;

  // H and Ht matrices
  Matrix<float, 1, MEKF_WIND_COV_SIZE> H = Matrix<float, 1, MEKF_WIND_COV_SIZE>::Zero();
  H(0,MEKF_WIND_pz) = 1.0f; // TODO check index
  H(0,MEKF_WIND_bb) = -1.0f;
  Matrix<float, MEKF_WIND_COV_SIZE, 1> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt -> only pos.z component
  float S = mwp.P(MEKF_WIND_pz,MEKF_WIND_pz) - mwp.P(MEKF_WIND_bb,MEKF_WIND_bb) + mwp.R(MEKF_WIND_rb,MEKF_WIND_rb);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 1> K = mwp.P * Ht / S;
  // Residual z_m - h(z)
  float res = mwp.measurements.baro_alt - (mwp.state.pos(2) - mwp.state.baro_bias);
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,1>(MEKF_WIND_qx,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,1>(MEKF_WIND_vx,0) * res;
  mwp.state.pos         += K.block<3,1>(MEKF_WIND_px,0) * res;
  mwp.state.rates_bias  += K.block<3,1>(MEKF_WIND_rbp,0) * res;
  mwp.state.accel_bias  += K.block<3,1>(MEKF_WIND_abx,0) * res;
  mwp.state.baro_bias   += K(MEKF_WIND_bb,0) * res;
  if (!ins_mekf_wind_params.disable_wind) {
    mwp.state.wind        += K.block<3,1>(MEKF_WIND_wx,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_pos_speed(struct FloatVect3 *pos, struct FloatVect3 *speed)
{
  mwp.measurements.pos(0) = pos->x;
  mwp.measurements.pos(1) = pos->y;
  mwp.measurements.pos(2) = pos->z;
  mwp.measurements.speed(0) = speed->x;
  mwp.measurements.speed(1) = speed->y;
  mwp.measurements.speed(2) = speed->z;

  // H and Ht matrices
  Matrix<float, 6, MEKF_WIND_COV_SIZE> H = Matrix<float, 6, MEKF_WIND_COV_SIZE>::Zero();
  H.block<6,6>(0,MEKF_WIND_vx) = Matrix<float,6,6>::Identity();
  Matrix<float, MEKF_WIND_COV_SIZE, 6> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix<float, 6, 6> S = mwp.P.block<6,6>(MEKF_WIND_vx,MEKF_WIND_vx) + mwp.R.block<6,6>(MEKF_WIND_rvx,MEKF_WIND_rvx);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 6> K = mwp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Matrix<float, 6, 1> res = Matrix<float, 6, 1>::Zero();
  res.block<3,1>(0,0) = mwp.measurements.speed - mwp.state.speed;
  res.block<3,1>(3,0) = mwp.measurements.pos - mwp.state.pos;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,6>(MEKF_WIND_qx,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.speed       += K.block<3,6>(MEKF_WIND_vx,0) * res;
  mwp.state.pos         += K.block<3,6>(MEKF_WIND_px,0) * res;
  mwp.state.rates_bias  += K.block<3,6>(MEKF_WIND_rbp,0) * res;
  mwp.state.accel_bias  += K.block<3,6>(MEKF_WIND_abx,0) * res;
  mwp.state.baro_bias   += K.block<1,6>(MEKF_WIND_bb,0) * res;
  if (!ins_mekf_wind_params.disable_wind) {
    mwp.state.wind        += K.block<3,6>(MEKF_WIND_wx,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_airspeed(float airspeed)
{
  mwp.measurements.airspeed = airspeed;

  if (ins_mekf_wind_params.disable_wind) return;
  // H and Ht matrices
  const RowVector3f IuRqt = mwp.state.quat.toRotationMatrix().transpose().block<1,3>(0,0);
  const Vector3f va = mwp.state.speed - mwp.state.wind;
  Matrix<float, 1, MEKF_WIND_COV_SIZE> H = Matrix<float, 1, MEKF_WIND_COV_SIZE>::Zero();
  H.block<1,3>(0,MEKF_WIND_qx) = IuRqt * skew_sym(va);
  H.block<1,3>(0,MEKF_WIND_vx) = IuRqt;
  H.block<1,3>(0,MEKF_WIND_wx) = -IuRqt;
  Matrix<float, MEKF_WIND_COV_SIZE, 1> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  float S = H * mwp.P * Ht + mwp.R(MEKF_WIND_ras,MEKF_WIND_ras);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 1> K = mwp.P * Ht / S;
  // Residual z_m - h(z)
  float res = mwp.measurements.airspeed - IuRqt * va;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,1>(MEKF_WIND_qx,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,1>(MEKF_WIND_vx,0) * res;
  mwp.state.pos         += K.block<3,1>(MEKF_WIND_px,0) * res;
  mwp.state.rates_bias  += K.block<3,1>(MEKF_WIND_rbp,0) * res;
  mwp.state.accel_bias  += K.block<3,1>(MEKF_WIND_abx,0) * res;
  mwp.state.baro_bias   += K(MEKF_WIND_bb,0) * res;
  if (!ins_mekf_wind_params.disable_wind) {
    mwp.state.wind        += K.block<3,1>(MEKF_WIND_wx,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_incidence(float aoa, float aos)
{
  mwp.measurements.aoa = aoa;
  mwp.measurements.aos = aos;

  if (ins_mekf_wind_params.disable_wind) return;
  // H and Ht matrices
  const Matrix3f Rqt = mwp.state.quat.toRotationMatrix().transpose();
  const Vector3f va = Rqt * (mwp.state.speed - mwp.state.wind); // airspeed in body frame
  // check if data in valid range
  const float van = va.norm();
  //const float va_bound = 0.2f * van;
  if (van < 5.f /*|| va(0) < 0.8f * van
      || va(1) < - va_bound || va(1) > va_bound
      || va(2) < - va_bound || va(2) > va_bound*/
      || mwp.state.pos(2) > -10.f) {
    // filter doesn't work at zero airspeed
    return;
  }
  const RowVector3f C(sinf(aoa), 0.f, -cosf(aoa));
  const RowVector3f CRqt = C * Rqt;
  const float s_aos = sinf(aos);
  const float c_aos = cosf(aos);
  const Matrix3f B = Vector3f(s_aos * s_aos, - c_aos * c_aos, 0.f).asDiagonal();
  const RowVector3f vBRqt = 2.f * va.transpose() * B * Rqt;
  Matrix<float, 2, MEKF_WIND_COV_SIZE> H = Matrix<float, 2, MEKF_WIND_COV_SIZE>::Zero();
  H.block<1,3>(0,MEKF_WIND_qx) = CRqt * skew_sym(mwp.state.speed - mwp.state.wind);
  H.block<1,3>(0,MEKF_WIND_vx) = CRqt;
  H.block<1,3>(0,MEKF_WIND_wx) = -CRqt;
  H.block<1,3>(1,MEKF_WIND_qx) = vBRqt * skew_sym(mwp.state.speed - mwp.state.wind);
  H.block<1,3>(1,MEKF_WIND_vx) = vBRqt;
  H.block<1,3>(1,MEKF_WIND_wx) = -vBRqt;
  Matrix<float, MEKF_WIND_COV_SIZE, 2> Ht = H.transpose();
  // Hn and Hnt matrices
  Matrix2f Hn = Matrix2f::Identity();
  Hn(0,0) = C(2) * va(0) - C(0) * va(2);
  const float s_2aos = sinf(2.0f * aos);
  Hn(1,1) = (RowVector3f(-s_2aos, 0.f, s_2aos) * va.asDiagonal()) * va;
  Matrix2f Hnt = Hn.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix2f S = H * mwp.P * Ht + Hn * mwp.R.block<2,2>(MEKF_WIND_raoa,MEKF_WIND_raoa) * Hnt;
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 2> K = mwp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Vector2f res = Vector2f::Zero();
  res(0) = - C * va;
  res(1) = - va.transpose() * B * va;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,2>(MEKF_WIND_qx,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,2>(MEKF_WIND_vx,0) * res;
  mwp.state.pos         += K.block<3,2>(MEKF_WIND_px,0) * res;
  mwp.state.rates_bias  += K.block<3,2>(MEKF_WIND_rbp,0) * res;
  mwp.state.accel_bias  += K.block<3,2>(MEKF_WIND_abx,0) * res;
  mwp.state.baro_bias   += K.block<1,2>(MEKF_WIND_bb,0) * res;
  if (!ins_mekf_wind_params.disable_wind) {
    mwp.state.wind        += K.block<3,2>(MEKF_WIND_wx,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

/**
 * Getter/Setter functions
 */
struct NedCoor_f ins_mekf_wind_get_pos_ned(void)
{
  const struct NedCoor_f p = {
    .x = mwp.state.pos(0),
    .y = mwp.state.pos(1),
    .z = mwp.state.pos(2)
  };
  return p;
}

void ins_mekf_wind_set_pos_ned(struct NedCoor_f *p)
{
  mwp.state.pos(0) = p->x;
  mwp.state.pos(1) = p->y;
  mwp.state.pos(2) = p->z;
}

struct NedCoor_f ins_mekf_wind_get_speed_ned(void)
{
  const struct NedCoor_f s = {
    .x = mwp.state.speed(0),
    .y = mwp.state.speed(1),
    .z = mwp.state.speed(2)
  };
  return s;
}

void ins_mekf_wind_set_speed_ned(struct NedCoor_f *s)
{
  mwp.state.speed(0) = s->x;
  mwp.state.speed(1) = s->y;
  mwp.state.speed(2) = s->z;
}

struct NedCoor_f ins_mekf_wind_get_accel_ned(void)
{
  const struct NedCoor_f a = {
    .x = mwp.state.accel(0),
    .y = mwp.state.accel(1),
    .z = mwp.state.accel(2)
  };
  return a;
}

struct FloatQuat ins_mekf_wind_get_quat(void)
{
  const struct FloatQuat q = {
    .qi = mwp.state.quat.w(),
    .qx = mwp.state.quat.x(),
    .qy = mwp.state.quat.y(),
    .qz = mwp.state.quat.z()
  };
  return q;
}

void ins_mekf_wind_set_quat(struct FloatQuat *quat)
{
  mwp.state.quat.w() = quat->qi;
  mwp.state.quat.x() = quat->qx;
  mwp.state.quat.y() = quat->qy;
  mwp.state.quat.z() = quat->qz;
}

struct FloatRates ins_mekf_wind_get_body_rates(void)
{
  const struct FloatRates r = {
    .p = mwp.inputs.rates(0) - mwp.state.rates_bias(0),
    .q = mwp.inputs.rates(1) - mwp.state.rates_bias(1),
    .r = mwp.inputs.rates(2) - mwp.state.rates_bias(2)
  };
  return r;
}

struct NedCoor_f ins_mekf_wind_get_wind_ned(void)
{
  const struct NedCoor_f w = {
    .x = mwp.state.wind(0),
    .y = mwp.state.wind(1),
    .z = mwp.state.wind(2)
  };
  return w;
}

struct NedCoor_f ins_mekf_wind_get_airspeed_body(void)
{
  const Matrix3f Rqt = mwp.state.quat.toRotationMatrix().transpose();
  const Vector3f va = Rqt * (mwp.state.speed - mwp.state.wind); // airspeed in body frame
  const struct NedCoor_f a = {
    .x = va(0),
    .y = va(1),
    .z = va(2)
  };
  return a;
}

float ins_mekf_wind_get_airspeed_norm(void)
{
  return (mwp.state.speed - mwp.state.wind).norm();
}

struct FloatVect3 ins_mekf_wind_get_accel_bias(void)
{
  const struct FloatVect3 ab = {
    .x = mwp.state.accel_bias(0),
    .y = mwp.state.accel_bias(1),
    .z = mwp.state.accel_bias(2)
  };
  return ab;
}

struct FloatRates ins_mekf_wind_get_rates_bias(void)
{
  const struct FloatRates rb = {
    .p = mwp.state.rates_bias(0),
    .q = mwp.state.rates_bias(1),
    .r = mwp.state.rates_bias(2)
  };
  return rb;
}

float ins_mekf_wind_get_baro_bias(void)
{
  return mwp.state.baro_bias;
}

void ins_mekf_wind_update_params(void)
{
  Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, 1> vp;
  vp(MEKF_WIND_qgp) = vp(MEKF_WIND_qgq) = vp(MEKF_WIND_qgr) = ins_mekf_wind_params.Q_gyro;
  vp(MEKF_WIND_qax) = vp(MEKF_WIND_qay) = vp(MEKF_WIND_qaz) = ins_mekf_wind_params.Q_accel;
  vp(MEKF_WIND_qrbp) = vp(MEKF_WIND_qrbq) = vp(MEKF_WIND_qrbr) = ins_mekf_wind_params.Q_rates_bias;
  vp(MEKF_WIND_qabx) = vp(MEKF_WIND_qaby) = vp(MEKF_WIND_qabz) = ins_mekf_wind_params.Q_accel_bias;
  vp(MEKF_WIND_qbb) = ins_mekf_wind_params.Q_baro_bias;
  vp(MEKF_WIND_qwx) = vp(MEKF_WIND_qwy) = vp(MEKF_WIND_qwz) = ins_mekf_wind_params.Q_wind;
  mekf_wind_private.Q = vp.asDiagonal();

  Matrix<float, MEKF_WIND_MEAS_NOISE_SIZE, 1> vm;
  vm(MEKF_WIND_rvx) = vm(MEKF_WIND_rvy) = ins_mekf_wind_params.R_speed;
  vm(MEKF_WIND_rvz) = ins_mekf_wind_params.R_speed_z;
  vm(MEKF_WIND_rpx) = vm(MEKF_WIND_rpy) = ins_mekf_wind_params.R_pos;
  vm(MEKF_WIND_rpz) = ins_mekf_wind_params.R_pos_z;
  vm(MEKF_WIND_rmx) = vm(MEKF_WIND_rmy) = vm(MEKF_WIND_rmz) = ins_mekf_wind_params.R_mag;
  vm(MEKF_WIND_rb) = ins_mekf_wind_params.R_baro;
  vm(MEKF_WIND_ras) = ins_mekf_wind_params.R_airspeed;
  vm(MEKF_WIND_raoa) = ins_mekf_wind_params.R_aoa;
  vm(MEKF_WIND_raos) = ins_mekf_wind_params.R_aos;
  mekf_wind_private.R = vm.asDiagonal();
}


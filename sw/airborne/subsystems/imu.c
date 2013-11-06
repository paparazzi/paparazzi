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
 * @file subsystems/imu.c
 * Inertial Measurement Unit interface.
 */

#include "subsystems/imu.h"

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

#if USE_IMU_FLOAT

static void send_accel(void) {
  DOWNLINK_SEND_IMU_ACCEL(DefaultChannel, DefaultDevice,
      &imuf.accel.x, &imuf.accel.y, &imuf.accel.z);
}

static void send_gyro(void) {
  DOWNLINK_SEND_IMU_GYRO(DefaultChannel, DefaultDevice,
      &imuf.gyro.p, &imuf.gyro.q, &imuf.gyro.r);
}

#else // !USE_IMU_FLOAT

static void send_accel_raw(void) {
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,
      &imu.accel_unscaled.x, &imu.accel_unscaled.y, &imu.accel_unscaled.z);
}

static void send_accel_scaled(void) {
  DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice,
      &imu.accel.x, &imu.accel.y, &imu.accel.z);
}

static void send_accel(void) {
  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);
  DOWNLINK_SEND_IMU_ACCEL(DefaultChannel, DefaultDevice,
      &accel_float.x, &accel_float.y, &accel_float.z);
}

static void send_gyro_raw(void) {
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,
      &imu.gyro_unscaled.p, &imu.gyro_unscaled.q, &imu.gyro_unscaled.r);
}

static void send_gyro_scaled(void) {
  DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice,
      &imu.gyro.p, &imu.gyro.q, &imu.gyro.r);
}

static void send_gyro(void) {
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro);
  DOWNLINK_SEND_IMU_GYRO(DefaultChannel, DefaultDevice,
      &gyro_float.p, &gyro_float.q, &gyro_float.r);
}

static void send_mag_raw(void) {
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
      &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z);
}

static void send_mag_scaled(void) {
  DOWNLINK_SEND_IMU_MAG_SCALED(DefaultChannel, DefaultDevice,
      &imu.mag.x, &imu.mag.y, &imu.mag.z);
}

static void send_mag(void) {
  struct FloatVect3 mag_float;
  MAGS_FLOAT_OF_BFP(mag_float, imu.mag);
  DOWNLINK_SEND_IMU_MAG(DefaultChannel, DefaultDevice,
      &mag_float.x, &mag_float.y, &mag_float.z);
}
#endif // !USE_IMU_FLOAT

#endif

struct Imu imu;
struct ImuFloat imuf;

void imu_init(void) {

  /* initialises neutrals */
  RATES_ASSIGN(imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);

  VECT3_ASSIGN(imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);

#if defined IMU_MAG_X_NEUTRAL && defined IMU_MAG_Y_NEUTRAL && defined IMU_MAG_Z_NEUTRAL
  VECT3_ASSIGN(imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);
#else
#if USE_MAGNETOMETER
INFO("Magnetometer neutrals are set to zero, you should calibrate!")
#endif
  INT_VECT3_ZERO(imu.mag_neutral);
#endif

  /*
    Compute quaternion and rotation matrix
    for conversions between body and imu frame
  */
  struct Int32Eulers body_to_imu_eulers =
    { ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_PHI),
      ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_THETA),
      ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_PSI) };
  INT32_QUAT_OF_EULERS(imu.body_to_imu_quat, body_to_imu_eulers);
  INT32_QUAT_NORMALIZE(imu.body_to_imu_quat);
  INT32_RMAT_OF_EULERS(imu.body_to_imu_rmat, body_to_imu_eulers);

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "IMU_ACCEL", send_accel);
  register_periodic_telemetry(DefaultPeriodic, "IMU_GYRO", send_gyro);
#if USE_IMU_FLOAT
#else // !USE_IMU_FLOAT
  register_periodic_telemetry(DefaultPeriodic, "IMU_ACCEL_RAW", send_accel_raw);
  register_periodic_telemetry(DefaultPeriodic, "IMU_ACCEL_SCALED", send_accel_scaled);
  register_periodic_telemetry(DefaultPeriodic, "IMU_ACCEL", send_accel);
  register_periodic_telemetry(DefaultPeriodic, "IMU_GYRO_RAW", send_gyro_raw);
  register_periodic_telemetry(DefaultPeriodic, "IMU_GYRO_SCALED", send_gyro_scaled);
  register_periodic_telemetry(DefaultPeriodic, "IMU_GYRO", send_gyro);
  register_periodic_telemetry(DefaultPeriodic, "IMU_MAG_RAW", send_mag_raw);
  register_periodic_telemetry(DefaultPeriodic, "IMU_MAG_SCALED", send_mag_scaled);
  register_periodic_telemetry(DefaultPeriodic, "IMU_MAG", send_mag);
#endif // !USE_IMU_FLOAT
#endif // DOWNLINK

  imu_impl_init();
}


void imu_float_init(void) {
  /*
    Compute quaternion and rotation matrix
    for conversions between body and imu frame
  */
  EULERS_ASSIGN(imuf.body_to_imu_eulers,
		IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI);
  FLOAT_QUAT_OF_EULERS(imuf.body_to_imu_quat, imuf.body_to_imu_eulers);
  FLOAT_QUAT_NORMALIZE(imuf.body_to_imu_quat);
  FLOAT_RMAT_OF_EULERS(imuf.body_to_imu_rmat, imuf.body_to_imu_eulers);
}

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

#include BOARD_CONFIG
#include "subsystems/imu.h"
#include "state.h"

#ifdef IMU_POWER_GPIO
#include "mcu_periph/gpio.h"

#ifndef IMU_POWER_GPIO_ON
#define IMU_POWER_GPIO_ON gpio_set
#endif
#endif

#if PERIODIC_TELEMETRY
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

#ifdef IMU_POWER_GPIO
  gpio_setup_output(IMU_POWER_GPIO);
  IMU_POWER_GPIO_ON(IMU_POWER_GPIO);
#endif

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

  struct FloatEulers body_to_imu_eulers =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "IMU_ACCEL", send_accel);
  register_periodic_telemetry(DefaultPeriodic, "IMU_GYRO", send_gyro);
#if !USE_IMU_FLOAT
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
  struct FloatEulers body_to_imu_eulers =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  orientationSetEulers_f(&imuf.body_to_imu, &body_to_imu_eulers);
}

void imu_SetBodyToImuPhi(float phi) {
  struct FloatEulers imu_to_body_eulers;
  memcpy(&imu_to_body_eulers, orientationGetEulers_f(&imu.body_to_imu), sizeof(struct FloatEulers));
  imu_to_body_eulers.phi = phi;
  orientationSetEulers_f(&imu.body_to_imu, &imu_to_body_eulers);
}

void imu_SetBodyToImuTheta(float theta) {
  struct FloatEulers imu_to_body_eulers;
  memcpy(&imu_to_body_eulers, orientationGetEulers_f(&imu.body_to_imu), sizeof(struct FloatEulers));
  imu_to_body_eulers.theta = theta;
  orientationSetEulers_f(&imu.body_to_imu, &imu_to_body_eulers);
}

void imu_SetBodyToImuPsi(float psi) {
  struct FloatEulers imu_to_body_eulers;
  memcpy(&imu_to_body_eulers, orientationGetEulers_f(&imu.body_to_imu), sizeof(struct FloatEulers));
  imu_to_body_eulers.psi = psi;
  orientationSetEulers_f(&imu.body_to_imu, &imu_to_body_eulers);
}

void imu_SetBodyToImuCurrent(float set) {
  imu.b2i_set_current = set;

  if (imu.b2i_set_current) {
    // adjust imu_to_body roll and pitch by current NedToBody roll and pitch
    struct FloatEulers imu_to_body_eulers;
    memcpy(&imu_to_body_eulers, orientationGetEulers_f(&imu.body_to_imu), sizeof(struct FloatEulers));
    if (stateIsAttitudeValid()) {
      // adjust imu_to_body roll and pitch by current NedToBody roll and pitch
      imu_to_body_eulers.phi += stateGetNedToBodyEulers_f()->phi;
      imu_to_body_eulers.theta += stateGetNedToBodyEulers_f()->theta;
      orientationSetEulers_f(&imu.body_to_imu, &imu_to_body_eulers);
    }
    else {
      // indicate that we couldn't set to current roll/pitch
      imu.b2i_set_current = FALSE;
    }
  }
  else {
    // reset to BODY_TO_IMU as defined in airframe file
    struct FloatEulers imu_to_body_eulers =
      {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
    orientationSetEulers_f(&imu.body_to_imu, &imu_to_body_eulers);
  }
}


/*
 * Copyright (C) 2008-2022 The Paparazzi Team
 *                         Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu.c
 * Inertial Measurement Unit interface.
 */

#ifdef BOARD_CONFIG
#include BOARD_CONFIG
#endif

#include "modules/imu/imu.h"
#include "state.h"
#include "modules/core/abi.h"
#include "modules/energy/electrical.h"

/** By default disable IMU integration calculations */
#ifndef IMU_INTEGRATION
#define IMU_INTEGRATION false
#endif

/** Default gyro calibration is for single IMU with old format */
#if defined(IMU_GYRO_P_NEUTRAL) || defined(IMU_GYRO_Q_NEUTRAL) || defined(IMU_GYRO_R_NEUTRAL)
#define GYRO_NEUTRAL {IMU_GYRO_P_NEUTRAL, IMU_GYRO_Q_NEUTRAL, IMU_GYRO_R_NEUTRAL}
PRINT_CONFIG_MSG("Using single IMU gyro neutral calibration")
#endif

#if defined(IMU_GYRO_P_SENS) ||  defined(IMU_GYRO_Q_SENS) || defined(IMU_GYRO_R_SENS)
#define GYRO_SCALE {{IMU_GYRO_P_SENS_NUM, IMU_GYRO_Q_SENS_NUM, IMU_GYRO_R_SENS_NUM}, {IMU_GYRO_P_SENS_DEN, IMU_GYRO_Q_SENS_DEN, IMU_GYRO_R_SENS_DEN}}
PRINT_CONFIG_MSG("Using single IMU gyro sensitivity calibration")
#endif

#if defined(GYRO_NEUTRAL) && defined(GYRO_SCALE)
#define IMU_GYRO_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.neutral=true, .scale=true}, .neutral=GYRO_NEUTRAL, .scale=GYRO_SCALE}}
#elif defined(GYRO_NEUTRAL)
#define IMU_GYRO_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.neutral=true}, .neutral=GYRO_NEUTRAL}}
#elif defined(GYRO_SCALE)
#define IMU_GYRO_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.scale=true}, .scale=GYRO_SCALE}}
#elif !defined(IMU_GYRO_CALIB)
#define IMU_GYRO_CALIB {}
#endif

/** Default accel calibration is for single IMU with old format */
#if defined(IMU_ACCEL_X_NEUTRAL) || defined(IMU_ACCEL_Y_NEUTRAL) || defined(IMU_ACCEL_Z_NEUTRAL)
#define ACCEL_NEUTRAL {IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL}
PRINT_CONFIG_MSG("Using single IMU accel neutral calibration")
#endif

#if defined(IMU_ACCEL_X_SENS) ||  defined(IMU_ACCEL_Y_SENS) || defined(IMU_ACCEL_Z_SENS)
#define ACCEL_SCALE {{IMU_ACCEL_X_SENS_NUM, IMU_ACCEL_Y_SENS_NUM, IMU_ACCEL_Z_SENS_NUM}, {IMU_ACCEL_X_SENS_DEN, IMU_ACCEL_Y_SENS_DEN, IMU_ACCEL_Z_SENS_DEN}}
PRINT_CONFIG_MSG("Using single IMU accel sensitivity calibration")
#endif

#if defined(ACCEL_NEUTRAL) && defined(ACCEL_SCALE)
#define IMU_ACCEL_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.neutral=true, .scale=true}, .neutral=ACCEL_NEUTRAL, .scale=ACCEL_SCALE}}
#elif defined(ACCEL_NEUTRAL)
#define IMU_ACCEL_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.neutral=true}, .neutral=ACCEL_NEUTRAL}}
#elif defined(ACCEL_SCALE)
#define IMU_ACCEL_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.scale=true}, .scale=ACCEL_SCALE}}
#elif !defined(IMU_ACCEL_CALIB)
#define IMU_ACCEL_CALIB {}
#endif

/** Default mag calibration is for single IMU with old format */
#if defined(IMU_MAG_X_NEUTRAL) || defined(IMU_MAG_Y_NEUTRAL) || defined(IMU_MAG_Z_NEUTRAL)
#define MAG_NEUTRAL {IMU_MAG_X_NEUTRAL, IMU_MAG_Y_NEUTRAL, IMU_MAG_Z_NEUTRAL}
PRINT_CONFIG_MSG("Using single IMU mag neutral calibration")
#endif

#if defined(IMU_MAG_X_SENS) ||  defined(IMU_MAG_Y_SENS) || defined(IMU_MAG_Z_SENS)
#define MAG_SCALE {{IMU_MAG_X_SENS_NUM, IMU_MAG_Y_SENS_NUM, IMU_MAG_Z_SENS_NUM}, {IMU_MAG_X_SENS_DEN, IMU_MAG_Y_SENS_DEN, IMU_MAG_Z_SENS_DEN}}
PRINT_CONFIG_MSG("Using single IMU mag sensitivity calibration")
#endif

#if defined(MAG_NEUTRAL) && defined(MAG_SCALE)
#define IMU_MAG_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.neutral=true, .scale=true}, .neutral=MAG_NEUTRAL, .scale=MAG_SCALE}}
#elif defined(MAG_NEUTRAL)
#define IMU_MAG_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.neutral=true}, .neutral=MAG_NEUTRAL}}
#elif defined(MAG_SCALE)
#define IMU_MAG_CALIB {{.abi_id=ABI_BROADCAST, .calibrated={.scale=true}, .scale=MAG_SCALE}}
#elif !defined(IMU_MAG_CALIB)
#define IMU_MAG_CALIB {}
#endif


/** Default body to imu is 0 (radians) */
#if !defined(IMU_BODY_TO_IMU_PHI) && !defined(IMU_BODY_TO_IMU_THETA) && !defined(IMU_BODY_TO_IMU_PSI)
#define IMU_BODY_TO_IMU_PHI   0
#define IMU_BODY_TO_IMU_THETA 0
#define IMU_BODY_TO_IMU_PSI   0
#endif
PRINT_CONFIG_VAR(IMU_BODY_TO_IMU_PHI)
PRINT_CONFIG_VAR(IMU_BODY_TO_IMU_THETA)
PRINT_CONFIG_VAR(IMU_BODY_TO_IMU_PSI)


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_accel_raw(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_ACCEL_RAW(trans, dev, AC_ID, &imu.accels[id].abi_id,
                              &imu.accels[id].unscaled.x, &imu.accels[id].unscaled.y, &imu.accels[id].unscaled.z);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.accels[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_accel_scaled(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_ACCEL_SCALED(trans, dev, AC_ID, &imu.accels[id].abi_id,
                                 &imu.accels[id].scaled.x, &imu.accels[id].scaled.y, &imu.accels[id].scaled.z);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.accels[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_accel(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accels[id].scaled);
  pprz_msg_send_IMU_ACCEL(trans, dev, AC_ID, &imu.accels[id].abi_id,
                          &accel_float.x, &accel_float.y, &accel_float.z);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.accels[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_gyro_raw(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_GYRO_RAW(trans, dev, AC_ID, &imu.gyros[id].abi_id,
                             &imu.gyros[id].unscaled.p, &imu.gyros[id].unscaled.q, &imu.gyros[id].unscaled.r);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.gyros[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_gyro_scaled(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_GYRO_SCALED(trans, dev, AC_ID, &imu.gyros[id].abi_id,
                                &imu.gyros[id].scaled.p, &imu.gyros[id].scaled.q, &imu.gyros[id].scaled.r);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.gyros[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_gyro(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyros[id].scaled);
  pprz_msg_send_IMU_GYRO(trans, dev, AC_ID, &imu.gyros[id].abi_id,
                         &gyro_float.p, &gyro_float.q, &gyro_float.r);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.gyros[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_mag_raw(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID, &imu.mags[id].abi_id,
                            &imu.mags[id].unscaled.x, &imu.mags[id].unscaled.y, &imu.mags[id].unscaled.z);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.mags[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_mag_scaled(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_MAG_SCALED(trans, dev, AC_ID, &imu.mags[id].abi_id ,
                               &imu.mags[id].scaled.x, &imu.mags[id].scaled.y, &imu.mags[id].scaled.z);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.mags[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_mag(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  struct FloatVect3 mag_float;
  MAGS_FLOAT_OF_BFP(mag_float, imu.mags[id].scaled);
  pprz_msg_send_IMU_MAG(trans, dev, AC_ID, &imu.mags[id].abi_id,
                        &mag_float.x, &mag_float.y, &mag_float.z);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.mags[id].abi_id == ABI_DISABLE)
    id = 0;
}

static void send_mag_current(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t id = 0;
  pprz_msg_send_IMU_MAG_CURRENT_CALIBRATION(trans, dev, AC_ID,
      &imu.mags[id].abi_id,
      &imu.mags[id].unscaled.x,
      &imu.mags[id].unscaled.y,
      &imu.mags[id].unscaled.z,
      &electrical.current);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.mags[id].abi_id == ABI_DISABLE)
    id = 0;
}

#endif /* PERIODIC_TELEMETRY */

struct Imu imu = {0};
static abi_event imu_gyro_raw_ev, imu_accel_raw_ev, imu_mag_raw_ev;
static void imu_gyro_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *data, uint8_t samples);
static void imu_accel_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *data, uint8_t samples);
static void imu_mag_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *data);
static void imu_set_body_to_imu_eulers(struct FloatEulers *body_to_imu_eulers);

void imu_init(void)
{
  // Set the Body to IMU rotation
  struct FloatEulers body_to_imu_eulers = {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);


  // Set the calibrated sensors
  struct Int32RMat body_to_sensor;
  const struct imu_gyro_t gyro_calib[] = IMU_GYRO_CALIB;
  const struct imu_accel_t accel_calib[] = IMU_ACCEL_CALIB;
  const struct imu_mag_t mag_calib[] = IMU_MAG_CALIB;
  const uint8_t gyro_calib_len = sizeof(gyro_calib) / sizeof(struct imu_gyro_t);
  const uint8_t accel_calib_len = sizeof(accel_calib) / sizeof(struct imu_accel_t);
  const uint8_t mag_calib_len = sizeof(mag_calib) / sizeof(struct imu_mag_t);

  // Initialize the sensors to default values
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    /* Copy gyro calibration if needed */
    if(i >= gyro_calib_len) {
      imu.gyros[i].abi_id = ABI_DISABLE;
      imu.gyros[i].calibrated.neutral = false;
      imu.gyros[i].calibrated.scale = false;
      imu.gyros[i].calibrated.rotation = false;
    } else {
      imu.gyros[i] = gyro_calib[i];
    }

    // Set the default safe values if not calibrated
    if(!imu.gyros[i].calibrated.neutral) {
      INT_RATES_ZERO(imu.gyros[i].neutral);
    }
    if(!imu.gyros[i].calibrated.scale) {
      RATES_ASSIGN(imu.gyros[i].scale[0], 1, 1, 1);
      RATES_ASSIGN(imu.gyros[i].scale[1], 1, 1, 1);
    }
    if(!imu.gyros[i].calibrated.rotation) {
      int32_rmat_identity(&imu.gyros[i].body_to_sensor);
    }
    imu.gyros[i].last_stamp = 0;
    int32_rmat_comp(&body_to_sensor, body_to_imu_rmat, &imu.gyros[i].body_to_sensor);
    RMAT_COPY(imu.gyros[i].body_to_sensor, body_to_sensor);


    /* Copy accel calibration if needed */
    if(i >= accel_calib_len) {
      imu.accels[i].abi_id = ABI_DISABLE;
      imu.accels[i].calibrated.neutral = false;
      imu.accels[i].calibrated.scale = false;
      imu.accels[i].calibrated.rotation = false;
    } else {
      imu.accels[i] = accel_calib[i];
    }

    // Set the default safe values if not calibrated
    if(!imu.accels[i].calibrated.neutral) {
      INT_VECT3_ZERO(imu.accels[i].neutral);
    }
    if(!imu.accels[i].calibrated.scale) {
      VECT3_ASSIGN(imu.accels[i].scale[0], 1, 1, 1);
      VECT3_ASSIGN(imu.accels[i].scale[1], 1, 1, 1);
    }
    if(!imu.accels[i].calibrated.rotation) {
      int32_rmat_identity(&imu.accels[i].body_to_sensor);
    }
    imu.accels[i].last_stamp = 0;
    int32_rmat_comp(&body_to_sensor, body_to_imu_rmat, &imu.accels[i].body_to_sensor);
    RMAT_COPY(imu.accels[i].body_to_sensor, body_to_sensor);


    /* Copy mag calibrated if needed */
    if(i >= mag_calib_len) {
      imu.mags[i].abi_id = ABI_DISABLE;
      imu.mags[i].calibrated.neutral = false;
      imu.mags[i].calibrated.scale = false;
      imu.mags[i].calibrated.rotation = false;
      imu.mags[i].calibrated.current = false;
    } else {
      imu.mags[i] = mag_calib[i];
    }

    // Set the default safe values if not calibrated
    if(!imu.mags[i].calibrated.neutral) {
      INT_VECT3_ZERO(imu.mags[i].neutral);
    }
    if(!imu.mags[i].calibrated.scale) {
      VECT3_ASSIGN(imu.mags[i].scale[0], 1, 1, 1);
      VECT3_ASSIGN(imu.mags[i].scale[1], 1, 1, 1);
    }
    if(!imu.mags[i].calibrated.rotation) {
      int32_rmat_identity(&imu.mags[i].body_to_sensor);
    }
    if(!imu.mags[i].calibrated.current) {
      INT_VECT3_ZERO(imu.mags[i].current_scale);
    }
    int32_rmat_comp(&body_to_sensor, body_to_imu_rmat, &imu.mags[i].body_to_sensor);
    RMAT_COPY(imu.mags[i].body_to_sensor, body_to_sensor);
  }

  // Bind to raw measurements
  AbiBindMsgIMU_GYRO_RAW(ABI_BROADCAST, &imu_gyro_raw_ev, imu_gyro_raw_cb);
  AbiBindMsgIMU_ACCEL_RAW(ABI_BROADCAST, &imu_accel_raw_ev, imu_accel_raw_cb);
  AbiBindMsgIMU_MAG_RAW(ABI_BROADCAST, &imu_mag_raw_ev, imu_mag_raw_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL_RAW, send_accel_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL_SCALED, send_accel_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL, send_accel);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO_RAW, send_gyro_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO_SCALED, send_gyro_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO, send_gyro);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, send_mag_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_SCALED, send_mag_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG, send_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_CURRENT_CALIBRATION, send_mag_current);
#endif // DOWNLINK

  imu.initialized = true;
}

/**
 * @brief Set the defaults for a gyro sensor
 * WARNING: Should be called before sensor is publishing messages to ensure correct values
 * @param abi_id The ABI sender id to set the defaults for
 * @param imu_to_sensor Imu to sensor rotation matrix
 * @param neutral Neutral values
 * @param scale Scale values, 0 index is multiply and 1 index is divide
 */
void imu_set_defaults_gyro(uint8_t abi_id, const struct Int32RMat *imu_to_sensor, const struct Int32Rates *neutral, const struct Int32Rates *scale)
{
  // Find the correct gyro
  struct imu_gyro_t *gyro = imu_get_gyro(abi_id, true);
  if(gyro == NULL)
    return;

  // Copy the defaults
  if(imu_to_sensor != NULL && !gyro->calibrated.rotation) {
    struct Int32RMat body_to_sensor;
    struct Int32RMat *body_to_imu = orientationGetRMat_i(&imu.body_to_imu);
    int32_rmat_comp(&body_to_sensor, body_to_imu, imu_to_sensor);
    RMAT_COPY(gyro->body_to_sensor, body_to_sensor);
  }
  if(neutral != NULL && !gyro->calibrated.neutral)
    RATES_COPY(gyro->neutral, *neutral);
  if(scale != NULL && !gyro->calibrated.scale) {
    RATES_COPY(gyro->scale[0], scale[0]);
    RATES_COPY(gyro->scale[1], scale[1]);
  }
}

/**
 * @brief Set the defaults for a accel sensor
 * WARNING: Should be called before sensor is publishing messages to ensure correct values
 * @param abi_id The ABI sender id to set the defaults for
 * @param imu_to_sensor Imu to sensor rotation matrix
 * @param neutral Neutral values
 * @param scale Scale values, 0 index is multiply and 1 index is divide
 */
void imu_set_defaults_accel(uint8_t abi_id, const struct Int32RMat *imu_to_sensor, const struct Int32Vect3 *neutral, const struct Int32Vect3 *scale)
{
  // Find the correct accel
  struct imu_accel_t *accel = imu_get_accel(abi_id, true);
  if(accel == NULL)
    return;

  // Copy the defaults
  if(imu_to_sensor != NULL && !accel->calibrated.rotation) {
    struct Int32RMat body_to_sensor;
    struct Int32RMat *body_to_imu = orientationGetRMat_i(&imu.body_to_imu);
    int32_rmat_comp(&body_to_sensor, body_to_imu, imu_to_sensor);
    RMAT_COPY(accel->body_to_sensor, body_to_sensor);
  }
  if(neutral != NULL && !accel->calibrated.neutral)
    VECT3_COPY(accel->neutral, *neutral);
  if(scale != NULL && !accel->calibrated.scale) {
    VECT3_COPY(accel->scale[0], scale[0]);
    VECT3_COPY(accel->scale[1], scale[1]);
  }
}

/**
 * @brief Set the defaults for a mag sensor
 * WARNING: Should be called before sensor is publishing messages to ensure correct values
 * @param abi_id The ABI sender id to set the defaults for
 * @param imu_to_sensor Imu to sensor rotation matrix
 * @param neutral Neutral values
 * @param scale Scale values, 0 index is multiply and 1 index is divide
 */
void imu_set_defaults_mag(uint8_t abi_id, const struct Int32RMat *imu_to_sensor, const struct Int32Vect3 *neutral, const struct Int32Vect3 *scale)
{
  // Find the correct mag
  struct imu_mag_t *mag = imu_get_mag(abi_id, true);
  if(mag == NULL)
    return;

  // Copy the defaults
  if(imu_to_sensor != NULL && !mag->calibrated.rotation) {
    struct Int32RMat body_to_sensor;
    struct Int32RMat *body_to_imu = orientationGetRMat_i(&imu.body_to_imu);
    int32_rmat_comp(&body_to_sensor, body_to_imu, imu_to_sensor);
    RMAT_COPY(mag->body_to_sensor, body_to_sensor);
  }
  if(neutral != NULL && !mag->calibrated.neutral)
    VECT3_COPY(mag->neutral, *neutral);
  if(scale != NULL && !mag->calibrated.scale) {
    VECT3_COPY(mag->scale[0], scale[0]);
    VECT3_COPY(mag->scale[1], scale[1]);
  }
}

static void imu_gyro_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *data, uint8_t samples)
{
  // Find the correct gyro
  struct imu_gyro_t *gyro = imu_get_gyro(sender_id, true);
  if(gyro == NULL || samples < 1)
    return;

  // Copy last sample as unscaled
  RATES_COPY(gyro->unscaled, data[samples-1]);

  // Scale the gyro
  struct Int32Rates scaled, scaled_rot;
  scaled.p = (gyro->unscaled.p - gyro->neutral.p) * gyro->scale[0].p / gyro->scale[1].p;
  scaled.q = (gyro->unscaled.q - gyro->neutral.q) * gyro->scale[0].q / gyro->scale[1].q;
  scaled.r = (gyro->unscaled.r - gyro->neutral.r) * gyro->scale[0].r / gyro->scale[1].r;

  // Rotate the sensor
  int32_rmat_transp_ratemult(&scaled_rot, &gyro->body_to_sensor, &scaled);

#if IMU_INTEGRATION
  // Only integrate if we have gotten a previous measurement and didn't overflow the timer
  if(gyro->last_stamp > 0 && stamp > gyro->last_stamp) {
    struct FloatRates integrated;
    uint16_t delta_dt = stamp - gyro->last_stamp;

    // Trapezoidal integration (TODO: coning correction)
    integrated.p = RATE_FLOAT_OF_BFP(gyro->scaled.p + scaled_rot.p) * 0.5f;
    integrated.q = RATE_FLOAT_OF_BFP(gyro->scaled.q + scaled_rot.q) * 0.5f;
    integrated.r = RATE_FLOAT_OF_BFP(gyro->scaled.r + scaled_rot.r) * 0.5f;

    for(uint8_t i = 0; i < samples-1; i++) {
      integrated.p += data[i].p;
      integrated.q += data[i].q;
      integrated.r += data[i].r;
    }

    integrated.p = integrated.p / samples * ((float)delta_dt * 1e-6f);
    integrated.q = integrated.q / samples * ((float)delta_dt * 1e-6f);
    integrated.r = integrated.r / samples * ((float)delta_dt * 1e-6f);

    // Send the integrated values
    AbiSendMsgIMU_GYRO_INT(sender_id, stamp, &integrated, delta_dt);
  }
#endif

  // Copy and send
  RATES_COPY(gyro->scaled, scaled_rot);
  AbiSendMsgIMU_GYRO(sender_id, stamp, &gyro->scaled);
  gyro->last_stamp = stamp;
}

static void imu_accel_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *data, uint8_t samples)
{
  // Find the correct accel
  struct imu_accel_t *accel = imu_get_accel(sender_id, true);
  if(accel == NULL || samples < 1)
    return;

  // Copy last sample as unscaled
  VECT3_COPY(accel->unscaled, data[samples-1]);

  // Scale the accel
  struct Int32Vect3 scaled, scaled_rot;
  scaled.x = (accel->unscaled.x - accel->neutral.x) * accel->scale[0].x / accel->scale[1].x;
  scaled.y = (accel->unscaled.y - accel->neutral.y) * accel->scale[0].y / accel->scale[1].y;
  scaled.z = (accel->unscaled.z - accel->neutral.z) * accel->scale[0].z / accel->scale[1].z;

  // Rotate the sensor
  int32_rmat_transp_vmult(&scaled_rot, &accel->body_to_sensor, &scaled);

#if IMU_INTEGRATION
  // Only integrate if we have gotten a previous measurement and didn't overflow the timer
  if(accel->last_stamp > 0 && stamp > accel->last_stamp) {
    struct FloatVect3 integrated;
    uint16_t delta_dt = stamp - accel->last_stamp;

    // Trapezoidal integration
    integrated.x = RATE_FLOAT_OF_BFP(accel->scaled.x + scaled_rot.x) * 0.5f;
    integrated.y = RATE_FLOAT_OF_BFP(accel->scaled.y + scaled_rot.y) * 0.5f;
    integrated.z = RATE_FLOAT_OF_BFP(accel->scaled.z + scaled_rot.z) * 0.5f;

    for(uint8_t i = 0; i < samples-1; i++) {
      integrated.x += data[i].x;
      integrated.y += data[i].y;
      integrated.z += data[i].z;
    }

    integrated.x = integrated.x / samples * ((float)delta_dt * 1e-6f);
    integrated.y = integrated.y / samples * ((float)delta_dt * 1e-6f);
    integrated.z = integrated.z / samples * ((float)delta_dt * 1e-6f);

    // Send the integrated values
    AbiSendMsgIMU_ACCEL_INT(sender_id, stamp, &integrated, delta_dt);
  }
#endif

  // Copy and send
  VECT3_COPY(accel->scaled, scaled_rot);
  AbiSendMsgIMU_ACCEL(sender_id, stamp, &accel->scaled);
  accel->last_stamp = stamp;
}

static void imu_mag_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *data)
{
  // Find the correct mag
  struct imu_mag_t *mag = imu_get_mag(sender_id, true);
  if(mag == NULL)
    return;

  // Calculate current compensation
  struct Int32Vect3 mag_correction;
  mag_correction.x = (int32_t)(mag->current_scale.x * (float) electrical.current);
  mag_correction.y = (int32_t)(mag->current_scale.y * (float) electrical.current);
  mag_correction.z = (int32_t)(mag->current_scale.z * (float) electrical.current);

  // Copy last sample as unscaled
  VECT3_COPY(mag->unscaled, *data);

  // Scale the mag
  struct Int32Vect3 scaled;
  scaled.x = (mag->unscaled.x - mag_correction.x - mag->neutral.x) * mag->scale[0].x / mag->scale[1].x;
  scaled.y = (mag->unscaled.y - mag_correction.y - mag->neutral.y) * mag->scale[0].y / mag->scale[1].y;
  scaled.z = (mag->unscaled.z - mag_correction.z - mag->neutral.z) * mag->scale[0].z / mag->scale[1].z;

  // Rotate the sensor
  int32_rmat_transp_vmult(&mag->scaled, &mag->body_to_sensor, &scaled);
  AbiSendMsgIMU_MAG(sender_id, stamp, &mag->scaled);
}

/**
 * @brief Find or create the gyro in the imu structure
 * 
 * @param sender_id The ABI sender id to search for
 * @param create Create a new index if not found
 * @return struct imu_gyro_t* The gyro structure if found/created else NULL
 */
struct imu_gyro_t *imu_get_gyro(uint8_t sender_id, bool create) {
  // Find the correct gyro or create index
  // If abi_id was set to broadcast (assuming a single IMU case), the first request takes the index for himself
  struct imu_gyro_t *gyro = NULL;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.gyros[i].abi_id == sender_id || (create && (imu.gyros[i].abi_id == ABI_BROADCAST || imu.gyros[i].abi_id == ABI_DISABLE))) {
      gyro = &imu.gyros[i];
      gyro->abi_id = sender_id;
      break;
    } else if(sender_id == ABI_BROADCAST && imu.gyros[i].abi_id != ABI_DISABLE) {
      gyro = &imu.gyros[i];
    }
  }
  return gyro;
}

/**
 * @brief Find or create the accel in the imu structure
 * 
 * @param sender_id The ABI sender id to search for
 * @param create Create a new index if not found
 * @return struct imu_accel_t* The accel structure if found/created else NULL
 */
struct imu_accel_t *imu_get_accel(uint8_t sender_id, bool create) {
  // Find the correct accel
  // If abi_id was set to broadcast (assuming a single IMU case), the first request takes the index for himself
  struct imu_accel_t *accel = NULL;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.accels[i].abi_id == sender_id || (create && (imu.accels[i].abi_id == ABI_BROADCAST || imu.accels[i].abi_id == ABI_DISABLE))) {
      accel = &imu.accels[i];
      accel->abi_id = sender_id;
      break;
    } else if(sender_id == ABI_BROADCAST && imu.accels[i].abi_id != ABI_DISABLE) {
      accel = &imu.accels[i];
    }
  }

  return accel;
}

/**
 * @brief Find or create the mag in the imu structure
 * 
 * @param sender_id The ABI sender id to search for
 * @param create Create a new index if not found
 * @return struct imu_mag_t* The mag structure if found/created else NULL
 */
struct imu_mag_t *imu_get_mag(uint8_t sender_id, bool create) {
  // Find the correct mag
  // If abi_id was set to broadcast (assuming a single IMU case), the first request takes the index for himself
  struct imu_mag_t *mag = NULL;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.mags[i].abi_id == sender_id || (create && (imu.mags[i].abi_id == ABI_BROADCAST || imu.mags[i].abi_id == ABI_DISABLE))) {
      mag = &imu.mags[i];
      mag->abi_id = sender_id;
      break;
    } else if(sender_id == ABI_BROADCAST && imu.mags[i].abi_id != ABI_DISABLE) {
      mag = &imu.mags[i];
    }
  }

  return mag;
}

/**
 * @brief Set the body to IMU rotation in eulers
 * This will update all the sensor values
 * @param body_to_imu_eulers 321 Euler angles in radians
 */
static void imu_set_body_to_imu_eulers(struct FloatEulers *body_to_imu_eulers)
{
  struct Int32RMat new_body_to_imu, diff_body_to_imu;
  struct Int32Eulers body_to_imu_eulers_i;
  // Convert to RMat
  struct Int32RMat *old_body_to_imu = orientationGetRMat_i(&imu.body_to_imu);
  EULERS_BFP_OF_REAL(body_to_imu_eulers_i, *body_to_imu_eulers);
  int32_rmat_of_eulers(&new_body_to_imu, &body_to_imu_eulers_i);

  // Calculate the difference between old and new
  int32_rmat_comp_inv(&diff_body_to_imu, &new_body_to_imu, old_body_to_imu);

  // Apply the difference to all sensors
  struct Int32RMat old_rmat;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    old_rmat = imu.gyros[i].body_to_sensor;
    int32_rmat_comp(&imu.gyros[i].body_to_sensor, &diff_body_to_imu, &old_rmat);

    old_rmat = imu.accels[i].body_to_sensor;
    int32_rmat_comp(&imu.accels[i].body_to_sensor, &diff_body_to_imu, &old_rmat);

    old_rmat = imu.mags[i].body_to_sensor;
    int32_rmat_comp(&imu.mags[i].body_to_sensor, &diff_body_to_imu, &old_rmat);
  }

  // Set the current body to imu
  orientationSetEulers_f(&imu.body_to_imu, body_to_imu_eulers);
}

void imu_SetBodyToImuPhi(float phi)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.phi = phi;
  imu_set_body_to_imu_eulers(&body_to_imu_eulers);
}

void imu_SetBodyToImuTheta(float theta)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.theta = theta;
  imu_set_body_to_imu_eulers(&body_to_imu_eulers);
}

void imu_SetBodyToImuPsi(float psi)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.psi = psi;
  imu_set_body_to_imu_eulers(&body_to_imu_eulers);
}

void imu_SetBodyToImuCurrent(float set)
{
  imu.b2i_set_current = set;

  if (imu.b2i_set_current) {
    // adjust imu_to_body roll and pitch by current NedToBody roll and pitch
    struct FloatEulers body_to_imu_eulers;
    body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
    if (stateIsAttitudeValid()) {
      // adjust imu_to_body roll and pitch by current NedToBody roll and pitch
      body_to_imu_eulers.phi += stateGetNedToBodyEulers_f()->phi;
      body_to_imu_eulers.theta += stateGetNedToBodyEulers_f()->theta;
      imu_set_body_to_imu_eulers(&body_to_imu_eulers);
    } else {
      // indicate that we couldn't set to current roll/pitch
      imu.b2i_set_current = false;
    }
  } else {
    // reset to BODY_TO_IMU as defined in airframe file
    struct FloatEulers body_to_imu_eulers = {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
    imu_set_body_to_imu_eulers(&body_to_imu_eulers);
  }
}

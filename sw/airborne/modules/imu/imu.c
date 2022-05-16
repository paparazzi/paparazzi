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

#ifndef IMU_INTEGRATION
#define IMU_INTEGRATION true
#endif

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
  if(id >= IMU_MAX_SENSORS)
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

#endif /* PERIODIC_TELEMETRY */

struct Imu imu = {0};
static abi_event imu_gyro_raw_ev, imu_accel_raw_ev, imu_mag_raw_ev;
static void imu_gyro_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *data, uint8_t samples);
static void imu_accel_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *data, uint8_t samples);
static void imu_mag_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *data);
static struct imu_gyro_t *imu_get_gyro(uint8_t sender_id);
static struct imu_accel_t *imu_get_accel(uint8_t sender_id);
static struct imu_mag_t *imu_get_mag(uint8_t sender_id);

void imu_init(void)
{
  // Do not initialize twice
  if(imu.initialized)
    return;
  
  // Set the Body to IMU rotation
  struct FloatEulers body_to_imu_eulers =
  {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);

  // Initialize the non-initialized sensors to default values
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.gyros[i].abi_id == ABI_DISABLE) {
      imu.gyros[i].last_stamp = 0;
      INT_RATES_ZERO(imu.gyros[i].neutral);
      RATES_ASSIGN(imu.gyros[i].scale[0], 1, 1, 1);
      RATES_ASSIGN(imu.gyros[i].scale[1], 1, 1, 1);
      int32_rmat_identity(&imu.gyros[i].imu_to_sensor);
    }

    if(imu.accels[i].abi_id == ABI_DISABLE) {
      imu.accels[i].last_stamp = 0;
      INT_VECT3_ZERO(imu.accels[i].neutral);
      VECT3_ASSIGN(imu.accels[i].scale[0], 1, 1, 1);
      VECT3_ASSIGN(imu.accels[i].scale[1], 1, 1, 1);
      int32_rmat_identity(&imu.accels[i].imu_to_sensor);
    }

    if(imu.mags[i].abi_id == ABI_DISABLE) {
      INT_VECT3_ZERO(imu.mags[i].neutral);
      VECT3_ASSIGN(imu.mags[i].scale[0], 1, 1, 1);
      VECT3_ASSIGN(imu.mags[i].scale[1], 1, 1, 1);
      INT_VECT3_ZERO(imu.mags[i].current_scale);
      int32_rmat_identity(&imu.mags[i].imu_to_sensor);
    }
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
#endif // DOWNLINK

  imu.initialized = true;
}

/**
 * @brief Set the sensor rotation
 * 
 * @param abi_id 
 * @param imu_to_sensor 
 */
void imu_set_gyro_rmat(uint8_t abi_id, struct Int32RMat *imu_to_sensor)
{
  // Could be that we are not initialized
  imu_init();

  // Find the correct gyro
  struct imu_gyro_t *gyro = imu_get_gyro(abi_id);
  if(gyro == NULL)
    return;

  RMAT_COPY(gyro->imu_to_sensor, *imu_to_sensor);
}

/**
 * @brief Set the sensor rotation
 * 
 * @param abi_id 
 * @param imu_to_sensor 
 */
void imu_set_accel_rmat(uint8_t abi_id, struct Int32RMat *imu_to_sensor)
{
  // Could be that we are not initialized
  imu_init();

  // Find the correct accel
  struct imu_accel_t *accel = imu_get_accel(abi_id);
  if(accel == NULL)
    return;

  RMAT_COPY(accel->imu_to_sensor, *imu_to_sensor);
}

static void imu_gyro_raw_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *data, uint8_t samples)
{
  // Find the correct gyro
  struct imu_gyro_t *gyro = imu_get_gyro(sender_id);
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
  int32_rmat_ratemult(&scaled_rot, &gyro->imu_to_sensor, &scaled);

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
  struct imu_accel_t *accel = imu_get_accel(sender_id);
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
  int32_rmat_transp_vmult(&scaled_rot, &accel->imu_to_sensor, &scaled);

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
  struct imu_mag_t *mag = imu_get_mag(sender_id);
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
  int32_rmat_transp_vmult(&mag->scaled, &mag->imu_to_sensor, &scaled);
  AbiSendMsgIMU_MAG(sender_id, stamp, &mag->scaled);
}

/**
 * @brief Find or create the gyro in the imu structure
 * 
 * @param sender_id The ABI sender id to search for
 * @return struct imu_gyro_t* The gyro structure if found/created else NULL
 */
static struct imu_gyro_t *imu_get_gyro(uint8_t sender_id) {
  // Find the correct gyro or create index
  struct imu_gyro_t *gyro = NULL;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.gyros[i].abi_id == sender_id || imu.gyros[i].abi_id == ABI_DISABLE) {
      gyro = &imu.gyros[i];
      gyro->abi_id = sender_id;
      break;
    }
  }

  return gyro;
}

/**
 * @brief Find or create the accel in the imu structure
 * 
 * @param sender_id The ABI sender id to search for
 * @return struct imu_accel_t* The accel structure if found/created else NULL
 */
static struct imu_accel_t *imu_get_accel(uint8_t sender_id) {
  // Find the correct accel
  struct imu_accel_t *accel = NULL;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.accels[i].abi_id == sender_id || imu.accels[i].abi_id == ABI_DISABLE) {
      accel = &imu.accels[i];
      accel->abi_id = sender_id;
      break;
    }
  }

  return accel;
}

/**
 * @brief Find or create the mag in the imu structure
 * 
 * @param sender_id The ABI sender id to search for
 * @return struct imu_mag_t* The mag structure if found/created else NULL
 */
static struct imu_mag_t *imu_get_mag(uint8_t sender_id) {
  // Find the correct mag
  struct imu_mag_t *mag = NULL;
  for(uint8_t i = 0; i < IMU_MAX_SENSORS; i++) {
    if(imu.mags[i].abi_id == sender_id || imu.mags[i].abi_id == ABI_DISABLE) {
      mag = &imu.mags[i];
      mag->abi_id = sender_id;
      break;
    }
  }

  return mag;
}

void imu_SetBodyToImuPhi(float phi)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.phi = phi;
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

void imu_SetBodyToImuTheta(float theta)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.theta = theta;
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

void imu_SetBodyToImuPsi(float psi)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.psi = psi;
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
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
      orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
      AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
    } else {
      // indicate that we couldn't set to current roll/pitch
      imu.b2i_set_current = false;
    }
  } else {
    // reset to BODY_TO_IMU as defined in airframe file
    struct FloatEulers body_to_imu_eulers =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
    orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
    AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
  }
}

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
 * @file modules/imu/imu.h
 * Inertial Measurement Unit interface.
 */

#ifndef IMU_H
#define IMU_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"
#include "filters/low_pass_filter.h"
#include "generated/airframe.h"

#ifndef IMU_MAX_SENSORS
#define IMU_MAX_SENSORS 4
#endif

struct imu_calib_t {
  bool neutral: 1;    ///< Neutral values calibrated
  bool scale: 1;      ///< Scale calibrated
  bool rotation: 1;   ///< Rotation calibrated
  bool current: 1;    ///< Current calibrated
  bool filter: 1;     ///< Enable the lowpass filter
};

struct imu_gyro_t {
  uint8_t abi_id;                     ///< ABI sensor ID
  uint32_t last_stamp;                ///< Last measurement timestamp for integration
  struct imu_calib_t calibrated;      ///< Calibration bitmask
  struct Int32Rates scaled;           ///< Last scaled values in body frame
  struct Int32Rates unscaled;         ///< Last unscaled values in sensor frame
  float temperature;                  ///< Temperature in degrees celcius
  struct Int32Rates neutral;          ///< Neutral values, compensation on unscaled->scaled
  struct Int32Rates scale[2];         ///< Scaling, first is numerator and second denominator
  struct Int32RMat body_to_sensor;    ///< Rotation from body to sensor frame (body to imu combined with imu to sensor)
  float filter_freq;                  ///< Filter frequency
  float filter_sample_freq;           ///< Lowpass filter sample frequency (Hz)
  Butterworth2LowPass filter[3];      ///< Lowpass filter optional
};

struct imu_accel_t {
  uint8_t abi_id;                     ///< ABI sensor ID
  uint32_t last_stamp;                ///< Last measurement timestamp for integration
  struct imu_calib_t calibrated;      ///< Calibration bitmask
  struct Int32Vect3 scaled;           ///< Last scaled values in body frame
  struct Int32Vect3 unscaled;         ///< Last unscaled values in sensor frame
  float temperature;                  ///< Temperature in degrees celcius
  struct Int32Vect3 neutral;          ///< Neutral values, compensation on unscaled->scaled
  struct Int32Vect3 scale[2];         ///< Scaling, first is numerator and second denominator
  struct Int32RMat body_to_sensor;    ///< Rotation from body to sensor frame (body to imu combined with imu to sensor)
  float filter_freq;                  ///< Lowpass filter frequency (Hz)
  float filter_sample_freq;           ///< Lowpass filter sample frequency (Hz)
  Butterworth2LowPass filter[3];      ///< Lowpass filter optional
};

struct imu_mag_t {
  uint8_t abi_id;                     ///< ABI sensor ID
  struct imu_calib_t calibrated;      ///< Calibration bitmask
  struct Int32Vect3 scaled;           ///< Last scaled values in body frame
  struct Int32Vect3 unscaled;         ///< Last unscaled values in sensor frame
  struct Int32Vect3 neutral;          ///< Neutral values, compensation on unscaled->scaled
  struct Int32Vect3 scale[2];         ///< Scaling, first is numerator and second denominator
  struct FloatVect3 current_scale;    ///< Current scaling multiplying
  struct Int32RMat body_to_sensor;    ///< Rotation from body to sensor frame (body to imu combined with imu to sensor)
};


/** abstract IMU interface providing fixed point interface  */
struct Imu {
  bool initialized;                           ///< Check if the IMU is initialized
  struct imu_gyro_t gyros[IMU_MAX_SENSORS];   ///< The gyro sensors
  struct imu_accel_t accels[IMU_MAX_SENSORS]; ///< The accelerometer sensors
  struct imu_mag_t mags[IMU_MAX_SENSORS];     ///< The magnetometer sensors
  struct OrientationReps body_to_imu;         ///< Rotation from body to imu (all sensors) frame
  uint8_t gyro_abi_send_id;                   ///< Filter out and send only a specific ABI id in telemetry for the gyro
  uint8_t accel_abi_send_id;                  ///< Filter out and send only a specific ABI id in telemetry for the accelerometer
  uint8_t mag_abi_send_id;                    ///< Filter out and send only a specific ABI id in telemetry for the magnetometer

  /** flag for adjusting body_to_imu via settings.
   * if FALSE, reset to airframe values, if TRUE set current roll/pitch
   */
  bool b2i_set_current;
};

/** global IMU state */
extern struct Imu imu;

/** External functions */
extern void imu_init(void);

extern void imu_set_defaults_gyro(uint8_t abi_id, const struct Int32RMat *imu_to_sensor, const struct Int32Rates *neutral, const struct Int32Rates *scale);
extern void imu_set_defaults_accel(uint8_t abi_id, const struct Int32RMat *imu_to_sensor, const struct Int32Vect3 *neutral, const struct Int32Vect3 *scale);
extern void imu_set_defaults_mag(uint8_t abi_id, const struct Int32RMat *imu_to_sensor, const struct Int32Vect3 *neutral, const struct Int32Vect3 *scale);

extern struct imu_gyro_t *imu_get_gyro(uint8_t sender_id, bool create);
extern struct imu_accel_t *imu_get_accel(uint8_t sender_id, bool create);
extern struct imu_mag_t *imu_get_mag(uint8_t sender_id, bool create);

extern void imu_SetBodyToImuPhi(float phi);
extern void imu_SetBodyToImuTheta(float theta);
extern void imu_SetBodyToImuPsi(float psi);
extern void imu_SetBodyToImuCurrent(float set);
extern void imu_ResetBodyToImu(float reset);

#endif /* IMU_H */

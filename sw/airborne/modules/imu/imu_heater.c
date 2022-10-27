/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_heater.c
 * IMU heater module which can actuate a resistor heater through GPIO or IOMCU.
 */

#include "imu_heater.h"
#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"
#include "modules/imu/imu.h"
#include "math/pprz_random.h"

#if INTERMCU_IOMCU
#include "modules/intermcu/iomcu.h"
#endif

/** Default target temperature */
#ifndef IMU_HEATER_TARGET_TEMP
#define IMU_HEATER_TARGET_TEMP 45
#endif

/** Default heater kP gain */
#ifndef IMU_HEATER_P_GAIN
#define IMU_HEATER_P_GAIN 200.0
#endif

/** Default heater kI gain */
#ifndef IMU_HEATER_I_GAIN
#define IMU_HEATER_I_GAIN 0.3
#endif

/** Default heater gyro abi ID */
#if !defined(IMU_HEATER_GYRO_ID) && !defined(IMU_HEATER_ACCEL_ID)
#define IMU_HEATER_GYRO_ID ABI_BROADCAST
#endif

/** Local variables and functions */
struct imu_heater_t imu_heater;
static abi_event imu_heater_abi_ev;
static float imu_heater_run(struct imu_heater_t *heater, float temp);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/**
 * @brief Send the IMU heater message
 *
 * @param trans Transport structure to send
 * @param dev Device to send the message over
 */
static void send_imu_heater(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_HEATER(trans, dev, AC_ID, &imu_heater.meas_temp, &imu_heater.target_temp, &imu_heater.heat_cmd);
}
#endif

#if defined(IMU_HEATER_GYRO_ID)
static void imu_heater_gyro_raw_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct Int32Rates *data __attribute__((unused)), uint8_t samples __attribute__((unused)), float temp) {
  if(isnan(temp))
    return;
  
  imu_heater.meas_temp_sum += temp;
  imu_heater.meas_temp_cnt++;
}
#endif

#if defined(IMU_HEATER_ACCEL_ID)
static void imu_heater_accel_raw_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct Int32Vect3 *data __attribute__((unused)), uint8_t samples __attribute__((unused)), float temp) {
  if(isnan(temp))
    return;

  imu_heater.meas_temp_sum += temp;
  imu_heater.meas_temp_cnt++;
}
#endif

/**
 * @brief Initialize the heater
 * Sets the default gains and resets the initial values
 */
void imu_heater_init(void)
{
  /* Setup the heater */
  imu_heater.target_temp = IMU_HEATER_TARGET_TEMP;
  imu_heater.gain_p = IMU_HEATER_P_GAIN;
  imu_heater.gain_i = IMU_HEATER_I_GAIN;
  imu_heater.meas_temp = NAN;
  imu_heater.meas_temp_sum = NAN;
  imu_heater.meas_temp_cnt = 0;
  imu_heater.integrated = 0;
  imu_heater.last_ts = 0;
  imu_heater.heat_cmd = 0;

  /* Bind to raw temperature measurements */
#if defined(IMU_HEATER_GYRO_ID)
  AbiBindMsgIMU_GYRO_RAW(IMU_HEATER_GYRO_ID, &imu_heater_abi_ev, imu_heater_gyro_raw_cb);
#elif defined(IMU_HEATER_ACCEL_ID)
  AbiBindMsgIMU_ACCEL_RAW(IMU_HEATER_ACCEL_ID, &imu_heater_abi_ev, imu_heater_accel_raw_cb);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_HEATER, send_imu_heater);
#endif
}

/**
 * @brief High speed heater periodic
 * This calculates the temperature average and in case of GPIO heater it
 * will provide a heater output. The GPIO out is randomized to reduce the
 * magnetometer influence.
 */
void imu_heater_periodic(void)
{
  // Set the GPIO heating resistor (using random noise for the magnetometer)
#if defined(IMU_HEATER_GPIO) && defined(IMU_HEATER_GPIO_PIN)
  if (imu_heater.heat_cmd != 0 && rand_uniform() * 100 <= imu_heater.heat_cmd) {
    gpio_set(IMU_HEATER_GPIO, IMU_HEATER_GPIO_PIN);
  } else {
    gpio_clear(IMU_HEATER_GPIO, IMU_HEATER_GPIO_PIN);
  }
#endif
}

/**
 * @brief 10Hz IMU heater periodic
 * This will run the control loop for the IMU heater and in case
 * of an IOMCU will transmit the duty cycle to the IMU co-processor.
 */
void imu_heater_periodic_10hz(void)
{
  // Calculate the command if we have measurements
  if (imu_heater.meas_temp_cnt > 0) {
    imu_heater.meas_temp = imu_heater.meas_temp_sum / imu_heater.meas_temp_cnt;
    imu_heater.heat_cmd = imu_heater_run(&imu_heater, imu_heater.meas_temp);

    imu_heater.meas_temp_sum = 0;
    imu_heater.meas_temp_cnt = 0;
  }

  // Send to the IOMCU if needed
#if INTERMCU_IOMCU
  iomcu_set_heater_duty_cycle(imu_heater.heat_cmd);
#endif
}

/**
 * @brief Calculate the IMU command percentage
 *
 * @param heater The heater to calculate the command for
 * @param temp The current measured temperature
 * @return float The target command [0-100%]
 */
static float imu_heater_run(struct imu_heater_t *heater, float temp)
{
  if (isnan(temp) || heater->target_temp == -1) {
    return 0;
  }

  // Get the delta time
  uint32_t current_ts = get_sys_time_usec();
  float dt = (current_ts - heater->last_ts) * 1e-6;

  // Calculate the error
  float error = heater->target_temp - temp;

  // Only do integration if dt is correct
  if (heater->last_ts != 0 && dt > 0) {
    // Calculate the integrated value and bound to 70%
    heater->integrated += heater->gain_i * error * dt;
    BoundAbs(heater->integrated, 70);
  }
  heater->last_ts = current_ts;

  // Return the command percentage (0-100% bounded)
  float cmd = error * heater->gain_p + heater->integrated;
  Bound(cmd, 0, 100);
  return cmd;
}

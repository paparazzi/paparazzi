/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file boards/apogee/imu_apogee.c
 *
 * Driver for the IMU on the Apogee board.
 *
 * Invensense MPU-6050
 */

#include <math.h>
#include "boards/apogee/imu_apogee.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "modules/core/abi.h"

// Downlink
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef IMU_APOGEE_CHAN_X
#define IMU_APOGEE_CHAN_X 0
#endif
#ifndef IMU_APOGEE_CHAN_Y
#define IMU_APOGEE_CHAN_Y 1
#endif
#ifndef IMU_APOGEE_CHAN_Z
#define IMU_APOGEE_CHAN_Z 2
#endif

#if !defined APOGEE_LOWPASS_FILTER && !defined  APOGEE_SMPLRT_DIV
#define APOGEE_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define APOGEE_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz")
#endif
PRINT_CONFIG_VAR(APOGEE_SMPLRT_DIV)
PRINT_CONFIG_VAR(APOGEE_LOWPASS_FILTER)

PRINT_CONFIG_VAR(APOGEE_GYRO_RANGE)
PRINT_CONFIG_VAR(APOGEE_ACCEL_RANGE)

#if APOGEE_USE_MPU9150
/** Normal frequency with the default settings
 *
 * the mag read function should be called at around 50 Hz
 */
#ifndef APOGEE_MAG_FREQ
#define APOGEE_MAG_FREQ 50
#endif
PRINT_CONFIG_VAR(APOGEE_MAG_FREQ)
/** Mag periodic prescaler
 */
#define MAG_PRESCALER Max(1,((PERIODIC_FREQUENCY)/APOGEE_MAG_FREQ))
PRINT_CONFIG_VAR(MAG_PRESCALER)

// mag config will be done later in bypass mode
bool configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu);
bool configure_mag_slave(Mpu60x0ConfigSet mpu_set __attribute__((unused)), void *mpu __attribute__((unused)))
{
  return true;
}

#endif

struct ImuApogee imu_apogee;

// baro config will be done later in bypass mode
bool configure_baro_slave(Mpu60x0ConfigSet mpu_set, void *mpu);
bool configure_baro_slave(Mpu60x0ConfigSet mpu_set __attribute__((unused)), void *mpu __attribute__((unused)))
{
  return true;
}

void imu_apogee_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // MPU-60X0
  mpu60x0_i2c_init(&imu_apogee.mpu, &(IMU_APOGEE_I2C_DEV), MPU60X0_ADDR_ALT);
  // change the default configuration
  imu_apogee.mpu.config.smplrt_div = APOGEE_SMPLRT_DIV;
  imu_apogee.mpu.config.dlpf_cfg = APOGEE_LOWPASS_FILTER;
  imu_apogee.mpu.config.gyro_range = APOGEE_GYRO_RANGE;
  imu_apogee.mpu.config.accel_range = APOGEE_ACCEL_RANGE;
  // set MPU in bypass mode for the baro
  imu_apogee.mpu.config.nb_slaves = 1;
  imu_apogee.mpu.config.slaves[0].configure = &configure_baro_slave;
  imu_apogee.mpu.config.i2c_bypass = true;
#if APOGEE_USE_MPU9150
  // if using MPU9150, internal mag needs to be configured
  ak8975_init(&imu_apogee.ak, &(IMU_APOGEE_I2C_DEV), AK8975_I2C_SLV_ADDR);
  imu_apogee.mpu.config.nb_slaves = 2;
  imu_apogee.mpu.config.slaves[1].configure = &configure_mag_slave;
#endif
}

void imu_apogee_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_apogee.mpu);

#if APOGEE_USE_MPU9150
  // Start reading internal mag if available
  RunOnceEvery(MAG_PRESCALER, ak8975_periodic(&imu_apogee.ak));
#endif

  //RunOnceEvery(10,imu_apogee_downlink_raw());
}

void imu_apogee_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
}


void imu_apogee_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the itg3200 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_apogee.mpu);
  if (imu_apogee.mpu.data_available) {
    struct Int32Rates rates = {
        (int32_t)( imu_apogee.mpu.data_rates.value[IMU_APOGEE_CHAN_X]),
        (int32_t)(-imu_apogee.mpu.data_rates.value[IMU_APOGEE_CHAN_Y]),
        (int32_t)(-imu_apogee.mpu.data_rates.value[IMU_APOGEE_CHAN_Z])
    };
    RATES_COPY(imu.gyro_unscaled, rates);
    struct Int32Vect3 accel = {
        (int32_t)( imu_apogee.mpu.data_accel.value[IMU_APOGEE_CHAN_X]),
        (int32_t)(-imu_apogee.mpu.data_accel.value[IMU_APOGEE_CHAN_Y]),
        (int32_t)(-imu_apogee.mpu.data_accel.value[IMU_APOGEE_CHAN_Z])
    };
    VECT3_COPY(imu.accel_unscaled, accel);
    imu_apogee.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

#if APOGEE_USE_MPU9150
  ak8975_event(&imu_apogee.ak);
  if (imu_apogee.ak.data_available) {
    struct Int32Vect3 mag = {
        (int32_t)( imu_apogee.ak.data.value[IMU_APOGEE_CHAN_Y]),
        (int32_t)(-imu_apogee.ak.data.value[IMU_APOGEE_CHAN_X]),
        (int32_t)( imu_apogee.ak.data.value[IMU_APOGEE_CHAN_Z])
    };
    VECT3_COPY(imu.mag_unscaled, mag);
    imu_apogee.ak.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
#endif
}


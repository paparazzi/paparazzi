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
#include "subsystems/abi.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
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

#ifndef APOGEE_GYRO_RANGE
#define APOGEE_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(APOGEE_GYRO_RANGE)

#ifndef APOGEE_ACCEL_RANGE
#define APOGEE_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif
PRINT_CONFIG_VAR(APOGEE_ACCEL_RANGE)

struct ImuApogee imu_apogee;

#ifdef MPU9150_SLV_MAG
struct Ak8975 ak;
#endif

#ifdef MPU9150_SLV_BARO
struct Mpl3115 mpl;
#endif

// baro config will be done later in bypass mode
bool_t configure_baro_slave(Mpu60x0ConfigSet mpu_set, void *mpu);

bool_t configure_baro_slave(Mpu60x0ConfigSet mpu_set __attribute__((unused)), void *mpu __attribute__((unused)))
{
  return TRUE;
}

void imu_impl_init(void)
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
  imu_apogee.mpu.config.i2c_bypass = TRUE;

  
  
#ifdef MPU9150_SLV_MAG

  // Status byte, Accel (3x2 bytes), blank byte, Gyros (3x2 bytes) => 14 bytes
  imu_apogee.mpu.config.nb_bytes = 15;

  // Extended sens bytes :
  //   Mag: status byte + 3x2 bytes + status byte => 8 bytes
  imu_apogee.mpu.config.nb_bytes += 8;
  imu_apogee.mpu.config.nb_slaves = 1; 

  imu_apogee.mpu.config.slaves[MPU_MAG_SLV_NB].configure = &mpu9150_i2c_configure_mag_slave;
  imu_apogee.mpu.config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status = 0;
  imu_apogee.mpu.config.slaves[MPU_MAG_SLV_NB].mpu_slave_privateData = &ak;

  ak8975_init(&ak, &(IMU_APOGEE_I2C_DEV), (AK8975_I2C_SLV_ADDR<<1));

  imu_apogee.mpu.config.i2c_bypass = FALSE;

  imu_apogee.mag_valid = FALSE;

#endif

#ifdef MPU9150_SLV_BARO

  // Extended sens bytes :
  //  Baro (3 bytes)
  imu_apogee.mpu.config.nb_bytes += 3;
  imu_apogee.mpu.config.nb_slaves += 1;

  imu_apogee.mpu.config.slaves[MPU_BARO_SLV_NB].configure = &mpu9150_i2c_configure_baro_slave;
  imu_apogee.mpu.config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status = 0;
  imu_apogee.mpu.config.slaves[MPU_BARO_SLV_NB].mpu_slave_privateData = &mpl;

  mpl3115_init(&mpl, &(IMU_APOGEE_I2C_DEV), MPL3115_I2C_ADDR);

#endif
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_apogee.mpu);

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

#ifdef MPU9150_SLV_MAG
   struct Int32Vect3 mag={0,0,0};
#endif
#ifdef MPU9150_SLV_BARO
   float pressure = 0;
#endif

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
    imu_apogee.mpu.data_available = FALSE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);

#ifdef MPU9150_SLV_MAG
    imu_apogee.mag_valid = mpu9150_i2c_mag_event(&imu_apogee.mpu, &mag);
    if(imu_apogee.mag_valid) VECT3_COPY(imu.mag_unscaled, mag);
#endif

#ifdef MPU9150_SLV_BARO
    mpu9150_i2c_baro_event(&imu_apogee.mpu, &pressure);
    AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
#endif

  }
}


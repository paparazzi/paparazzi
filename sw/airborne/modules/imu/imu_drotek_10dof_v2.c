/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/imu/imu_drotek_10dof_v2.c
 *
 * Driver for the Drotek 10DOF V2 IMU.
 * MPU6050 + HMC5883 + MS5611
 * Reading the baro is not part of the IMU driver.
 *
 * By default the axes orientation should be as printed on the pcb,
 * meaning z-axis pointing down if ICs are facing down.
 * The orientation can be switched so that the IMU can be mounted ICs facing up
 * by defining IMU_DROTEK_2_ORIENTATION_IC_UP.
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"

#if !defined DROTEK_2_LOWPASS_FILTER && !defined  DROTEK_2_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define DROTEK_2_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define DROTEK_2_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define DROTEK_2_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define DROTEK_2_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(DROTEK_2_SMPLRT_DIV)
PRINT_CONFIG_VAR(DROTEK_2_LOWPASS_FILTER)

PRINT_CONFIG_VAR(DROTEK_2_GYRO_RANGE)
PRINT_CONFIG_VAR(DROTEK_2_ACCEL_RANGE)

#ifndef DROTEK_2_MPU_I2C_ADDR
#define DROTEK_2_MPU_I2C_ADDR MPU60X0_ADDR_ALT
#endif
PRINT_CONFIG_VAR(DROTEK_2_MPU_I2C_ADDR)

#ifndef DROTEK_2_HMC_I2C_ADDR
#define DROTEK_2_HMC_I2C_ADDR HMC58XX_ADDR
#endif
PRINT_CONFIG_VAR(DROTEK_2_HMC_I2C_ADDR)


struct ImuDrotek2 imu_drotek2;

void imu_drotek2_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_drotek2.mpu, &(DROTEK_2_I2C_DEV), DROTEK_2_MPU_I2C_ADDR);
  // change the default configuration
  imu_drotek2.mpu.config.smplrt_div = DROTEK_2_SMPLRT_DIV;
  imu_drotek2.mpu.config.dlpf_cfg = DROTEK_2_LOWPASS_FILTER;
  imu_drotek2.mpu.config.gyro_range = DROTEK_2_GYRO_RANGE;
  imu_drotek2.mpu.config.accel_range = DROTEK_2_ACCEL_RANGE;

  /* HMC5883 magnetometer */
  hmc58xx_init(&imu_drotek2.hmc, &(DROTEK_2_I2C_DEV), DROTEK_2_HMC_I2C_ADDR);

  /* mag is declared as slave to call the configure function,
   * regardless if it is an actual MPU slave or passthrough
   */
  imu_drotek2.mpu.config.nb_slaves = 1;
  /* set callback function to configure mag */
  imu_drotek2.mpu.config.slaves[0].configure = &imu_drotek2_configure_mag_slave;

  // use hmc mag via passthrough
  imu_drotek2.mpu.config.i2c_bypass = true;
}

void imu_drotek2_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_drotek2.mpu);

  // Read HMC58XX at ~50Hz (main loop for rotorcraft: 512Hz)
  if (imu_drotek2.mpu.config.initialized) {
    RunOnceEvery(10, hmc58xx_read(&imu_drotek2.hmc));
  }
}

void imu_drotek2_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU6050 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_drotek2.mpu);

  if (imu_drotek2.mpu.data_available) {
#if IMU_DROTEK_2_ORIENTATION_IC_UP
    /* change orientation, so if ICs face up, z-axis is down */
    imu.gyro_unscaled.p = imu_drotek2.mpu.data_rates.rates.p;
    imu.gyro_unscaled.q = -imu_drotek2.mpu.data_rates.rates.q;
    imu.gyro_unscaled.r = -imu_drotek2.mpu.data_rates.rates.r;
    imu.accel_unscaled.x = imu_drotek2.mpu.data_accel.vect.x;
    imu.accel_unscaled.y = -imu_drotek2.mpu.data_accel.vect.y;
    imu.accel_unscaled.z = -imu_drotek2.mpu.data_accel.vect.z;
#else
    /* default orientation as should be printed on the pcb, z-down, ICs down */
    RATES_COPY(imu.gyro_unscaled, imu_drotek2.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_drotek2.mpu.data_accel.vect);
#endif

    imu_drotek2.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_DROTEK_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_DROTEK_ID, now_ts, &imu.accel);
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_drotek2.hmc);
  if (imu_drotek2.hmc.data_available) {
#if IMU_DROTEK_2_ORIENTATION_IC_UP
    imu.mag_unscaled.x = imu_drotek2.hmc.data.vect.x;
    imu.mag_unscaled.y = -imu_drotek2.hmc.data.vect.y;
    imu.mag_unscaled.z = -imu_drotek2.hmc.data.vect.z;
#else
    VECT3_COPY(imu.mag_unscaled, imu_drotek2.hmc.data.vect);
#endif
    imu_drotek2.hmc.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_DROTEK_ID, now_ts, &imu.mag);
  }
}

/** callback function to configure hmc5883 mag
 * @return TRUE if mag configuration finished
 */
bool imu_drotek2_configure_mag_slave(Mpu60x0ConfigSet mpu_set __attribute__((unused)),
                                       void *mpu __attribute__((unused)))
{
  hmc58xx_start_configure(&imu_drotek2.hmc);
  if (imu_drotek2.hmc.initialized) {
    return true;
  } else {
    return false;
  }
}

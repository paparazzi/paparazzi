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
 * @file modules/imu/imu_px4fmu.c
 * Driver for the PX4FMU SPI1 for the MPU6000 and I2C2 for the HMC5883.
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/hmc58xx_regs.h"


/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined PX4FMU_LOWPASS_FILTER && !defined  PX4FMU_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define PX4FMU_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define PX4FMU_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define PX4FMU_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define PX4FMU_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define PX4FMU_LOWPASS_FILTER and PX4FMU_SMPLRT_DIV.
#endif
#endif
PRINT_CONFIG_VAR(PX4FMU_LOWPASS_FILTER)
PRINT_CONFIG_VAR(PX4FMU_SMPLRT_DIV)

PRINT_CONFIG_VAR(PX4FMU_GYRO_RANGE)
PRINT_CONFIG_VAR(PX4FMU_ACCEL_RANGE)

struct ImuPx4fmu imu_px4fmu;

void imu_px4fmu_init(void)
{
  /* MPU is on spi1 and CS is SLAVE2 */
  mpu60x0_spi_init(&imu_px4fmu.mpu, &spi1, SPI_SLAVE2);
  // change the default configuration
  imu_px4fmu.mpu.config.smplrt_div = PX4FMU_SMPLRT_DIV;
  imu_px4fmu.mpu.config.dlpf_cfg = PX4FMU_LOWPASS_FILTER;
  imu_px4fmu.mpu.config.gyro_range = PX4FMU_GYRO_RANGE;
  imu_px4fmu.mpu.config.accel_range = PX4FMU_ACCEL_RANGE;

  /* initialize mag on i2c2 and set default options */
  hmc58xx_init(&imu_px4fmu.hmc, &i2c2, HMC58XX_ADDR);
}


void imu_px4fmu_periodic(void)
{
  mpu60x0_spi_periodic(&imu_px4fmu.mpu);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_px4fmu.hmc));
}

void imu_px4fmu_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  mpu60x0_spi_event(&imu_px4fmu.mpu);
  if (imu_px4fmu.mpu.data_available) {
    RATES_ASSIGN(imu.gyro_unscaled,
                 imu_px4fmu.mpu.data_rates.rates.q,
                 imu_px4fmu.mpu.data_rates.rates.p,
                 -imu_px4fmu.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled,
                 imu_px4fmu.mpu.data_accel.vect.y,
                 imu_px4fmu.mpu.data_accel.vect.x,
                 -imu_px4fmu.mpu.data_accel.vect.z);
    imu_px4fmu.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_px4fmu.hmc);
  if (imu_px4fmu.hmc.data_available) {
    imu.mag_unscaled.x =  imu_px4fmu.hmc.data.vect.y;
    imu.mag_unscaled.y =  imu_px4fmu.hmc.data.vect.x;
    imu.mag_unscaled.z = -imu_px4fmu.hmc.data.vect.z;
    imu_px4fmu.hmc.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
}


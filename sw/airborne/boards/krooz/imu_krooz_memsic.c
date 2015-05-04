/*
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
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
 * @file boards/krooz/imu_krooz_memsic.c
 *
 * Driver for the IMU on the KroozSD Big Rotorcraft Edition board.
 *
 * Invensense MPU-6050
 * Memsic MXR9500 with AD7689
 * Honeywell HMC-5883
 */

#include <math.h>
#include "boards/krooz/imu_krooz_memsic.h"
#include "subsystems/imu/imu_krooz_sd_arch.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "filters/median_filter.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"

#if !defined KROOZ_LOWPASS_FILTER && !defined  KROOZ_SMPLRT_DIV
#define KROOZ_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define KROOZ_SMPLRT_DIV 1
#endif
PRINT_CONFIG_VAR(KROOZ_SMPLRT_DIV)
PRINT_CONFIG_VAR(KROOZ_LOWPASS_FILTER)

#ifndef KROOZ_GYRO_RANGE
#define KROOZ_GYRO_RANGE MPU60X0_GYRO_RANGE_250
#endif
PRINT_CONFIG_VAR(KROOZ_GYRO_RANGE)

#ifndef KROOZ_ACCEL_RANGE
#define KROOZ_ACCEL_RANGE MPU60X0_ACCEL_RANGE_2G
#endif
PRINT_CONFIG_VAR(KROOZ_ACCEL_RANGE)

struct ImuKrooz imu_krooz;

#if IMU_KROOZ_USE_ACCEL_MEDIAN_FILTER
struct MedianFilter3Int median_accel;
#endif
struct MedianFilter3Int median_mag;

static uint32_t ad7689_event_timer;
static uint8_t axis_cnt;
static uint8_t axis_nb;

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // MPU-60X0
  mpu60x0_i2c_init(&imu_krooz.mpu, &(IMU_KROOZ_I2C_DEV), MPU60X0_ADDR);
  // change the default configuration
  imu_krooz.mpu.config.smplrt_div = KROOZ_SMPLRT_DIV;
  imu_krooz.mpu.config.dlpf_cfg = KROOZ_LOWPASS_FILTER;
  imu_krooz.mpu.config.gyro_range = KROOZ_GYRO_RANGE;
  imu_krooz.mpu.config.accel_range = KROOZ_ACCEL_RANGE;
  imu_krooz.mpu.config.drdy_int_enable = TRUE;

  hmc58xx_init(&imu_krooz.hmc, &(IMU_KROOZ_I2C_DEV), HMC58XX_ADDR);

  // Init median filters
#if IMU_KROOZ_USE_ACCEL_MEDIAN_FILTER
  InitMedianFilterVect3Int(median_accel);
#endif
  InitMedianFilterVect3Int(median_mag);

  RATES_ASSIGN(imu_krooz.rates_sum, 0, 0, 0);
  VECT3_ASSIGN(imu_krooz.accel_sum, 0, 0, 0);
  imu_krooz.meas_nb = 0;

  imu_krooz.hmc_eoc = FALSE;
  imu_krooz.mpu_eoc = FALSE;

  imu_krooz.ad7689_trans.slave_idx     = IMU_KROOZ_SPI_SLAVE_IDX;
  imu_krooz.ad7689_trans.select        = SPISelectUnselect;
  imu_krooz.ad7689_trans.cpol          = SPICpolIdleLow;
  imu_krooz.ad7689_trans.cpha          = SPICphaEdge1;
  imu_krooz.ad7689_trans.dss           = SPIDss8bit;
  imu_krooz.ad7689_trans.bitorder      = SPIMSBFirst;
  imu_krooz.ad7689_trans.cdiv          = SPIDiv16;
  imu_krooz.ad7689_trans.output_length = sizeof(imu_krooz.ad7689_spi_tx_buffer);
  imu_krooz.ad7689_trans.output_buf    = (uint8_t *) imu_krooz.ad7689_spi_tx_buffer;
  imu_krooz.ad7689_trans.input_length  = sizeof(imu_krooz.ad7689_spi_rx_buffer);
  imu_krooz.ad7689_trans.input_buf     = (uint8_t *) imu_krooz.ad7689_spi_rx_buffer;
  imu_krooz.ad7689_trans.before_cb     = NULL;
  imu_krooz.ad7689_trans.after_cb      = NULL;
  axis_cnt = 0;
  axis_nb = 2;

  imu_krooz_sd_arch_init();
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  if (!imu_krooz.mpu.config.initialized) {
    mpu60x0_i2c_start_configure(&imu_krooz.mpu);
  }

  if (!imu_krooz.hmc.initialized) {
    hmc58xx_start_configure(&imu_krooz.hmc);
  }

  uint32_t now_ts = get_sys_time_usec();

  if (imu_krooz.meas_nb) {
    RATES_ASSIGN(imu.gyro_unscaled, -imu_krooz.rates_sum.q / imu_krooz.meas_nb,
                 imu_krooz.rates_sum.p / imu_krooz.meas_nb,
                 imu_krooz.rates_sum.r / imu_krooz.meas_nb);

    RATES_ASSIGN(imu_krooz.rates_sum, 0, 0, 0);
    imu_krooz.meas_nb = 0;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
  }

  if (imu_krooz.meas_nb_acc.x && imu_krooz.meas_nb_acc.y && imu_krooz.meas_nb_acc.z) {
    imu.accel_unscaled.x = 65536 - imu_krooz.accel_sum.x / imu_krooz.meas_nb_acc.x;
    imu.accel_unscaled.y = 65536 - imu_krooz.accel_sum.y / imu_krooz.meas_nb_acc.y;
    imu.accel_unscaled.z = imu_krooz.accel_sum.z / imu_krooz.meas_nb_acc.z;

#if IMU_KROOZ_USE_ACCEL_MEDIAN_FILTER
    UpdateMedianFilterVect3Int(median_accel, imu.accel_unscaled);
#endif
    VECT3_SMUL(imu_krooz.accel_filtered, imu_krooz.accel_filtered, IMU_KROOZ_ACCEL_AVG_FILTER);
    VECT3_ADD(imu_krooz.accel_filtered, imu.accel_unscaled);
    VECT3_SDIV(imu_krooz.accel_filtered, imu_krooz.accel_filtered, (IMU_KROOZ_ACCEL_AVG_FILTER + 1));
    VECT3_COPY(imu.accel_unscaled, imu_krooz.accel_filtered);

    INT_VECT3_ZERO(imu_krooz.accel_sum);
    INT_VECT3_ZERO(imu_krooz.meas_nb_acc);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  RunOnceEvery(128, {axis_nb = 5;});
}

void imu_krooz_event(void)
{
  if (imu_krooz.mpu_eoc) {
    mpu60x0_i2c_read(&imu_krooz.mpu);
    imu_krooz.mpu_eoc = FALSE;
  }

  // If the MPU6050 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_krooz.mpu);
  if (imu_krooz.mpu.data_available) {
    RATES_ADD(imu_krooz.rates_sum, imu_krooz.mpu.data_rates.rates);
    imu_krooz.meas_nb++;
    imu_krooz.mpu.data_available = FALSE;
  }

  if (SysTimeTimer(ad7689_event_timer) > 215) {
    SysTimeTimerStart(ad7689_event_timer);
    if (axis_cnt < axis_nb) {
      axis_cnt++;
    } else {
      axis_cnt = 0;
    }
    imu_krooz.ad7689_trans.output_buf[0] =
      axis_cnt <= 2 ? 0xF0 | (axis_cnt << 1) : (axis_cnt >= 4 ? 0xF0 | ((axis_cnt - 3) << 1) : 0xB0);
    imu_krooz.ad7689_trans.output_buf[1] = 0x44;
    spi_submit(&(IMU_KROOZ_SPI_DEV), &imu_krooz.ad7689_trans);
  }
  if (imu_krooz.ad7689_trans.status == SPITransSuccess) {
    imu_krooz.ad7689_trans.status = SPITransDone;
    uint16_t buf = (imu_krooz.ad7689_trans.input_buf[0] << 8) | imu_krooz.ad7689_trans.input_buf[1];
    switch (axis_cnt) {
      case 0:
      case 3:
        imu_krooz.accel_sum.x += (int32_t)buf;
        imu_krooz.meas_nb_acc.x++;
        break;
      case 1:
      case 4:
        imu_krooz.accel_sum.y += (int32_t)buf;
        imu_krooz.meas_nb_acc.y++;
        break;
      case 2:
        imu_krooz.accel_sum.z += (int32_t)buf;
        imu_krooz.meas_nb_acc.z++;
        break;
      case 5:
        imu_krooz.temperature = (imu_krooz.temperature * 4 + (int32_t)buf) / 5;
        //imu.temperature = 33000 * imu_krooz.temp / 65536 - 2400;
        axis_nb = 2;
        break;
      default:
        axis_cnt = 0;
        break;
    }
  }

  if (imu_krooz.hmc_eoc) {
    hmc58xx_read(&imu_krooz.hmc);
    imu_krooz.hmc_eoc = FALSE;
  }

  // If the HMC5883 I2C transaction has succeeded: convert the data
  hmc58xx_event(&imu_krooz.hmc);
  if (imu_krooz.hmc.data_available) {
    VECT3_ASSIGN(imu.mag_unscaled, imu_krooz.hmc.data.vect.y, -imu_krooz.hmc.data.vect.x, imu_krooz.hmc.data.vect.z);
    UpdateMedianFilterVect3Int(median_mag, imu.mag_unscaled);
    imu_krooz.hmc.data_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, get_sys_time_usec(), &imu.mag);
  }
}

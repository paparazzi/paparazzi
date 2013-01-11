/*
 * Copyright (C) 2012 The Paparazzi Team
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
 * @file subsystems/imu/imu_aspirin.c
 * Driver for the Aspirin v1.x IMU.
 */

#include "subsystems/imu.h"
//#include "peripherals/hmc5843.h"
#include "peripherals/hmc58xx.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"

#ifndef ASPIRIN_ACCEL_RATE
#define ASPIRIN_ACCEL_RATE ADXL345_RATE_800
INFO("Using default ASPIRIN_ACCEL_RATE of 800Hz.")
#endif

#ifndef ADXL345_SLAVE_IDX
#define ADXL345_SLAVE_IDX SPI_SLAVE2
#endif

#ifndef ADXL345_SPI_DEV
#define ADXL345_SPI_DEV spi2
#endif


#ifdef ASPIRIN_GYRO_LOWPASS_HZ
PRINT_CONFIG_VAR(ASPIRIN_GYRO_LOWPASS_HZ)
#define __DLPF(x, hz) x##hz
#define _DLPF(hz) __DLPF(ITG3200_DLPF_, hz)
#define ASPIRIN_GYRO_DLPF_CFG _DLPF(ASPIRIN_GYRO_LOWPASS_HZ)
#else
#define ASPIRIN_GYRO_DLPF_CFG ITG3200_DLPF_256HZ
INFO("Using default ASPIRIN_GYRO_LOWPASS of 256HZ")
#endif
//PRINT_CONFIG_VAR(ASPIRIN_GYRO_DLPF_CFG)

/// Default sample rate divider: with default DLPF -> 533Hz
#ifndef ASPIRIN_GYRO_SMPLRT_DIV
#define ASPIRIN_GYRO_SMPLRT_DIV 14
#endif
//PRINT_CONFIG_VAR(ASPIRIN_GYRO_SMPLRT_DIV)


struct ImuAspirin imu_aspirin;

struct spi_transaction aspirin_adxl345;

void adxl345_write_to_reg(uint8_t _reg, uint8_t _val);
static void adxl345_trans_cb( struct spi_transaction *trans );
static void configure_accel(void);
static void accel_copy_spi(void);

void imu_impl_init(void) {

  imu_aspirin.status = AspirinStatusUninit;
  imu_aspirin.accel_valid = FALSE;
  imu_aspirin.gyro_valid = FALSE;
  imu_aspirin.mag_valid = FALSE;

  aspirin_adxl345.select = SPISelectUnselect;
  aspirin_adxl345.cpol = SPICpolIdleHigh;
  aspirin_adxl345.cpha = SPICphaEdge2;
  aspirin_adxl345.dss = SPIDss8bit;
  aspirin_adxl345.bitorder = SPIMSBFirst;
  aspirin_adxl345.cdiv = SPIDiv64;
  aspirin_adxl345.slave_idx = ADXL345_SLAVE_IDX;
  aspirin_adxl345.output_length = 7;
  aspirin_adxl345.input_length = 7;
  aspirin_adxl345.after_cb = adxl345_trans_cb;
  aspirin_adxl345.input_buf = &imu_aspirin.accel_rx_buf[0];
  aspirin_adxl345.output_buf = &imu_aspirin.accel_tx_buf[0];

  /* Gyro configuration and initalization */
  itg3200_init(&imu_aspirin.gyro_itg, &(IMU_ASPIRIN_I2C_DEVICE), ITG3200_ADDR);
  /* change the default config */
  imu_aspirin.gyro_itg.config.smplrt_div = ASPIRIN_GYRO_SMPLRT_DIV; // Sample rate divider defaults to 533Hz
  imu_aspirin.gyro_itg.config.dlpf_cfg = ASPIRIN_GYRO_DLPF_CFG; // defaults to 8kHz internal with 256Hz low pass

  /// @todo eoc interrupt for itg3200, polling for now (including status reg)
  /* interrupt on data ready, idle high, latch until read any register */
  //itg_conf.int_cfg = (0x01 | (0x1<<4) | (0x1<<5) | 0x01<<7);


  imu_aspirin_arch_init();
  hmc58xx_init(&imu_aspirin.mag_hmc, TRUE);

}


void imu_periodic(void) {
  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, Hmc58xxPeriodic(imu_aspirin.mag_hmc));

  // Start reading the latest gyroscope data
  Itg3200Periodic(imu_aspirin.gyro_itg);

  if (imu_aspirin.status == AspirinStatusUninit) {
    configure_accel();
    //imu_aspirin_arch_int_enable();
    imu_aspirin.accel_tx_buf[0] = (1<<7|1<<6|ADXL345_REG_DATA_X0);
    imu_aspirin.status = AspirinStatusIdle;
  } else {
    spi_submit(&(ADXL345_SPI_DEV), &aspirin_adxl345);
  }
}

static void adxl345_trans_cb( struct spi_transaction *trans ) {
  if ( imu_aspirin.status != AspirinStatusUninit ) {
    imu_aspirin.accel_valid = TRUE;
  }
}

void adxl345_write_to_reg(uint8_t _reg, uint8_t _val) {
  imu_aspirin.accel_tx_buf[0] = _reg;
  imu_aspirin.accel_tx_buf[1] = _val;
  spi_submit(&(ADXL345_SPI_DEV), &aspirin_adxl345);

  // FIXME: no busy waiting! if really needed add a timeout!!!!
  while(aspirin_adxl345.status != SPITransSuccess);
}

static void configure_accel(void) {
  /* set data rate to 800Hz and bandwidth to 400Hz (unless set otherwise) */
  adxl345_write_to_reg(ADXL345_REG_BW_RATE, ASPIRIN_ACCEL_RATE);
  /* switch to measurememnt mode */
  adxl345_write_to_reg(ADXL345_REG_POWER_CTL, 1<<3);
  /* enable data ready interrupt */
  adxl345_write_to_reg(ADXL345_REG_INT_ENABLE, 1<<7);
  /* Enable full res with +-16g range and interrupt active low */
  adxl345_write_to_reg(ADXL345_REG_DATA_FORMAT, ADXL345_FULL_RES|ADXL345_RANGE_16G|ADXL345_INT_INVERT);
}

static void accel_copy_spi(void) {
  const int16_t ax = imu_aspirin.accel_rx_buf[1] | (imu_aspirin.accel_rx_buf[2]<<8);
  const int16_t ay = imu_aspirin.accel_rx_buf[3] | (imu_aspirin.accel_rx_buf[4]<<8);
  const int16_t az = imu_aspirin.accel_rx_buf[5] | (imu_aspirin.accel_rx_buf[6]<<8);
  VECT3_ASSIGN(imu.accel_unscaled, ax, ay, az);
}

void imu_aspirin_event(void) {

  //imu_aspirin_arch_int_disable();
  if (imu_aspirin.accel_valid) {
    imu_aspirin.accel_valid = FALSE;
    accel_copy_spi();
  }
  //imu_aspirin_arch_int_enable();

  /* If the itg3200 I2C transaction has succeeded: convert the data */
  itg3200_event(&imu_aspirin.gyro_itg);
  if (imu_aspirin.gyro_itg.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_aspirin.gyro_itg.data.rates);
    imu_aspirin.gyro_itg.data_available = FALSE;
    imu_aspirin.gyro_valid = TRUE;
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_aspirin.mag_hmc);
  if (imu_aspirin.mag_hmc.data_available) {
    imu.mag_unscaled.x = imu_aspirin.mag_hmc.data.value[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = imu_aspirin.mag_hmc.data.value[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = imu_aspirin.mag_hmc.data.value[IMU_MAG_Z_CHAN];
    imu_aspirin.mag_hmc.data_available = FALSE;
    imu_aspirin.mag_valid = TRUE;
  }

}

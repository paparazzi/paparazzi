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
#include "peripherals/hmc5843.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"

#ifndef ASPIRIN_ACCEL_RATE
#define ASPIRIN_ACCEL_RATE ADXL345_RATE_800
#endif

#ifndef ADXL345_SLAVE_IDX
#define ADXL345_SLAVE_IDX SPI_SLAVE2
#endif

#ifndef ADXL345_SPI_DEV
#define ADXL345_SPI_DEV spi2
#endif

#ifndef ITG3200_I2C_DEV
#define ITG3200_I2C_DEV i2c2
#endif

struct ImuAspirin imu_aspirin;

struct spi_transaction aspirin_adxl345;

void adxl345_write_to_reg(uint8_t _reg, uint8_t _val);
static void adxl345_trans_cb( struct spi_transaction *trans );

/* initialize peripherals */
static void configure_gyro(void);
static void configure_accel(void);
//static void configure_mag(void);


// FIXME: there should be no arch dependent code here!
static void send_i2c_msg_with_retry(struct i2c_transaction* t) {
  uint8_t max_retry = 8;
  uint8_t nb_retry = 0;
  do {
    i2c_submit(&i2c2, t);
    while((I2C2_SR2 & I2C_SR2_BUSY));
    //while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    while (t->status == I2CTransPending || t->status == I2CTransRunning);
    if (t->status == I2CTransFailed)
      nb_retry++;
  }
  while (t->status != I2CTransSuccess && nb_retry < max_retry);
}

void imu_impl_init(void) {

  imu_aspirin.status = AspirinStatusUninit;
  imu_aspirin.gyro_available_blaaa = FALSE;
  imu_aspirin.mag_available = FALSE;
  imu_aspirin.accel_available = FALSE;

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

  imu_aspirin.i2c_trans_gyro.type = I2CTransTxRx;
  imu_aspirin.i2c_trans_gyro.buf[0] = ITG3200_REG_GYRO_XOUT_H;
  imu_aspirin.i2c_trans_gyro.slave_addr = ITG3200_ADDR;
  imu_aspirin.i2c_trans_gyro.len_w = 1;
  imu_aspirin.i2c_trans_gyro.len_r = 6;
  imu_aspirin.i2c_trans_gyro.status = I2CTransFailed;

  imu_aspirin_arch_init();
  hmc5843_init();

}


void imu_periodic(void) {
  hmc5843_periodic();
  if (imu_aspirin.status == AspirinStatusUninit) {
    configure_gyro();
    configure_accel();
    //imu_aspirin_arch_int_enable();
    imu_aspirin.accel_tx_buf[0] = (1<<7|1<<6|ADXL345_REG_DATA_X0);
    imu_aspirin.status = AspirinStatusIdle;
  } else {
    imu_aspirin.gyro_available_blaaa = TRUE;
    imu_aspirin.time_since_last_reading++;
    imu_aspirin.time_since_last_accel_reading++;
    spi_submit(&(ADXL345_SPI_DEV), &aspirin_adxl345);

    //if (imu_aspirin.time_since_last_accel_reading > ASPIRIN_ACCEL_TIMEOUT) {
    //  configure_accel();
    //  imu_aspirin.time_since_last_accel_reading=0;
    //}
  }
}

static void adxl345_trans_cb( struct spi_transaction *trans ) {
  if ( imu_aspirin.status != AspirinStatusUninit ) {
    imu_aspirin.accel_available = TRUE;
  }
}

/* sends a serie of I2C commands to configure the ITG3200 gyro */
static void configure_gyro(void) {
  struct i2c_transaction t;
  t.type = I2CTransTx;
  t.slave_addr = ITG3200_ADDR;
  /* set gyro range to 2000deg/s and low pass at 256Hz */
  t.buf[0] = ITG3200_REG_DLPF_FS;
  t.buf[1] = (0x03<<3);
  t.len_w = 2;
  //send_i2c_msg_with_retry(&t);
  i2c_submit(&(ITG3200_I2C_DEV), &t);
  /* set sample rate to 533Hz */
  t.buf[0] = ITG3200_REG_SMPLRT_DIV;
  t.buf[1] = 0x0E;
  //send_i2c_msg_with_retry(&t);
  i2c_submit(&(ITG3200_I2C_DEV), &t);
  /* switch to gyroX clock */
  t.buf[0] = ITG3200_REG_PWR_MGM;
  t.buf[1] = 0x01;
  //send_i2c_msg_with_retry(&t);
  i2c_submit(&(ITG3200_I2C_DEV), &t);
  /* enable interrupt on data ready, idle high, latch until read any register */
  t.buf[0] = ITG3200_REG_INT_CFG;
  t.buf[1] = (0x01 | (0x1<<4) | (0x1<<5) | 0x01<<7);
  //send_i2c_msg_with_retry(&t);
  i2c_submit(&(ITG3200_I2C_DEV), &t);
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

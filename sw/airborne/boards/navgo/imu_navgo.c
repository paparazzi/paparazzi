/*
 * Copyright (C) 2011 Gautier Hattenberger
 * Derived from Aspirin and ppzuavimu drivers
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
 *
 */

#include <math.h>
#include "imu_navgo.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


// Peripherials
#include "peripherals/itg3200.h"
#include "peripherals/adxl345.h"
#include "peripherals/hmc58xx.h"

// Results
volatile bool_t gyr_valid;
volatile bool_t acc_valid;
volatile bool_t mag_valid;

// Communication
struct i2c_transaction imu_navgo_itg3200;
struct i2c_transaction imu_navgo_adxl345;

#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 60
#endif

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // ITG3200
  imu_navgo_itg3200.type = I2CTransTx;
  imu_navgo_itg3200.slave_addr = ITG3200_ADDR_ALT;
  imu_navgo_itg3200.buf[0] = ITG3200_REG_DLPF_FS;
  /* set gyro range to 2000deg/s and low pass at 42Hz (< 120Hz/2) internal sampling at 1kHz */
  imu_navgo_itg3200.buf[1] = (0x03<<3) | (0x03<<0);
  imu_navgo_itg3200.len_w = 2;
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_itg3200);
  while(imu_navgo_itg3200.status == I2CTransPending);

  /* set sample rate to 66Hz: so at 60Hz there is always a new sample ready and you loose little */
  imu_navgo_itg3200.buf[0] = ITG3200_REG_SMPLRT_DIV;
  imu_navgo_itg3200.buf[1] = 1;  // 500Hz
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_itg3200);
  while(imu_navgo_itg3200.status == I2CTransPending);

  /* switch to gyroX clock */
  imu_navgo_itg3200.buf[0] = ITG3200_REG_PWR_MGM;
  imu_navgo_itg3200.buf[1] = 0x01;
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_itg3200);
  while(imu_navgo_itg3200.status == I2CTransPending);

  /* no interrupts for now, but set data ready interrupt to enable reading status bits */
  imu_navgo_itg3200.buf[0] = ITG3200_REG_INT_CFG;
  imu_navgo_itg3200.buf[1] = 0x00;
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_itg3200);
  while(imu_navgo_itg3200.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // ADXL345

  /* set data rate to 100Hz */
  imu_navgo_adxl345.slave_addr = ADXL345_ADDR_ALT;
  imu_navgo_adxl345.type = I2CTransTx;
  imu_navgo_adxl345.buf[0] = ADXL345_REG_BW_RATE;
  imu_navgo_adxl345.buf[1] = 0x0a;  // normal power and 100Hz sampling, 50Hz BW
  imu_navgo_adxl345.len_w = 2;
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_adxl345);
  while(imu_navgo_adxl345.status == I2CTransPending);

  /* switch to measurement mode */
  imu_navgo_adxl345.type = I2CTransTx;
  imu_navgo_adxl345.buf[0] = ADXL345_REG_POWER_CTL;
  imu_navgo_adxl345.buf[1] = 1<<3;
  imu_navgo_adxl345.len_w = 2;
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_adxl345);
  while(imu_navgo_adxl345.status == I2CTransPending);

  /* Set range to 16g but keeping full resolution of 3.9 mV/g */
  imu_navgo_adxl345.type = I2CTransTx;
  imu_navgo_adxl345.buf[0] = ADXL345_REG_DATA_FORMAT;
  imu_navgo_adxl345.buf[1] = 1<<3 | 0<<2 | 0x03;  // bit 3 is full resolution bit, bit 2 is left justify bit 0,1 are range: 00=2g 01=4g 10=8g 11=16g
  imu_navgo_adxl345.len_w = 2;
  i2c_submit(&IMU_NAVGO_I2C_DEVICE,&imu_navgo_adxl345);
  while(imu_navgo_adxl345.status == I2CTransPending);


  ///////////////////////////////////////////////////////////////////////
  // HMC58XX
  hmc58xx_init();
}

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  if (imu_navgo_itg3200.status == I2CTransDone) {
    imu_navgo_itg3200.type = I2CTransTxRx;
    imu_navgo_itg3200.len_r = 9;
    imu_navgo_itg3200.len_w = 1;
    imu_navgo_itg3200.buf[0] = ITG3200_REG_INT_STATUS;
    i2c_submit(&IMU_NAVGO_I2C_DEVICE, &imu_navgo_itg3200);
  }

  // Start reading the latest accelerometer data
  if (imu_navgo_adxl345.status == I2CTransDone) {
    imu_navgo_adxl345.type = I2CTransTxRx;
    imu_navgo_adxl345.len_r = 6;
    imu_navgo_adxl345.len_w = 1;
    imu_navgo_adxl345.buf[0] = ADXL345_REG_DATA_X0;
    i2c_submit(&IMU_NAVGO_I2C_DEVICE, &imu_navgo_adxl345);
  }

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  //RunOnceEvery(10,hmc58xx_periodic());
  hmc58xx_periodic();

  RunOnceEvery(20,imu_navgo_downlink_raw());
}


void imu_navgo_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,&imu.mag_unscaled.x,&imu.mag_unscaled.y,&imu.mag_unscaled.z);
}


void imu_navgo_event( void )
{
  int32_t x, y, z;

  // If the itg3200 I2C transaction has succeeded: convert the data
  if (imu_navgo_itg3200.status == I2CTransSuccess) {
#define ITG_STA_DAT_OFFSET 3
    x = (int16_t) ((imu_navgo_itg3200.buf[0+ITG_STA_DAT_OFFSET] << 8) | imu_navgo_itg3200.buf[1+ITG_STA_DAT_OFFSET]);
    y = (int16_t) ((imu_navgo_itg3200.buf[2+ITG_STA_DAT_OFFSET] << 8) | imu_navgo_itg3200.buf[3+ITG_STA_DAT_OFFSET]);
    z = (int16_t) ((imu_navgo_itg3200.buf[4+ITG_STA_DAT_OFFSET] << 8) | imu_navgo_itg3200.buf[5+ITG_STA_DAT_OFFSET]);

    // Is this is new data
    if (imu_navgo_itg3200.buf[0] & 0x01) {
      gyr_valid = TRUE;
    }

    RATES_ASSIGN(imu.gyro_unscaled,
        (/*IMU_GYRO_P_SIGN * */x),
        (/*IMU_GYRO_Q_SIGN * */y),
        (/*IMU_GYRO_R_SIGN * */z));

    imu_navgo_itg3200.status = I2CTransDone;
  }
  // Reset status if transaction fails
  if (imu_navgo_itg3200.status == I2CTransFailed) {
    imu_navgo_itg3200.status = I2CTransDone;
  }


  // If the adxl345 I2C transaction has succeeded: convert the data
  if (imu_navgo_adxl345.status == I2CTransSuccess)
  {
    x = (int16_t) ((imu_navgo_adxl345.buf[1] << 8) | imu_navgo_adxl345.buf[0]);
    y = (int16_t) ((imu_navgo_adxl345.buf[3] << 8) | imu_navgo_adxl345.buf[2]);
    z = (int16_t) ((imu_navgo_adxl345.buf[5] << 8) | imu_navgo_adxl345.buf[4]);

    VECT3_ASSIGN(imu.accel_unscaled,
        (/*IMU_ACCEL_X_SIGN * */y),
        (/*IMU_ACCEL_Y_SIGN * */-x),
        (/*IMU_ACCEL_Z_SIGN * */z));

    acc_valid = TRUE;
    imu_navgo_adxl345.status = I2CTransDone;
  }
  // Reset status if transaction fails
  if (imu_navgo_adxl345.status == I2CTransFailed) {
    imu_navgo_adxl345.status = I2CTransDone;
  }

  // HMC58XX event task
  hmc58xx_event();
  if (hmc58xx_data_available) {
    VECT3_COPY(imu.mag_unscaled, hmc58xx_data);
    hmc58xx_data_available = FALSE;
    mag_valid = TRUE;
  }

}


/*
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
#include "estimator.h"
#include "mcu_periph/i2c.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


// Peripherials
#define HMC5843_NO_IRQ
#include "../../peripherals/itg3200.h"
#include "../../peripherals/adxl345.h"
#include "../../peripherals/hmc5843.h"

// Results
int32_t mag_x, mag_y, mag_z;
volatile bool_t mag_valid;

int32_t gyr_x, gyr_y, gyr_z;
volatile bool_t gyr_valid;

int32_t acc_x, acc_y, acc_z;
volatile bool_t acc_valid;


// Communication
struct i2c_transaction ppzuavimu_hmc5843;
struct i2c_transaction ppzuavimu_itg3200;
struct i2c_transaction ppzuavimu_adxl345;


void ppzuavimu_module_init( void )
{
  /////////////////////////////////////////////////////////////////////
  // ITG3200
  ppzuavimu_itg3200.type = I2CTransTx;
  ppzuavimu_itg3200.slave_addr = ITG3200_ADDR;
  /* set gyro range to 2000deg/s and low pass at 20Hz (< 60Hz/2) internal sampling at 1kHz */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_DLPF_FS;
  ppzuavimu_itg3200.buf[1] = (0x03<<3) | (0x04<<0);
  ppzuavimu_itg3200.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /* set sample rate to 66Hz: so at 60Hz there is always a new sample ready and you loose little */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_SMPLRT_DIV;
  ppzuavimu_itg3200.buf[1] = 0x0E;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /* switch to gyroX clock */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_PWR_MGM;
  ppzuavimu_itg3200.buf[1] = 0x01;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /* no interrupts for now */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_INT_CFG;
  ppzuavimu_itg3200.buf[1] = 0x00;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // ADXL345

  /* set data rate to 100Hz */
  ppzuavimu_adxl345.slave_addr = ADXL345_ADDR;
  ppzuavimu_adxl345.type = I2CTransTx;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_BW_RATE;
  ppzuavimu_adxl345.buf[1] = 0x0a;  // normal power and 100Hz sampling, 50Hz BW
  ppzuavimu_adxl345.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_adxl345);
    while(ppzuavimu_adxl345.status == I2CTransPending);

  /* switch to measurement mode */
  ppzuavimu_adxl345.type = I2CTransTx;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_POWER_CTL;
  ppzuavimu_adxl345.buf[1] = 1<<3;
  ppzuavimu_adxl345.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_adxl345);
    while(ppzuavimu_adxl345.status == I2CTransPending);

  /* Set range to 4g */
  ppzuavimu_adxl345.type = I2CTransTx;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_DATA_FORMAT;
  ppzuavimu_adxl345.buf[1] = 1<<3 | 0<<2 | 0x01;  // bit 3 is full resolution bit, bit 2 is left justify bit 0,1 are range: 00=2g 01=4g 10=8g 11=16g
  ppzuavimu_adxl345.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_adxl345);
    while(ppzuavimu_adxl345.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // HMC5843
  ppzuavimu_hmc5843.slave_addr = HMC5843_ADDR;
  ppzuavimu_hmc5843.type = I2CTransTx;
  ppzuavimu_hmc5843.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
  ppzuavimu_hmc5843.buf[1] = 0x00 | (0x06 << 2);
  ppzuavimu_hmc5843.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_hmc5843);
    while(ppzuavimu_hmc5843.status == I2CTransPending);

  ppzuavimu_hmc5843.type = I2CTransTx;
  ppzuavimu_hmc5843.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
  ppzuavimu_hmc5843.buf[1] = 0x01<<5;
  ppzuavimu_hmc5843.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_hmc5843);
    while(ppzuavimu_hmc5843.status == I2CTransPending);

  ppzuavimu_hmc5843.type = I2CTransTx;
  ppzuavimu_hmc5843.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
  ppzuavimu_hmc5843.buf[1] = 0x00;
  ppzuavimu_hmc5843.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_hmc5843);
    while(ppzuavimu_hmc5843.status == I2CTransPending);

}

void ppzuavimu_module_periodic( void )
{
  // Start reading the latest gyroscope data
  ppzuavimu_itg3200.type = I2CTransTxRx;
  ppzuavimu_itg3200.len_r = 6;
  ppzuavimu_itg3200.len_w = 1;
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_GYRO_XOUT_H;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE, &ppzuavimu_itg3200);

  // Start reading the latest accelerometer data
  ppzuavimu_adxl345.type = I2CTransTxRx;
  ppzuavimu_adxl345.len_r = 6;
  ppzuavimu_adxl345.len_w = 1;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_DATA_X0;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE, &ppzuavimu_adxl345);

  // Start reading the latest magnetometer data
  ppzuavimu_hmc5843.type = I2CTransTxRx;
  ppzuavimu_hmc5843.len_r = 6;
  ppzuavimu_hmc5843.len_w = 1;
  ppzuavimu_hmc5843.buf[0] = HMC5843_REG_DATXM;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE, &ppzuavimu_hmc5843);

  RunOnceEvery(6,DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,&gyr_x,&gyr_y,&gyr_z));
  RunOnceEvery(6,DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,&acc_x,&acc_y,&acc_z));
  RunOnceEvery(6,DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,&mag_x,&mag_y,&mag_z));
}

void ppzuavimu_module_event( void )
{
  // If the itg3200 I2C transaction has succeeded: convert the data
  if (ppzuavimu_itg3200.status == I2CTransSuccess)
  {
    gyr_x = (int16_t) ((ppzuavimu_itg3200.buf[0] << 8) | ppzuavimu_itg3200.buf[1]);
    gyr_y = (int16_t) ((ppzuavimu_itg3200.buf[2] << 8) | ppzuavimu_itg3200.buf[3]);
    gyr_z = (int16_t) ((ppzuavimu_itg3200.buf[4] << 8) | ppzuavimu_itg3200.buf[5]);
    gyr_valid = TRUE;
  }

  // If the adxl345 I2C transaction has succeeded: convert the data
  if (ppzuavimu_adxl345.status == I2CTransSuccess)
  {
    acc_x = (int16_t) ((ppzuavimu_adxl345.buf[1] << 8) | ppzuavimu_adxl345.buf[0]);
    acc_y = (int16_t) ((ppzuavimu_adxl345.buf[3] << 8) | ppzuavimu_adxl345.buf[2]);
    acc_z = (int16_t) ((ppzuavimu_adxl345.buf[5] << 8) | ppzuavimu_adxl345.buf[4]);
    acc_valid = TRUE;
  }

  // If the hmc5843 I2C transaction has succeeded: convert the data
  if (ppzuavimu_hmc5843.status == I2CTransSuccess)
  {
    mag_x = (int16_t) ((ppzuavimu_hmc5843.buf[0] << 8) | ppzuavimu_hmc5843.buf[1]);
    mag_y = (int16_t) ((ppzuavimu_hmc5843.buf[2] << 8) | ppzuavimu_hmc5843.buf[3]);
    mag_z = (int16_t) ((ppzuavimu_hmc5843.buf[4] << 8) | ppzuavimu_hmc5843.buf[5]);
    mag_valid = TRUE;
  }
}

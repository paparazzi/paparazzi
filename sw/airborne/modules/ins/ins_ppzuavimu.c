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
#include "ins_ppzuavimu.h"
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
#define HMC5843_NO_IRQ
#include "../../peripherals/itg3200.h"
#include "../../peripherals/adxl345.h"
#include "../../peripherals/hmc5843.h"

// Results
volatile bool_t mag_valid;
volatile bool_t gyr_valid;
volatile bool_t acc_valid;

// Communication
struct i2c_transaction ppzuavimu_hmc5843;
struct i2c_transaction ppzuavimu_itg3200;
struct i2c_transaction ppzuavimu_adxl345;

// Standalone option: run module only
#ifndef IMU_TYPE_H
struct Imu imu;
#endif

#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 60
#endif

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // ITG3200
  ppzuavimu_itg3200.type = I2CTransTx;
  ppzuavimu_itg3200.slave_addr = ITG3200_ADDR;
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_DLPF_FS;
#if PERIODIC_FREQUENCY == 60
  /* set gyro range to 2000deg/s and low pass at 20Hz (< 60Hz/2) internal sampling at 1kHz */
  ppzuavimu_itg3200.buf[1] = (0x03<<3) | (0x04<<0);
#  warning ITG3200 read at 50Hz
#else
#  if PERIODIC_FREQUENCY == 120
#  warning ITG3200 read at 100Hz
  /* set gyro range to 2000deg/s and low pass at 42Hz (< 120Hz/2) internal sampling at 1kHz */
  ppzuavimu_itg3200.buf[1] = (0x03<<3) | (0x03<<0);
#  else
#  error PERIODIC_FREQUENCY should be either 60Hz or 120Hz. Otherwise manually fix the sensor rates
#  endif
#endif
  ppzuavimu_itg3200.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /* set sample rate to 66Hz: so at 60Hz there is always a new sample ready and you loose little */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_SMPLRT_DIV;
#if PERIODIC_FREQUENCY == 60
  ppzuavimu_itg3200.buf[1] = 19;  // 50Hz
#else
  ppzuavimu_itg3200.buf[1] = 9;  // 100Hz
#endif
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /* switch to gyroX clock */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_PWR_MGM;
  ppzuavimu_itg3200.buf[1] = 0x01;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /* no interrupts for now, but set data ready interrupt to enable reading status bits */
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_INT_CFG;
  ppzuavimu_itg3200.buf[1] = 0x01;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_itg3200);
    while(ppzuavimu_itg3200.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // ADXL345

  /* set data rate to 100Hz */
  ppzuavimu_adxl345.slave_addr = ADXL345_ADDR;
  ppzuavimu_adxl345.type = I2CTransTx;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_BW_RATE;
#if PERIODIC_FREQUENCY == 60
  ppzuavimu_adxl345.buf[1] = 0x09;  // normal power and 50Hz sampling, 50Hz BW
#else
  ppzuavimu_adxl345.buf[1] = 0x0a;  // normal power and 100Hz sampling, 50Hz BW
#endif
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

  /* Set range to 16g but keeping full resolution of 3.9 mV/g */
  ppzuavimu_adxl345.type = I2CTransTx;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_DATA_FORMAT;
  ppzuavimu_adxl345.buf[1] = 1<<3 | 0<<2 | 0x03;  // bit 3 is full resolution bit, bit 2 is left justify bit 0,1 are range: 00=2g 01=4g 10=8g 11=16g
  ppzuavimu_adxl345.len_w = 2;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE,&ppzuavimu_adxl345);
    while(ppzuavimu_adxl345.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // HMC5843
  ppzuavimu_hmc5843.slave_addr = HMC5843_ADDR;
  ppzuavimu_hmc5843.type = I2CTransTx;
  ppzuavimu_hmc5843.buf[0] = HMC5843_REG_CFGA;  // set to rate to max speed: 50Hz no bias
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

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  ppzuavimu_itg3200.type = I2CTransTxRx;
  ppzuavimu_itg3200.len_r = 9;
  ppzuavimu_itg3200.len_w = 1;
  ppzuavimu_itg3200.buf[0] = ITG3200_REG_INT_STATUS;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE, &ppzuavimu_itg3200);

  // Start reading the latest accelerometer data
  ppzuavimu_adxl345.type = I2CTransTxRx;
  ppzuavimu_adxl345.len_r = 6;
  ppzuavimu_adxl345.len_w = 1;
  ppzuavimu_adxl345.buf[0] = ADXL345_REG_DATA_X0;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE, &ppzuavimu_adxl345);

  // Start reading the latest magnetometer data
#if PERIODIC_FREQUENCY > 60
  RunOnceEvery(2,{
#endif
  ppzuavimu_hmc5843.type = I2CTransTxRx;
  ppzuavimu_hmc5843.len_r = 6;
  ppzuavimu_hmc5843.len_w = 1;
  ppzuavimu_hmc5843.buf[0] = HMC5843_REG_DATXM;
  i2c_submit(&PPZUAVIMU_I2C_DEVICE, &ppzuavimu_hmc5843);
#if PERIODIC_FREQUENCY > 60
  });
#endif
  //RunOnceEvery(10,ppzuavimu_module_downlink_raw());
}

void ppzuavimu_module_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,&imu.mag_unscaled.x,&imu.mag_unscaled.y,&imu.mag_unscaled.z);
}

void ppzuavimu_module_event( void )
{
  int32_t x, y, z;

  // If the itg3200 I2C transaction has succeeded: convert the data
  if (ppzuavimu_itg3200.status == I2CTransSuccess)
  {
#define ITG_STA_DAT_OFFSET 3
    x = (int16_t) ((ppzuavimu_itg3200.buf[0+ITG_STA_DAT_OFFSET] << 8) | ppzuavimu_itg3200.buf[1+ITG_STA_DAT_OFFSET]);
    y = (int16_t) ((ppzuavimu_itg3200.buf[2+ITG_STA_DAT_OFFSET] << 8) | ppzuavimu_itg3200.buf[3+ITG_STA_DAT_OFFSET]);
    z = (int16_t) ((ppzuavimu_itg3200.buf[4+ITG_STA_DAT_OFFSET] << 8) | ppzuavimu_itg3200.buf[5+ITG_STA_DAT_OFFSET]);

    // Is this is new data
    if (ppzuavimu_itg3200.buf[0] & 0x01)
    {
      //LED_ON(3);
      gyr_valid = TRUE;
      //LED_OFF(3);
    }
    else
    {
    }

    // Signs depend on the way sensors are soldered on the board: so they are hardcoded
#ifdef ASPIRIN_IMU
    RATES_ASSIGN(imu.gyro_unscaled, x, -y, -z);
#else // PPZIMU
    RATES_ASSIGN(imu.gyro_unscaled, -x, y, -z);
#endif

    ppzuavimu_itg3200.status = I2CTransDone;  // remove the I2CTransSuccess status, otherwise data ready will be triggered again without new data
  }

  // If the adxl345 I2C transaction has succeeded: convert the data
  if (ppzuavimu_adxl345.status == I2CTransSuccess)
  {
    x = (int16_t) ((ppzuavimu_adxl345.buf[1] << 8) | ppzuavimu_adxl345.buf[0]);
    y = (int16_t) ((ppzuavimu_adxl345.buf[3] << 8) | ppzuavimu_adxl345.buf[2]);
    z = (int16_t) ((ppzuavimu_adxl345.buf[5] << 8) | ppzuavimu_adxl345.buf[4]);

#ifdef ASPIRIN_IMU
    VECT3_ASSIGN(imu.accel_unscaled, x, -y, -z);
#else // PPZIMU
    VECT3_ASSIGN(imu.accel_unscaled, -x, y, -z);
#endif

    acc_valid = TRUE;
    ppzuavimu_adxl345.status = I2CTransDone;
  }

  // If the hmc5843 I2C transaction has succeeded: convert the data
  if (ppzuavimu_hmc5843.status == I2CTransSuccess)
  {
    x = (int16_t) ((ppzuavimu_hmc5843.buf[0] << 8) | ppzuavimu_hmc5843.buf[1]);
    y = (int16_t) ((ppzuavimu_hmc5843.buf[2] << 8) | ppzuavimu_hmc5843.buf[3]);
    z = (int16_t) ((ppzuavimu_hmc5843.buf[4] << 8) | ppzuavimu_hmc5843.buf[5]);

#ifdef ASPIRIN_IMU
    VECT3_ASSIGN(imu.mag_unscaled, x, -y, -z);
#else // PPZIMU
    VECT3_ASSIGN(imu.mag_unscaled, -y, -x, -z);
#endif

    mag_valid = TRUE;
    ppzuavimu_hmc5843.status = I2CTransDone;
  }
}

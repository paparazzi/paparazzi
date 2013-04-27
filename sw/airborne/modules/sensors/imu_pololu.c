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
#include "imu_pololu.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Set SPI_CS High
#include "mcu_periph/gpio_arch.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


// Peripherials
#define HMC5843_NO_IRQ
#include "../../peripherals/l3g4200.h"
#include "../../peripherals/lsm303dl.h"
#include "../../peripherals/hmc5843.h"

// Results
volatile bool_t mag_valid;
volatile bool_t gyr_valid;
volatile bool_t acc_valid;

// Communication
struct i2c_transaction pololuimu_hmc5843;
struct i2c_transaction pololuimu_l3g4200;
struct i2c_transaction pololuimu_lsm303dl;

// Standalone option: run module only
#ifndef IMU_TYPE_H
struct Imu imu;
#endif

#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 60
#endif

void imu_impl_init(void)
{
  GPIO_ARCH_SET_SPI_CS_HIGH();

  /////////////////////////////////////////////////////////////////////
  // L3G4200
  pololuimu_l3g4200.slave_addr = L3G4200_ADDR;
  pololuimu_l3g4200.type = I2CTransTx;
//  pololuimu_l3g4200.buf[0] = L3G4200_REG_DLPF_FS;
#if PERIODIC_FREQUENCY == 60
  /* set gyro range to 2000deg/s and low pass at 20Hz (< 60Hz/2) internal sampling at 1kHz */
//  pololuimu_l3g4200.buf[1] = (0x03<<3) | (0x04<<0);
#  warning Info: L3G4200 read at 50Hz
#else
#  if PERIODIC_FREQUENCY == 120
#  warning Info: L3G4200 read at 100Hz
  /* set gyro range to 2000deg/s and low pass at 42Hz (< 120Hz/2) internal sampling at 1kHz */
//  pololuimu_l3g4200.buf[1] = (0x03<<3) | (0x03<<0);
#  else
#  error PERIODIC_FREQUENCY should be either 60Hz or 120Hz. Otherwise manually fix the sensor rates
#  endif
#endif
//  pololuimu_l3g4200.len_w = 2;
//  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_l3g4200);
//    while(pololuimu_l3g4200.status == I2CTransPending);

  /* set sample rate to 66Hz: so at 60Hz there is always a new sample ready and you loose little */
//  pololuimu_l3g4200.buf[0] = L3G4200_REG_SMPLRT_DIV;
#if PERIODIC_FREQUENCY == 60
//  pololuimu_l3g4200.buf[1] = 19;  // 50Hz
#else
//  pololuimu_l3g4200.buf[1] = 9;  // 100Hz
#endif
//  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_l3g4200);
//    while(pololuimu_l3g4200.status == I2CTransPending);

  /* switch to gyroX clock */
  pololuimu_l3g4200.type = I2CTransTx;
  pololuimu_l3g4200.buf[0] = 0x20;
  pololuimu_l3g4200.buf[1] = 0x8f;
  pololuimu_l3g4200.len_w = 2;  
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_l3g4200);
    while(pololuimu_l3g4200.status == I2CTransPending);

  pololuimu_l3g4200.type = I2CTransTx;
  pololuimu_l3g4200.buf[0] = 0x24;
  pololuimu_l3g4200.buf[1] = 0x02;
  pololuimu_l3g4200.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_l3g4200);
    while(pololuimu_l3g4200.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // LSM303DL

  pololuimu_lsm303dl.slave_addr = LSM303DL_ADDR;
  /* set ????? to ???? */
//  pololuimu_lsm303dl.type = I2CTransTx;
//  pololuimu_lsm303dl.buf[0] = ADXL345_REG_BW_RATE;
#if PERIODIC_FREQUENCY == 60
//  pololuimu_lsm303dl.buf[1] = 0x09;  // normal power and 50Hz sampling, 50Hz BW
#else
//  pololuimu_lsm303dl.buf[1] = 0x0a;  // normal power and 100Hz sampling, 50Hz BW
#endif
//  pololuimu_lsm303dl.len_w = 2;
//  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_lsm303dl);
//    while(pololuimu_lsm303dl.status == I2CTransPending);

#if LSM303_ACC_RATE
#  warning Info: LSM303 ACC RATE NOT DEFAULT
#endif


  /* switch to measurement mode */
  pololuimu_lsm303dl.type = I2CTransTx;
  pololuimu_lsm303dl.buf[0] = LSM303DL_REG_CTRL_REG1_A;
  pololuimu_lsm303dl.buf[1] = 0x27;
  pololuimu_lsm303dl.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_lsm303dl);
    while(pololuimu_lsm303dl.status == I2CTransPending);

  /* switch to measurement mode */
  pololuimu_lsm303dl.type = I2CTransTx;
  pololuimu_lsm303dl.buf[0] = LSM303DL_REG_CTRL_REG4_A;
  pololuimu_lsm303dl.buf[1] = 0x30;
  pololuimu_lsm303dl.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_lsm303dl);
    while(pololuimu_lsm303dl.status == I2CTransPending);

  /* switch to measurement mode */
  pololuimu_lsm303dl.type = I2CTransTx;
  pololuimu_lsm303dl.buf[0] = LSM303DL_REG_CTRL_REG2_A;
  pololuimu_lsm303dl.buf[1] = 0x00;
  pololuimu_lsm303dl.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_lsm303dl);
    while(pololuimu_lsm303dl.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // HMC5843
  pololuimu_hmc5843.slave_addr = HMC5843_ADDR;

  pololuimu_hmc5843.type = I2CTransTx;
  pololuimu_hmc5843.buf[0] = HMC5843_REG_CFGA;  // set to rate to max speed: 50Hz no bias
  pololuimu_hmc5843.buf[1] = 0x00 | (0x06 << 2);
  pololuimu_hmc5843.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_hmc5843);
    while(pololuimu_hmc5843.status == I2CTransPending);

  pololuimu_hmc5843.type = I2CTransTx;
  pololuimu_hmc5843.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
  pololuimu_hmc5843.buf[1] = 0x01<<5;
  pololuimu_hmc5843.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_hmc5843);
    while(pololuimu_hmc5843.status == I2CTransPending);

  pololuimu_hmc5843.type = I2CTransTx;
  pololuimu_hmc5843.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
  pololuimu_hmc5843.buf[1] = 0x00;
  pololuimu_hmc5843.len_w = 2;
  i2c_submit(&POLOLUIMU_I2C_DEVICE,&pololuimu_hmc5843);
    while(pololuimu_hmc5843.status == I2CTransPending);

}

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  pololuimu_l3g4200.type = I2CTransTxRx;
  pololuimu_l3g4200.len_r = 8;
  pololuimu_l3g4200.len_w = 1;
  pololuimu_l3g4200.buf[0] = 0x26 | 0x80;
  i2c_submit(&POLOLUIMU_I2C_DEVICE, &pololuimu_l3g4200);

  // Start reading the latest accelerometer data
  pololuimu_lsm303dl.type = I2CTransTxRx;
  pololuimu_lsm303dl.len_r = 6;
  pololuimu_lsm303dl.len_w = 1;
  pololuimu_lsm303dl.buf[0] = 0x28 | 0x80;
  i2c_submit(&POLOLUIMU_I2C_DEVICE, &pololuimu_lsm303dl);

  // Start reading the latest magnetometer data
#if PERIODIC_FREQUENCY > 60
  RunOnceEvery(2,{
#endif
  pololuimu_hmc5843.type = I2CTransTxRx;
  pololuimu_hmc5843.len_r = 6;
  pololuimu_hmc5843.len_w = 1;
  pololuimu_hmc5843.buf[0] = HMC5843_REG_DATXM;
  i2c_submit(&POLOLUIMU_I2C_DEVICE, &pololuimu_hmc5843);
#if PERIODIC_FREQUENCY > 60
  });
#endif
  //RunOnceEvery(10,ppzuavimu_module_downlink_raw());
}

void pololuimu_module_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,DefaultDevice,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,DefaultDevice,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,DefaultDevice,&imu.mag_unscaled.x,&imu.mag_unscaled.y,&imu.mag_unscaled.z);
}

void pololuimu_module_event( void )
{
  int32_t x, y, z;

  // If the ltg4200 I2C transaction has succeeded: convert the data
  if (pololuimu_l3g4200.status == I2CTransSuccess)
  {
#define offset_data 2
    x = (int16_t) ((pololuimu_l3g4200.buf[1+offset_data] << 8) | pololuimu_l3g4200.buf[0+offset_data]);
    y = (int16_t) ((pololuimu_l3g4200.buf[3+offset_data] << 8) | pololuimu_l3g4200.buf[2+offset_data]);
    z = (int16_t) ((pololuimu_l3g4200.buf[5+offset_data] << 8) | pololuimu_l3g4200.buf[4+offset_data]);

    // Is this is new data
    if (pololuimu_l3g4200.buf[1] & (1<<3)) //ZYX new data
	{
        gyr_valid = TRUE;
	}
    else
	{
	}

    // Signs depend on the way sensors are soldered on the board: so they are hardcoded
    RATES_ASSIGN(imu.gyro_unscaled, -x, y, -z);
    pololuimu_l3g4200.status = I2CTransDone;  // remove the I2CTransSuccess status, otherwise data ready will be triggered again without new data
  }

  // If the lsm303dl I2C transaction has succeeded: convert the data
  if (pololuimu_lsm303dl.status == I2CTransSuccess)
  {
    x = (int16_t) ((pololuimu_lsm303dl.buf[1] << 8) | pololuimu_lsm303dl.buf[0]);
    y = (int16_t) ((pololuimu_lsm303dl.buf[3] << 8) | pololuimu_lsm303dl.buf[2]);
    z = (int16_t) ((pololuimu_lsm303dl.buf[5] << 8) | pololuimu_lsm303dl.buf[4]);


    VECT3_ASSIGN(imu.accel_unscaled, -x, y, -z);

    acc_valid = TRUE;

    pololuimu_lsm303dl.status = I2CTransDone;
  }

  // If the hmc5843 I2C transaction has succeeded: convert the data
  if (pololuimu_hmc5843.status == I2CTransSuccess)
  {
    x = (int16_t) ((pololuimu_hmc5843.buf[0] << 8) | pololuimu_hmc5843.buf[1]);
    y = (int16_t) ((pololuimu_hmc5843.buf[2] << 8) | pololuimu_hmc5843.buf[3]);
    z = (int16_t) ((pololuimu_hmc5843.buf[4] << 8) | pololuimu_hmc5843.buf[5]);

    VECT3_ASSIGN(imu.mag_unscaled, -x, -z, -y);

    mag_valid = TRUE;
    pololuimu_hmc5843.status = I2CTransDone;
  }
}

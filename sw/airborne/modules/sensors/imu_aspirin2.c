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
#include "imu_aspirin2.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "subsystems/abi.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"



// Peripherials
#include "../../peripherals/mpu60x0.h"
// #include "../../peripherals/hmc5843.h"

// Communication
struct i2c_transaction aspirin2_mpu60x0;

// Standalone option: run module only
#ifndef IMU_TYPE_H
struct Imu imu;
#endif

#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 60
#endif

void imu_impl_init(void)
{

  ///////////////////
  // Configure power:

  // MPU60X0_REG_AUX_VDDIO = 0 (good on startup)

  // MPU60X0_REG_USER_CTRL:
  // -Enable Aux I2C Master Mode
  // -Enable SPI

  // MPU60X0_REG_PWR_MGMT_1
  // -switch to gyroX clock
  aspirin2_mpu60x0.buf[0] = MPU60X0_REG_PWR_MGMT_1;
  aspirin2_mpu60x0.buf[1] = 0x01;
  i2c_transmit(&PPZUAVIMU_I2C_DEV, &aspirin2_mpu60x0, MPU60X0_ADDR, 2);
  while (aspirin2_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_PWR_MGMT_2: Nothing should be in standby: default OK

  /////////////////////////
  // Measurement Settings

  // MPU60X0_REG_CONFIG
  // -ext sync on gyro X (bit 3->6)
  // -digital low pass filter: 1kHz sampling of gyro/acc with 44Hz bandwidth: since reading is at 100Hz
#if PERIODIC_FREQUENCY == 60
#else
#  if PERIODIC_FREQUENCY == 120
#  else
#  error PERIODIC_FREQUENCY should be either 60Hz or 120Hz. Otherwise manually fix the sensor rates
#  endif
#endif
  aspirin2_mpu60x0.buf[0] = MPU60X0_REG_CONFIG;
  aspirin2_mpu60x0.buf[1] = (2 << 3) | (3 << 0);
  i2c_transmit(&PPZUAVIMU_I2C_DEV, &aspirin2_mpu60x0, MPU60X0_ADDR, 2);
  while (aspirin2_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_SMPLRT_DIV
  // -100Hz output = 1kHz / (9 + 1)
  aspirin2_mpu60x0.buf[0] = MPU60X0_REG_SMPLRT_DIV;
  aspirin2_mpu60x0.buf[1] = 9;
  i2c_transmit(&PPZUAVIMU_I2C_DEV, &aspirin2_mpu60x0, MPU60X0_ADDR, 2);
  while (aspirin2_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_GYRO_CONFIG
  // -2000deg/sec
  aspirin2_mpu60x0.buf[0] = MPU60X0_REG_GYRO_CONFIG;
  aspirin2_mpu60x0.buf[1] = (3 << 3);
  i2c_transmit(&PPZUAVIMU_I2C_DEV, &aspirin2_mpu60x0, MPU60X0_ADDR, 2);
  while (aspirin2_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_ACCEL_CONFIG
  // 16g, no HPFL
  aspirin2_mpu60x0.buf[0] = MPU60X0_REG_ACCEL_CONFIG;
  aspirin2_mpu60x0.buf[1] = (3 << 3);
  i2c_transmit(&PPZUAVIMU_I2C_DEV, &aspirin2_mpu60x0, MPU60X0_ADDR, 2);
  while (aspirin2_mpu60x0.status == I2CTransPending);





  /*
    // no interrupts for now, but set data ready interrupt to enable reading status bits
    aspirin2_mpu60x0.buf[0] = ITG3200_REG_INT_CFG;
    aspirin2_mpu60x0.buf[1] = 0x01;
    i2c_submit(&PPZUAVIMU_I2C_DEV,&aspirin2_mpu60x0);
      while(aspirin2_mpu60x0.status == I2CTransPending);
  */

  /*
    /////////////////////////////////////////////////////////////////////
    // HMC5843
    ppzuavimu_hmc5843.slave_addr = HMC5843_ADDR;
    ppzuavimu_hmc5843.type = I2CTransTx;
    ppzuavimu_hmc5843.buf[0] = HMC5843_REG_CFGA;  // set to rate to max speed: 50Hz no bias
    ppzuavimu_hmc5843.buf[1] = 0x00 | (0x06 << 2);
    ppzuavimu_hmc5843.len_w = 2;
    i2c_submit(&PPZUAVIMU_I2C_DEV,&ppzuavimu_hmc5843);
      while(ppzuavimu_hmc5843.status == I2CTransPending);

    ppzuavimu_hmc5843.type = I2CTransTx;
    ppzuavimu_hmc5843.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    ppzuavimu_hmc5843.buf[1] = 0x01<<5;
    ppzuavimu_hmc5843.len_w = 2;
    i2c_submit(&PPZUAVIMU_I2C_DEV,&ppzuavimu_hmc5843);
      while(ppzuavimu_hmc5843.status == I2CTransPending);

    ppzuavimu_hmc5843.type = I2CTransTx;
    ppzuavimu_hmc5843.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
    ppzuavimu_hmc5843.buf[1] = 0x00;
    ppzuavimu_hmc5843.len_w = 2;
    i2c_submit(&PPZUAVIMU_I2C_DEV,&ppzuavimu_hmc5843);
      while(ppzuavimu_hmc5843.status == I2CTransPending);
  */
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  aspirin2_mpu60x0.buf[0] = MPU60X0_REG_INT_STATUS;
  i2c_transceive(&PPZUAVIMU_I2C_DEV, &aspirin2_mpu60x0, MPU60X0_ADDR, 1, 21);

  /*
    // Start reading the latest accelerometer data
    ppzuavimu_adxl345.buf[0] = ADXL345_REG_DATA_X0;
    i2c_transceive(&PPZUAVIMU_I2C_DEV, &ppzuavimu_adxl345, ADXL345_ADDR, 1, 6);
  */
  // Start reading the latest magnetometer data
#if PERIODIC_FREQUENCY > 60
  RunOnceEvery(2, {
#endif
    /*  ppzuavimu_hmc5843.type = I2CTransTxRx;
      ppzuavimu_hmc5843.len_r = 6;
      ppzuavimu_hmc5843.len_w = 1;
      ppzuavimu_hmc5843.buf[0] = HMC5843_REG_DATXM;
      i2c_submit(&PPZUAVIMU_I2C_DEV, &ppzuavimu_hmc5843);
    */
#if PERIODIC_FREQUENCY > 60
  });
#endif
  //RunOnceEvery(10,aspirin2_subsystem_downlink_raw());
}

void aspirin2_subsystem_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z);
}

void aspirin2_subsystem_event(void)
{
  int32_t x, y, z;
  uint32_t now_ts = get_sys_time_usec();

  // If the itg3200 I2C transaction has succeeded: convert the data
  if (aspirin2_mpu60x0.status == I2CTransSuccess) {
#define MPU_OFFSET_GYRO 9
    x = (int16_t)((aspirin2_mpu60x0.buf[0 + MPU_OFFSET_GYRO] << 8) | aspirin2_mpu60x0.buf[1 + MPU_OFFSET_GYRO]);
    y = (int16_t)((aspirin2_mpu60x0.buf[2 + MPU_OFFSET_GYRO] << 8) | aspirin2_mpu60x0.buf[3 + MPU_OFFSET_GYRO]);
    z = (int16_t)((aspirin2_mpu60x0.buf[4 + MPU_OFFSET_GYRO] << 8) | aspirin2_mpu60x0.buf[5 + MPU_OFFSET_GYRO]);

    RATES_ASSIGN(imu.gyro_unscaled, x, y, z);

#define MPU_OFFSET_ACC 1
    x = (int16_t)((aspirin2_mpu60x0.buf[0 + MPU_OFFSET_ACC] << 8) | aspirin2_mpu60x0.buf[1 + MPU_OFFSET_ACC]);
    y = (int16_t)((aspirin2_mpu60x0.buf[2 + MPU_OFFSET_ACC] << 8) | aspirin2_mpu60x0.buf[3 + MPU_OFFSET_ACC]);
    z = (int16_t)((aspirin2_mpu60x0.buf[4 + MPU_OFFSET_ACC] << 8) | aspirin2_mpu60x0.buf[5 + MPU_OFFSET_ACC]);

    VECT3_ASSIGN(imu.accel_unscaled, x, y, z);

    // Is this is new data
    if (aspirin2_mpu60x0.buf[0] & 0x01) {
      imu_scale_gyro(&imu);
      imu_scale_accel(&imu);
      AbiSendMsgIMU_GYRO_INT32(IMU_PPZUAV_ID, now_ts, &imu.gyro);
      AbiSendMsgIMU_ACCEL_INT32(IMU_PPZUAV_ID, now_ts, &imu.accel);
    }

    aspirin2_mpu60x0.status =
      I2CTransDone;  // remove the I2CTransSuccess status, otherwise data ready will be triggered again without new data
  }
  /*
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
  */
}


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
#include "imu_drotec.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


// Peripherials
#include "../../peripherals/mpu60X0.h"
#include "../../peripherals/hmc58xx.h"

// Results
volatile bool_t mag_valid;
volatile bool_t gyr_valid;
volatile bool_t acc_valid;

// Communication
struct i2c_transaction drotec_mpu60x0;

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
  // MPU60X0
  drotec_mpu60x0.type = I2CTransTx;
  drotec_mpu60x0.slave_addr = MPU60X0_ADDR_ALT;
  drotec_mpu60x0.len_r = 0;
  drotec_mpu60x0.len_w = 2;


  ///////////////////
  // Configure power:

  // MPU60X0_REG_AUX_VDDIO = 0 (good on startup)

  // MPU60X0_REG_USER_CTRL:
  // -Enable Aux I2C Master Mode
  // -Enable SPI

  // MPU60X0_REG_PWR_MGMT_1
  // -switch to gyroX clock
  drotec_mpu60x0.buf[0] = MPU60X0_REG_PWR_MGMT_1;
  drotec_mpu60x0.buf[1] = 0x01;
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

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
  drotec_mpu60x0.buf[0] = MPU60X0_REG_CONFIG;
  drotec_mpu60x0.buf[1] = (2 << 3) | (3 << 0);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_SMPLRT_DIV
  // -100Hz output = 1kHz / (9 + 1)
  drotec_mpu60x0.buf[0] = MPU60X0_REG_SMPLRT_DIV;
  drotec_mpu60x0.buf[1] = 9;
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_GYRO_CONFIG
  // -2000deg/sec
  drotec_mpu60x0.buf[0] = MPU60X0_REG_GYRO_CONFIG;
  drotec_mpu60x0.buf[1] = (3<<3);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

  // MPU60X0_REG_ACCEL_CONFIG
  // 16g, no HPFL
  drotec_mpu60x0.buf[0] = MPU60X0_REG_ACCEL_CONFIG;
  drotec_mpu60x0.buf[1] = (3<<3);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);


#ifdef READ_MAG
// i2c Slave Configuration Section

// Power the Aux I2C Circuit:
// drotec_mpu60x0.buf[0] = MPU60X0_REG_AUX_VDDIO;
// drotec_mpu60x0.buf[1] = (0 << 7); //(0 << 7); // 0 (good on startup):  (0 << 7);	// MPU6000: 0=Vdd. MPU6050 : 0=VLogic 1=Vdd
//  i2c_submit(&DROTEC_I2C_DEVICE,&drotec_mpu60x0);
//  while(drotec_mpu60x0.status == I2CTransPending);
 
  // MPU60X0_REG_USER_CTRL:
  drotec_mpu60x0.buf[0] = MPU60X0_REG_USER_CTRL;
  drotec_mpu60x0.buf[1] = ((1 << 5) |		// I2C_MST_EN: Enable Aux I2C Master Mode
				(0 << 1) );		// Trigger a I2C_MST_RESET
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

  // Enable the aux i2c
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_MST_CTRL;
  drotec_mpu60x0.buf[1] =	((0 << 7) | 		// no multimaster
				(0 << 6) |		// do not delay IRQ waiting for all external slaves
				(0 << 5) | 		// no slave 3 FIFO
				(0 << 4) | 		// restart or stop/start from one slave to another: read -> write is always stop/start
				(8 << 0) );		// 0=348kHz 8=256kHz, 9=500kHz
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

    
  // HMC5883 Magnetometer Configuration
  //step1
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_ADDR;
  drotec_mpu60x0.buf[1] = (HMC58XX_ADDR >> 1);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_REG;
  drotec_mpu60x0.buf[1] = (HMC58XX_REG_CFGA);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_DO;
  drotec_mpu60x0.buf[1] = (HMC58XX_CRA);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	
  
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_CTRL;
  drotec_mpu60x0.buf[1] =	((1 << 7) |		// Slave 4 enable
				(0 << 6) |		// Byte Swap
				(0 << 0) );		// Full Speed
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

//step2
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_ADDR;
  drotec_mpu60x0.buf[1] = (HMC58XX_ADDR >> 1);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_REG;
  drotec_mpu60x0.buf[1] = (HMC58XX_REG_CFGB);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_DO;
  drotec_mpu60x0.buf[1] = (HMC58XX_CRB);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_CTRL;
  drotec_mpu60x0.buf[1] =	((1 << 7) |		// Slave 4 enable
				(0 << 6) |		// Byte Swap
				(0 << 0) );		// Full Speed
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

//step3
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_ADDR;
  drotec_mpu60x0.buf[1] = (HMC58XX_ADDR >> 1);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_REG;
  drotec_mpu60x0.buf[1] = (HMC58XX_REG_MODE);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_DO;
  drotec_mpu60x0.buf[1] = (HMC58XX_MD);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	

  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV4_CTRL;
  drotec_mpu60x0.buf[1] =	((1 << 7) |		// Slave 4 enable
				(0 << 6) |		// Byte Swap
				(0 << 0) );		// Full Speed
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);

  // HMC5883 Reading:
  // a) write hmc-register to HMC
  // b) read 6 bytes from HMC

  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV0_ADDR;
  drotec_mpu60x0.buf[1] = ((HMC58XX_ADDR >> 1) | MPU60X0_SPI_READ);
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	
	
  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV0_REG;
  drotec_mpu60x0.buf[1] = HMC58XX_REG_DATXM;
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);	

  drotec_mpu60x0.buf[0] = MPU60X0_REG_I2C_SLV0_CTRL;
  drotec_mpu60x0.buf[1] =	((1 << 7) |		// Slave 0 enable
				(0 << 6) |		// Byte Swap
				(6 << 0) );		// Read 6 bytes
  i2c_submit(&DROTECIMU_I2C_DEVICE,&drotec_mpu60x0);
    while(drotec_mpu60x0.status == I2CTransPending);
  // end conf mpu60x0 slave i2c magnetometer
#endif // READ_MAG


}

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  drotec_mpu60x0.type = I2CTransTxRx;
  drotec_mpu60x0.len_r = 21;
  drotec_mpu60x0.len_w = 1;
  drotec_mpu60x0.buf[0] = MPU60X0_REG_INT_STATUS;
  i2c_submit(&DROTECIMU_I2C_DEVICE, &drotec_mpu60x0);
}

void drotec_subsystem_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,DefaultDevice,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,DefaultDevice,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
#ifdef READ_MAG
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,DefaultDevice,&imu.mag_unscaled.x,&imu.mag_unscaled.y,&imu.mag_unscaled.z);
#endif
}

void drotec_subsystem_event( void )
{
  int32_t x, y, z;

  // If the itg3200 I2C transaction has succeeded: convert the data
  if (drotec_mpu60x0.status == I2CTransSuccess)
  {
#define MPU_OFFSET_GYRO 9
    x = (int16_t) ((drotec_mpu60x0.buf[0+MPU_OFFSET_GYRO] << 8) | drotec_mpu60x0.buf[1+MPU_OFFSET_GYRO]);
    y = (int16_t) ((drotec_mpu60x0.buf[2+MPU_OFFSET_GYRO] << 8) | drotec_mpu60x0.buf[3+MPU_OFFSET_GYRO]);
    z = (int16_t) ((drotec_mpu60x0.buf[4+MPU_OFFSET_GYRO] << 8) | drotec_mpu60x0.buf[5+MPU_OFFSET_GYRO]);

    RATES_ASSIGN(imu.gyro_unscaled, x, y, z);

#define MPU_OFFSET_ACC 1
    x = (int16_t) ((drotec_mpu60x0.buf[0+MPU_OFFSET_ACC] << 8) | drotec_mpu60x0.buf[1+MPU_OFFSET_ACC]);
    y = (int16_t) ((drotec_mpu60x0.buf[2+MPU_OFFSET_ACC] << 8) | drotec_mpu60x0.buf[3+MPU_OFFSET_ACC]);
    z = (int16_t) ((drotec_mpu60x0.buf[4+MPU_OFFSET_ACC] << 8) | drotec_mpu60x0.buf[5+MPU_OFFSET_ACC]);

    VECT3_ASSIGN(imu.accel_unscaled, x, y, z);

#ifdef READ_MAG
#define MPU_OFFSET_MAG 15
    x = (int16_t) ((drotec_mpu60x0.buf[0+MPU_OFFSET_MAG] << 8) | drotec_mpu60x0.buf[1+MPU_OFFSET_MAG]);
    y = (int16_t) ((drotec_mpu60x0.buf[2+MPU_OFFSET_MAG] << 8) | drotec_mpu60x0.buf[3+MPU_OFFSET_MAG]);
    z = (int16_t) ((drotec_mpu60x0.buf[4+MPU_OFFSET_MAG] << 8) | drotec_mpu60x0.buf[5+MPU_OFFSET_MAG]);
	
    VECT3_ASSIGN(imu.mag_unscaled, x, -y, -z);
#endif

    // Is this is new data
    if (drotec_mpu60x0.buf[0] & 0x01)
    {
      gyr_valid = TRUE;
      acc_valid = TRUE;
#ifdef READ_MAG
      mag_valid = TRUE;
#endif
    }
    else
    {
    }

    drotec_mpu60x0.status = I2CTransDone;  // remove the I2CTransSuccess status, otherwise data ready will be triggered again without new data
  }

}


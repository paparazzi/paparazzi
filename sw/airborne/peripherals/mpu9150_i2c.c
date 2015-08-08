/*
 * Copyright (C) 2014 Xavier Paris
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
 * @file peripherals/mpu9150_i2c.c
 *
 * Driver for the MPU-9150 using I2C.
 *
 */

#include "peripherals/mpu9150_i2c.h"

#ifdef MPU9150_SLV_MAG
bool_t mpu9150_i2c_configure_mag_slave(Mpu60x0ConfigSet mpu_set __attribute__ ((unused)), void* mpu __attribute__ ((unused))) {

  struct Mpu60x0_I2c* mpu_i2c = (struct Mpu60x0_I2c*)(mpu);

  switch (mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status) {

    // Precondition:
    // MPU I2C as been set to passthrough mode to configure AK8975 

    case 0:
      // Stay in this state (passthrough) until mag is configured
      if(ak8975_mpu_configure(mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_privateData))
        (mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status)++;
      break;

    case 1:
      // Disable MPU I2C passthrough to proceed MPU slave configuration for AK8975
      mpu_i2c->i2c_trans.slave_addr = MPU60X0_ADDR_ALT;
      mpu_set(mpu, MPU60X0_REG_INT_PIN_CFG, (0<<1));
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;

    // Configure I2C SLAVE 0 to read AK8975 measurements
    case 2:
      // Set I2C address Slave 0 at AK8975 address
      mpu_set(mpu, MPU60X0_REG_I2C_SLV0_ADDR, (0x1<<7)|AK8975_I2C_SLV_ADDR);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;
    case 3:
      // Register address to read from slave : (status 1 + measurement + status 2)
      mpu_set(mpu, MPU60X0_REG_I2C_SLV0_REG, AK8975_REG_ST1_ADDR);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;
    case 4:
      // Read 8 byte and enable this slave transaction
      mpu_set(mpu, MPU60X0_REG_I2C_SLV0_CTRL, (0x1<<7) | 8);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;

    // Configure I2C SLAVE 1 for AK8975 single mesure request
    case 5:
      // Set I2C address Slave 1 at AK8975 address
      mpu_set(mpu, MPU60X0_REG_I2C_SLV1_ADDR, (0x0<<7)|AK8975_I2C_SLV_ADDR);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;
    case 6:
      // Register address to write on slave : AK8975 read/write
      mpu_set(mpu, MPU60X0_REG_I2C_SLV1_REG, AK8975_REG_CNTL_ADDR);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;
    case 7:
      // Write 1 byte and enable this slave transaction
      mpu_set(mpu, MPU60X0_REG_I2C_SLV1_CTRL, (0x1<<7)|0x01);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;

    case 8:
      // Output data for Slave 1 is fixed, single mesure mode
      mpu_set(mpu, MPU60X0_REG_I2C_SLV1_DO, AK8975_MODE_SINGLE_MEAS);
      mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_init_status++;
      break;

    case 9:
      return TRUE;
    default:
      break;
  }

  return FALSE;
}
#endif


#ifdef MPU9150_SLV_BARO
bool_t mpu9150_i2c_configure_baro_slave(Mpu60x0ConfigSet mpu_set __attribute__ ((unused)), void* mpu __attribute__ ((unused))) {

  struct Mpu60x0_I2c* mpu_i2c = (struct Mpu60x0_I2c*)(mpu);

  struct Mpl3115 *mpl = mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_privateData;

  switch (mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status) {

    case 0:
      // switch to I2C passthrough 
      mpu_set(mpu, MPU60X0_REG_INT_PIN_CFG, (1<<1));
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;

    case 1:
      // Stay in this state (passthrough) until baro is configured
      if(mpl3115_mpu_configure(mpu_i2c->config.slaves[MPU_MAG_SLV_NB].mpu_slave_privateData))
        mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;

    // Configure I2C SLAVE 2 for MPL3115 measure
    case 2:
      mpu_set(mpu, MPU60X0_REG_I2C_SLV2_ADDR, (0x0<<7)|0x60); // MPL3115_I2C_ADDR
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;
    case 3:
      // Register address to write on slave : MPL3115 read/write
      mpu_set(mpu, MPU60X0_REG_I2C_SLV2_REG, MPL3115_REG_CTRL_REG1);
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;
    case 4:
      // Write 1 byte and enable this slave transaction
      mpu_set(mpu, MPU60X0_REG_I2C_SLV2_CTRL, (0x1<<7)|0x01);
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;
    case 5:
      // MPL3115 continuousMode mode to write on slave 
//      #define  OVERSAMPLING (0x0 << 3) // no oversampling conv in 6 ms
//      mpu_set(mpu, MPU60X0_REG_I2C_SLV0_DO, 0x2 | OVERSAMPLING);


      mpu_set(mpu, MPU60X0_REG_I2C_SLV2_DO,
                                ((MPL3115_OVERSAMPLING<<3) | (mpl->raw_mode<<6) |
                                (mpl->alt_mode<<7) | MPL3115_OST_BIT));

      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;

    // Configure I2C SLAVE 3 to read MPL3115 measurements
    case 6:
      mpu_set(mpu, MPU60X0_REG_I2C_SLV3_ADDR, (0x1<<7)|0x60); // MPL3115_I2C_ADDR
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;
    case 7:
      // Register address to read from slave : pressure 
      mpu_set(mpu, MPU60X0_REG_I2C_SLV3_REG, MPL3115_REG_OUT_P_MSB);
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;
    case 8:
      // In the selected mode read 3 bytes (pressure) and enable this slave transaction
      mpu_set(mpu, MPU60X0_REG_I2C_SLV3_CTRL, (0x1<<7) | 3);
      mpu_i2c->config.slaves[MPU_BARO_SLV_NB].mpu_slave_init_status++;
      break;

    case 9:
      return TRUE;
    default:
      break;
  }

  return FALSE;
}
#endif

#ifdef MPU9150_SLV_MAG
#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))
bool_t mpu9150_i2c_mag_event(struct Mpu60x0_I2c *mpu, struct Int32Vect3 *mag) {

int16_t value;

  // Mag data :
  //   Status 1 
  //     1 byte
  //   Measures :
  //     2 bytes 
  //     2 bytes 
  //     2 bytes 
  //   Status 2
  //     1 byte
  
  const uint8_t status1= mpu->data_ext[0];
  const uint8_t status2= mpu->data_ext[7];
  const bool_t dataReady = (status1 & 0b0001);
  const bool_t dataError = (status2 & 0b0100);
  const bool_t magneticOverflow = (status2 & 0b1000);
  
  if (dataReady) {
    if (magneticOverflow) {
      mag->x = mag->y = mag->z = 1e6f;
    } else if (dataError) {
      mag->x = mag->y = mag->z = -1e6f;
    } else {

      value = Int16FromBuf(mpu->data_ext,1);
      value = value & 0x1FFF;
      if(value > 0x0FFF) value = value - 0x2000;
      mag->x = i2cMasterGetAK8975_ajustedValue(value, 0);
    
      value = Int16FromBuf(mpu->data_ext,3);
      value = value & 0x1FFF;
      if(value > 0x0FFF) value = value - 0x2000;
      mag->y = i2cMasterGetAK8975_ajustedValue(value, 1);
    
      value = Int16FromBuf(mpu->data_ext,5);
      value = value & 0x1FFF;
      if(value > 0x0FFF) value = value - 0x2000;
      mag->z = i2cMasterGetAK8975_ajustedValue(value, 2);

      return TRUE;
    }
  }
  return FALSE;
}
#endif

#ifdef MPU9150_SLV_BARO
void mpu9150_i2c_baro_event(struct Mpu60x0_I2c *mpu, float *pressure) {


  // Baro data :
  //   Pressure (20 bits): 
  //     1 byte => [19-12]
  //     1 byte => [11-4]
  //     1 byte => [3-0]

  const uint8_t offset = 8;
  uint32_t tmp = (((uint32_t)mpu->data_ext[0+offset]<<16) |
                 ((uint16_t)mpu->data_ext[1+offset]<<8) |
                 mpu->data_ext[2+offset]);

  uint32_t int_pressure=(tmp>>4);  // pressure in 1/4 Pascal

  *pressure = ((float)int_pressure/(1<<2));
}
#endif

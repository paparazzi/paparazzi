/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file peripherals/mpu60x0_i2c.c
 *
 * Driver for the MPU-60X0 using I2C.
 *
 */

#include "peripherals/mpu60x0_i2c.h"

void mpu60x0_i2c_init(struct Mpu60x0_I2c *mpu, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  mpu->i2c_p = i2c_p;

  /* slave address */
  mpu->i2c_trans.slave_addr = addr;
  /* set inital status: Success or Done */
  mpu->i2c_trans.status = I2CTransDone;

  /* set default MPU60X0 config options */
  mpu60x0_set_default_config(&(mpu->config));

  mpu->data_available = FALSE;
  mpu->config.initialized = FALSE;
  mpu->config.init_status = MPU60X0_CONF_UNINIT;
}


static void mpu60x0_i2c_write_to_reg(void* mpu, uint8_t _reg, uint8_t _val) {
  struct Mpu60x0_I2c* mpu_i2c = (struct Mpu60x0_I2c*)(mpu);
  mpu_i2c->i2c_trans.buf[0] = _reg;
  mpu_i2c->i2c_trans.buf[1] = _val;
  i2c_transmit(mpu_i2c->i2c_p, &(mpu_i2c->i2c_trans), mpu_i2c->i2c_trans.slave_addr, 2);
}

// Configuration function called once before normal use
void mpu60x0_i2c_start_configure(struct Mpu60x0_I2c *mpu)
{
  if (mpu->config.init_status == MPU60X0_CONF_UNINIT) {
    mpu->config.init_status++;
    if (mpu->i2c_trans.status == I2CTransSuccess || mpu->i2c_trans.status == I2CTransDone) {
      mpu60x0_send_config(mpu60x0_i2c_write_to_reg, (void*)mpu, &(mpu->config));
    }
  }
}

void mpu60x0_i2c_read(struct Mpu60x0_I2c *mpu)
{
  if (mpu->config.initialized && mpu->i2c_trans.status == I2CTransDone) {
    /* set read bit and multiple byte bit, then address */
    mpu->i2c_trans.buf[0] = MPU60X0_REG_INT_STATUS;
    i2c_transceive(mpu->i2c_p, &(mpu->i2c_trans), mpu->i2c_trans.slave_addr, 1, 15);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void mpu60x0_i2c_event(struct Mpu60x0_I2c *mpu)
{
  if (mpu->config.initialized) {
    if (mpu->i2c_trans.status == I2CTransFailed) {
      mpu->i2c_trans.status = I2CTransDone;
    }
    else if (mpu->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      if (bit_is_set(mpu->i2c_trans.buf[0],0)) {
        // new data
        mpu->data_accel.vect.x = Int16FromBuf(mpu->i2c_trans.buf,1);
        mpu->data_accel.vect.y = Int16FromBuf(mpu->i2c_trans.buf,3);
        mpu->data_accel.vect.z = Int16FromBuf(mpu->i2c_trans.buf,5);
        mpu->data_rates.rates.p = Int16FromBuf(mpu->i2c_trans.buf,9);
        mpu->data_rates.rates.q = Int16FromBuf(mpu->i2c_trans.buf,11);
        mpu->data_rates.rates.r = Int16FromBuf(mpu->i2c_trans.buf,13);
        mpu->data_available = TRUE;
      }
      mpu->i2c_trans.status = I2CTransDone;
    }
  }
  else if (mpu->config.init_status != MPU60X0_CONF_UNINIT) { // Configuring but not yet initialized
    switch (mpu->i2c_trans.status) {
      case I2CTransFailed:
        mpu->config.init_status--; // Retry config (TODO max retry)
      case I2CTransSuccess:
      case I2CTransDone:
        mpu->i2c_trans.status = I2CTransDone;
        mpu60x0_send_config(mpu60x0_i2c_write_to_reg, (void*)mpu, &(mpu->config));
        if (mpu->config.initialized) mpu->i2c_trans.status = I2CTransDone;
        break;
      default:
        break;
    }
  }
}

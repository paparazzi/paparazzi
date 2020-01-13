/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/bmi088_i2c.c
 *
 * Driver for the BMI088 using I2C.
 *
 */

#include "peripherals/bmi088_i2c.h"

void bmi088_i2c_init(struct Bmi088_I2c *bmi, struct i2c_periph *i2c_p, uint8_t gyro_addr, uint8_t accel_addr)
{
  /* set i2c_peripheral */
  bmi->i2c_p = i2c_p;

  /* slave address */
  bmi->gyro_trans.slave_addr = gyro_addr;
  bmi->accel_trans.slave_addr = accel_addr;
  /* set inital status: Success or Done */
  bmi->gyro_trans.status = I2CTransDone;
  bmi->accel_trans.status = I2CTransDone;

  /* set default BMI088 config options */
  bmi088_set_default_config(&(bmi->config));

  bmi->gyro_available = false;
  bmi->accel_available = false;
  bmi->config.initialized = false;
  bmi->config.init_status = BMI088_CONF_UNINIT;
}


static void bmi088_i2c_write_to_reg(void *bmi, uint8_t _reg, uint8_t _val, uint8_t _type)
{
  struct Bmi088_I2c *bmi_i2c = (struct Bmi088_I2c *)(bmi);
  if (_type == BMI088_CONFIG_ACCEL) {
    bmi_i2c->accel_trans.buf[0] = _reg;
    bmi_i2c->accel_trans.buf[1] = _val;
    i2c_transmit(bmi_i2c->i2c_p, &(bmi_i2c->accel_trans), bmi_i2c->accel_trans.slave_addr, 2);
  } else if (_type == BMI088_CONFIG_GYRO) {
    bmi_i2c->gyro_trans.buf[0] = _reg;
    bmi_i2c->gyro_trans.buf[1] = _val;
    i2c_transmit(bmi_i2c->i2c_p, &(bmi_i2c->gyro_trans), bmi_i2c->gyro_trans.slave_addr, 2);
  }
}

// Configuration function called once before normal use
void bmi088_i2c_start_configure(struct Bmi088_I2c *bmi)
{
  if (bmi->config.init_status == BMI088_CONF_UNINIT) {
    bmi->config.init_status++;
    if (bmi->accel_trans.status == I2CTransSuccess || bmi->accel_trans.status == I2CTransDone) {
      bmi088_send_config(bmi088_i2c_write_to_reg, (void *)bmi, &(bmi->config));
    }
  }
}

void bmi088_i2c_read(struct Bmi088_I2c *bmi)
{
  if (bmi->config.initialized &&
      bmi->gyro_trans.status == I2CTransDone &&
      bmi->accel_trans.status == I2CTransDone) {
    /* read gyro */
    bmi->gyro_trans.buf[0] = BMI088_GYRO_RATE_X_LSB;
    i2c_transceive(bmi->i2c_p, &(bmi->gyro_trans), bmi->gyro_trans.slave_addr, 1, 6); //9);
    /* read accel */
    bmi->accel_trans.buf[0] = BMI088_ACCEL_X_LSB;
    i2c_transceive(bmi->i2c_p, &(bmi->accel_trans), bmi->accel_trans.slave_addr, 1, 6); // 12);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void bmi088_i2c_event(struct Bmi088_I2c *bmi)
{
  if (bmi->config.initialized) {
    // Check gyro read
    if (bmi->gyro_trans.status == I2CTransFailed) {
      bmi->gyro_trans.status = I2CTransDone;
    } else if (bmi->gyro_trans.status == I2CTransSuccess) {
      // Successfull reading
      //if (bit_is_set(bmi->gyro_trans.buf[8], 7)) {
        // new data
        bmi->data_rates.rates.p = Int16FromBuf(bmi->gyro_trans.buf, 0);
        bmi->data_rates.rates.q = Int16FromBuf(bmi->gyro_trans.buf, 2);
        bmi->data_rates.rates.r = Int16FromBuf(bmi->gyro_trans.buf, 4);
        bmi->gyro_available = true;
      //}
      bmi->gyro_trans.status = I2CTransDone;
    }
    // Check accel read
    if (bmi->accel_trans.status == I2CTransFailed) {
      bmi->accel_trans.status = I2CTransDone;
    } else if (bmi->accel_trans.status == I2CTransSuccess) {
      // Successfull reading
      //if (bit_is_set(bmi->accel_trans.buf[11], 7)) {
        // new data
        bmi->data_accel.vect.x = Int16FromBuf(bmi->accel_trans.buf, 0);
        bmi->data_accel.vect.y = Int16FromBuf(bmi->accel_trans.buf, 2);
        bmi->data_accel.vect.z = Int16FromBuf(bmi->accel_trans.buf, 4);
        bmi->accel_available = true;
      //}
      bmi->accel_trans.status = I2CTransDone;
    }
  } else if (bmi->config.init_status != BMI088_CONF_UNINIT) { // Configuring but not yet initialized
    if (bmi->config.init_status <= BMI088_CONF_ACCEL_PWR_CTRL) {
      // Accel config not finished
      switch (bmi->accel_trans.status) {
        case I2CTransFailed:
          bmi->config.init_status--; // Retry config (TODO max retry)
          /* Falls through. */
        case I2CTransSuccess:
        case I2CTransDone:
          bmi088_send_config(bmi088_i2c_write_to_reg, (void *)bmi, &(bmi->config));
          break;
        default:
          break;
      }
    }
    else {
      // gyro config not finished
      switch (bmi->gyro_trans.status) {
        case I2CTransFailed:
          bmi->config.init_status--; // Retry config (TODO max retry)
          /* Falls through. */
        case I2CTransSuccess:
        case I2CTransDone:
          bmi088_send_config(bmi088_i2c_write_to_reg, (void *)bmi, &(bmi->config));
          if (bmi->config.initialized) {
            bmi->gyro_trans.status = I2CTransDone;
          }
          break;
        default:
          break;
      }
    }
  }
}


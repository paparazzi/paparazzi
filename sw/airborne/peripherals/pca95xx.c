/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file peripherals/pca95xx.c
 *
 * Driver for the 8-bit I/O expander based on i2c
 */

#include "peripherals/pca95xx.h"

// Init function
void pca95xx_init(struct pca95xx *dev, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  dev->i2c_p = i2c_p;

  /* slave address */
  dev->i2c_trans.slave_addr = addr;
  /* set initial status: Done */
  dev->i2c_trans.status = I2CTransDone;
}

// Configure function
bool pca95xx_configure(struct pca95xx *dev, uint8_t val, bool blocking)
{
  if (dev->i2c_trans.status != I2CTransDone &&
      dev->i2c_trans.status != I2CTransSuccess &&
      dev->i2c_trans.status != I2CTransFailed) {
    return false; // previous transaction not finished
  }
  // send config value
  dev->i2c_trans.buf[0] = PCA95XX_CONFIG_REG;
  dev->i2c_trans.buf[1] = val;
  if (blocking) {
    return i2c_blocking_transmit(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 2);
  } else {
    return i2c_transmit(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 2);
  }
}

// Set output function
bool pca95xx_set_output(struct pca95xx *dev, uint8_t mask, bool blocking)
{
  if (dev->i2c_trans.status != I2CTransDone &&
      dev->i2c_trans.status != I2CTransSuccess &&
      dev->i2c_trans.status != I2CTransFailed) {
    return false; // previous transaction not finished
  }
  // send mask value
  dev->i2c_trans.buf[0] = PCA95XX_OUTPUT_REG;
  dev->i2c_trans.buf[1] = mask;
  if (blocking) {
    return i2c_blocking_transmit(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 2);
  } else {
    return i2c_transmit(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 2);
  }
}

// Get input function
bool pca95xx_get_input(struct pca95xx *dev, uint8_t mask, uint8_t *result) {
  if (dev->i2c_trans.status != I2CTransDone &&
      dev->i2c_trans.status != I2CTransSuccess &&
      dev->i2c_trans.status != I2CTransFailed) {
    return false; // previous transaction not finished
  }
  // get input register
  dev->i2c_trans.buf[0] = PCA95XX_INPUT_REG;
  bool ret = i2c_blocking_transceive(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 1, 1);
  *result = dev->i2c_trans.buf[0] & mask;
  return ret;
}


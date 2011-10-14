 /*
 * Copyright (C) 2010 ENAC
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


/* driver for the ADC ads1114 (16 bits I2C 860SpS max) from Texas instruments
 *  Navarro & Gorraz & Hattenberger
 */

#include "peripherals/ads1114.h"

struct i2c_transaction ads1114_trans;

bool_t ads1114_config_done;
bool_t ads1114_data_available;

void ads1114_init( void ) {
  /* configure the ads1114 */
  ads1114_trans.buf[0] = ADS1114_POINTER_CONFIG_REG;
  ads1114_trans.buf[1] = ADS1114_CONFIG_MSB;
  ads1114_trans.buf[2] = ADS1114_CONFIG_LSB;
  I2CTransmit(ADS1114_I2C_DEVICE, ads1114_trans, ADS1114_I2C_ADDR, 3);
  ads1114_config_done = FALSE;
  ads1114_data_available = FALSE;
}


void ads1114_read( void ) {
  // Config done with success
  // start new reading when previous is done (and read if success)
  if (ads1114_config_done && ads1114_trans.status == I2CTransDone) {
    ads1114_trans.buf[0] = ADS1114_POINTER_CONV_REG;
    I2CTransceive(ADS1114_I2C_DEVICE, ads1114_trans, ADS1114_I2C_ADDR, 1, 2);
  }
}



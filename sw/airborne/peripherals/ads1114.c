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

#if USE_ADS1114_1
struct ads1114_periph ads1114_1;
#endif
#if USE_ADS1114_2
struct ads1114_periph ads1114_2;
#endif

void ads1114_init(void)
{
  /* configure the ads1114_1 */
#if USE_ADS1114_1
  ads1114_1.i2c_addr = ADS1114_1_I2C_ADDR;
  ads1114_1.trans.buf[0] = ADS1114_POINTER_CONFIG_REG;
  ads1114_1.trans.buf[1] = ADS1114_1_CONFIG_MSB;
  ads1114_1.trans.buf[2] = ADS1114_1_CONFIG_LSB;
  i2c_transmit(&ADS1114_I2C_DEV, &ads1114_1.trans, ADS1114_1_I2C_ADDR, 3);
  ads1114_1.config_done = FALSE;
  ads1114_1.data_available = FALSE;
#endif

  /* configure the ads1114_2 */
#if USE_ADS1114_2
  ads1114_2.i2c_addr = ADS1114_2_I2C_ADDR;
  ads1114_2.trans.buf[0] = ADS1114_POINTER_CONFIG_REG;
  ads1114_2.trans.buf[1] = ADS1114_2_CONFIG_MSB;
  ads1114_2.trans.buf[2] = ADS1114_2_CONFIG_LSB;
  i2c_transmit(&ADS1114_I2C_DEV, &ads1114_2.trans, ADS1114_2_I2C_ADDR, 3);
  ads1114_2.config_done = FALSE;
  ads1114_2.data_available = FALSE;
#endif
}


void ads1114_read(struct ads1114_periph *p)
{
  // Config done with success
  // start new reading when previous is done (and read if success)
  if (p->config_done && p->trans.status == I2CTransDone) {
    p->trans.buf[0] = ADS1114_POINTER_CONV_REG;
    i2c_transceive(&ADS1114_I2C_DEV, &(p->trans), p->i2c_addr, 1, 2);
  }
}



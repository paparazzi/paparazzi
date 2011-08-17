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

#ifndef ADS_1114_H
#define ADS_1114_H

#include "std.h"
#include "mcu_periph/i2c.h"


/* I2C slave address */
#define ADS1114_I2C_ADDR 0x90           // slave address byte (I2c address(7bits) + R/W @ 0)

/* I2C conf register */
#define ADS1114_POINTER_CONV_REG    0x00 // access to the Conversion register (16bits)
#define ADS1114_POINTER_CONFIG_REG  0x01 // access to the Configuration register (16bits)

/* ADS1114 default conf */
#ifndef ADS1114_OS
#define ADS1114_OS 0x0 // Operational status
#endif
#ifndef ADS1114_MUX
#define ADS1114_MUX 0x0 // Input multiplexer
#endif
#ifndef ADS1114_PGA
#define ADS1114_PGA 0x3 // Programable gain amplifier (= 4 with a Full Scale of +/- 1.024V)
#endif
#ifndef ADS1114_MODE
#define ADS1114_MODE 0x0 // Continuous conversion mode
#endif
#ifndef ADS1114_DR
#define ADS1114_DR 0x4 // Data rate (128 SPS)
#endif
#ifndef ADS1114_COMP_MODE
#define ADS1114_COMP_MODE 0x0 // Comparator mode
#endif
#ifndef ADS1114_COMP_POL
#define ADS1114_COMP_POL 0x0 // Comparator polarity
#endif
#ifndef ADS1114_COMP_LAT
#define ADS1114_COMP_LAT 0x0 // Latching comparator
#endif
#ifndef ADS1114_COMP_QUE
#define ADS1114_COMP_QUE 0x3 // Comparator queue (disable)
#endif

#define ADS1114_CONFIG_MSB ((ADS1114_OS<<7)|(ADS1114_MUX<<4)|(ADS1114_PGA<<1)|(ADS1114_MODE))
#define ADS1114_CONFIG_LSB ((ADS1114_DR<<5)|(ADS1114_COMP_MODE<<4)|(ADS1114_COMP_POL<<3)|(ADS1114_COMP_LAT<<2)|(ADS1114_COMP_QUE))

/* Default I2C device */
#ifndef ADS1114_I2C_DEVICE
#define ADS1114_I2C_DEVICE i2c1
#endif

extern struct i2c_transaction ads1114_trans;
extern bool_t ads1114_config_done;
extern bool_t ads1114_data_available;

extern void ads1114_init(void);
extern void ads1114_read(void);

#define Ads1114Event() { \
  if (!ads1114_config_done) { \
    if (ads1114_trans.status == I2CTransSuccess) { ads1114_config_done = TRUE; ads1114_trans.status = I2CTransDone; } \
    if (ads1114_trans.status == I2CTransFailed) { ads1114_trans.status = I2CTransDone; } \
  } else { \
    if (ads1114_trans.status == I2CTransSuccess) { ads1114_data_available = TRUE; ads1114_trans.status = I2CTransDone; } \
    if (ads1114_trans.status == I2CTransFailed) { ads1114_trans.status = I2CTransDone; } \
  }\
}

#define Ads1114GetValue() ((int16_t)(((int16_t)ads1114_trans.buf[0]<<8)|ads1114_trans.buf[1]))

#endif // ADS_1114_H

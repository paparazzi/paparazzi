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
#ifndef ADS1114_1_I2C_ADDR
#define ADS1114_1_I2C_ADDR 0x90           // slave address byte (I2c address(7bits) + R/W @ 0)
#endif
#ifndef ADS1114_2_I2C_ADDR
#define ADS1114_2_I2C_ADDR 0x92           // slave address byte (I2c address(7bits) + R/W @ 0)
#endif

/* I2C conf register */
#define ADS1114_POINTER_CONV_REG    0x00 // access to the Conversion register (16bits)
#define ADS1114_POINTER_CONFIG_REG  0x01 // access to the Configuration register (16bits)

/* ADS1114_1 default conf */
#ifndef ADS1114_1_OS
#define ADS1114_1_OS 0x0 // Operational status
#endif
#ifndef ADS1114_1_MUX
#define ADS1114_1_MUX 0x0 // Input multiplexer
#endif
#ifndef ADS1114_1_PGA
#define ADS1114_1_PGA 0x3 // Programable gain amplifier (= 4 with a Full Scale of +/- 1.024V)
#endif
#ifndef ADS1114_1_MODE
#define ADS1114_1_MODE 0x0 // Continuous conversion mode
#endif
#ifndef ADS1114_1_DR
#define ADS1114_1_DR 0x4 // Data rate (128 SPS)
#endif
#ifndef ADS1114_1_COMP_MODE
#define ADS1114_1_COMP_MODE 0x0 // Comparator mode
#endif
#ifndef ADS1114_1_COMP_POL
#define ADS1114_1_COMP_POL 0x0 // Comparator polarity
#endif
#ifndef ADS1114_1_COMP_LAT
#define ADS1114_1_COMP_LAT 0x0 // Latching comparator
#endif
#ifndef ADS1114_1_COMP_QUE
#define ADS1114_1_COMP_QUE 0x3 // Comparator queue (disable)
#endif

#define ADS1114_1_CONFIG_MSB ((ADS1114_1_OS<<7)|(ADS1114_1_MUX<<4)|(ADS1114_1_PGA<<1)|(ADS1114_1_MODE))
#define ADS1114_1_CONFIG_LSB ((ADS1114_1_DR<<5)|(ADS1114_1_COMP_MODE<<4)|(ADS1114_1_COMP_POL<<3)|(ADS1114_1_COMP_LAT<<2)|(ADS1114_1_COMP_QUE))

/* ADS1114_1 default conf */
#ifndef ADS1114_2_OS
#define ADS1114_2_OS 0x0 // Operational status
#endif
#ifndef ADS1114_2_MUX
#define ADS1114_2_MUX 0x0 // Input multiplexer
#endif
#ifndef ADS1114_2_PGA
#define ADS1114_2_PGA 0x3 // Programable gain amplifier (= 4 with a Full Scale of +/- 1.024V)
#endif
#ifndef ADS1114_2_MODE
#define ADS1114_2_MODE 0x0 // Continuous conversion mode
#endif
#ifndef ADS1114_2_DR
#define ADS1114_2_DR 0x4 // Data rate (128 SPS)
#endif
#ifndef ADS1114_2_COMP_MODE
#define ADS1114_2_COMP_MODE 0x0 // Comparator mode
#endif
#ifndef ADS1114_2_COMP_POL
#define ADS1114_2_COMP_POL 0x0 // Comparator polarity
#endif
#ifndef ADS1114_2_COMP_LAT
#define ADS1114_2_COMP_LAT 0x0 // Latching comparator
#endif
#ifndef ADS1114_2_COMP_QUE
#define ADS1114_2_COMP_QUE 0x3 // Comparator queue (disable)
#endif

#define ADS1114_2_CONFIG_MSB ((ADS1114_2_OS<<7)|(ADS1114_2_MUX<<4)|(ADS1114_2_PGA<<1)|(ADS1114_2_MODE))
#define ADS1114_2_CONFIG_LSB ((ADS1114_2_DR<<5)|(ADS1114_2_COMP_MODE<<4)|(ADS1114_2_COMP_POL<<3)|(ADS1114_2_COMP_LAT<<2)|(ADS1114_2_COMP_QUE))

/* Default I2C device */
// FIXME all ads on the same device for now
#ifndef ADS1114_I2C_DEV
#define ADS1114_I2C_DEV i2c1
#endif

struct ads1114_periph {
  struct i2c_transaction trans;
  uint8_t i2c_addr;
  bool config_done;
  bool data_available;
};

#if USE_ADS1114_1
extern struct ads1114_periph ads1114_1;
#endif

#if USE_ADS1114_2
extern struct ads1114_periph ads1114_2;
#endif

extern void ads1114_init(void);
extern void ads1114_read(struct ads1114_periph *p);

// Generic Event Macro
#define _Ads1114Event(_p) {\
    if (!_p.config_done) { \
      if (_p.trans.status == I2CTransSuccess) { _p.config_done = true; _p.trans.status = I2CTransDone; } \
      if (_p.trans.status == I2CTransFailed) { _p.trans.status = I2CTransDone; } \
    } else { \
      if (_p.trans.status == I2CTransSuccess) { _p.data_available = true; _p.trans.status = I2CTransDone; } \
      if (_p.trans.status == I2CTransFailed) { _p.trans.status = I2CTransDone; } \
    } \
  }

#if USE_ADS1114_1
#define Ads1114_1Event() _Ads1114Event(ads1114_1)
#else
#define Ads1114_1Event() {}
#endif

#if USE_ADS1114_2
#define Ads1114_2Event() _Ads1114Event(ads1114_2)
#else
#define Ads1114_2Event() {}
#endif

// Final event macro
#define Ads1114Event() {  \
    Ads1114_1Event();       \
    Ads1114_2Event();       \
  }

// Get value macro
// @param ads1114 periph
#define Ads1114GetValue(_p) ((int16_t)(((int16_t)_p.trans.buf[0]<<8)|_p.trans.buf[1]))

#endif // ADS_1114_H

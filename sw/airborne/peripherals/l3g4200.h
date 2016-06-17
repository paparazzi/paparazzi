/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
 *               2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 * @file peripherals/l3g4200.h
 *
 * Driver for the gyro L3G4200 From ST.
 */
#ifndef L3G4200_H
#define L3G4200_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Address and register definitions */
#include "peripherals/l3g4200_regs.h"


// Default Output rate 100hz
#define L3G4200_DEFAULT_DR L3G4200_DR_100Hz
// Default digital lowpass filter 25hz
#define L3G4200_DEFAULT_DLPF L3G4200_DLPF_1
// Default Scale
#define L3G4200_DEFAULT_SCALE L3G4200_SCALE_2000

/* Default conf */
#define L3G4200_DEFAULT_CTRL_REG1 ((L3G4200_DEFAULT_DR<<6) | (L3G4200_DEFAULT_DLPF<<4) | 0xf);
#define L3G4200_DEFAULT_CTRL_REG4 (L3G4200_DEFAULT_SCALE<<4) | 0x00; // 2000deg = 0x30
#define L3G4200_DEFAULT_CTRL_REG5 0x00 // first low pass filter enable

struct L3g4200Config {
  uint8_t ctrl_reg1;
  uint8_t ctrl_reg4;
  uint8_t ctrl_reg5;
};

/** config status states */
enum L3g4200ConfStatus {
  L3G_CONF_UNINIT,
  L3G_CONF_REG1,
  L3G_CONF_REG4,
  L3G_CONF_REG5,
  L3G_CONF_DONE
};

struct L3g4200 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool initialized;                 ///< config done flag
  enum L3g4200ConfStatus init_status; ///< init status
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int32Rates rates;          ///< data as angular rates in gyro coordinate system
    int32_t value[3];                 ///< data values accessible by channel index
  } data;
  struct L3g4200Config config;
};

// Functions
extern void l3g4200_init(struct L3g4200 *l3g, struct i2c_periph *i2c_p, uint8_t i2c_address);
extern void l3g4200_set_default_config(struct L3g4200Config *conf);
extern void l3g4200_start_configure(struct L3g4200 *l3g);
extern void l3g4200_read(struct L3g4200 *l3g);
extern void l3g4200_event(struct L3g4200 *l3g);

/// convenience function: read or start configuration if not already initialized
static inline void l3g4200_periodic(struct L3g4200 *l3g)
{
  if (l3g->initialized) {
    l3g4200_read(l3g);
  } else {
    l3g4200_start_configure(l3g);
  }
}

#endif // L3G4200_H

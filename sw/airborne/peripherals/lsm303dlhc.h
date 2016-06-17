/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lsm303dlhc.h
 *
 * Driver for ST LSM303DLHC 3D accelerometer and magnetometer.
 */
#ifndef LSM303_H
#define LSM303_H

#include "std.h"
/* Address and register definitions */
#include "peripherals/lsm303dlhc_regs.h"

/* LSM303DLHC default conf */
#ifndef LSM303DLHC_DEFAULT_AODR
#define LSM303DLHC_DEFAULT_AODR (0x01 << 4) //acc 3125 Hz
#endif

#ifndef LSM303DLHC_DEFAULT_AFS
#define LSM303DLHC_DEFAULT_AFS (0x04 <<3) // acc +- 16G
#endif

#ifndef LSM303DLHC_DEFAULT_MODR
#define LSM303DLHC_DEFAULT_MODR (0x5 << 2) // Magneto Data Output Rate (100Hz)
#endif

#ifndef LSM303DLHC_DEFAULT_MFS
#define LSM303DLHC_DEFAULT_MFS (0x0 << 5) // Magneto gain configuration (+/- 2 Gauss)
#endif

#ifndef LSM303DLHC_DEFAULT_MD
#define LSM303DLHC_DEFAULT_MD (0x00 << 0) // Magneto continious conversion mode
#endif

struct Lsm303dlhcAccConfig {
  uint8_t rate;    ///< Data Output Rate (Hz)
  uint8_t scale;   ///< full scale selection (m/s²)
};

struct Lsm303dlhcMagConfig {
  uint8_t rate;  ///< Data Output Rate Bits (Hz)
  uint8_t scale;  ///< Full scale gain configuration (Gauss)
  uint8_t mode;  ///< Measurement mode
};

/** config status states */
enum Lsm303dlhcConfStatus {
  LSM_CONF_UNINIT,
  LSM_CONF_WHO_AM_I,
  LSM_CONF_CTRL_REG1,
  LSM_CONF_CTRL_REG2,
  LSM_CONF_CTRL_REG3,
  LSM_CONF_CTRL_REG4,
  LSM_CONF_CTRL_REG5,
  LSM_CONF_CTRL_REG6,
  LSM_CONF_CTRL_REG7,
  LSM_CONF_DONE
};

enum Lsm303dlhcTarget {
  LSM_TARGET_ACC,
  LSM_TARGET_MAG
};

static inline void lsm303dlhc_acc_set_default_config(struct Lsm303dlhcAccConfig *c)
{
  c->rate = LSM303DLHC_DEFAULT_AODR;
  c->scale = LSM303DLHC_DEFAULT_AFS;
}

static inline void lsm303dlhc_mag_set_default_config(struct Lsm303dlhcMagConfig *c)
{
  c->rate = LSM303DLHC_DEFAULT_MODR;
  c->scale = LSM303DLHC_DEFAULT_MFS;
  c->mode = LSM303DLHC_DEFAULT_MD;
}
#endif // LSM303_H

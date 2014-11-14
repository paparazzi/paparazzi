/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/ak8963.c
 *
 * Driver for the AKM AK8963 magnetometer.
 */

#ifndef AK8963_H
#define AK8963_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* Address and register definitions */
#include "peripherals/ak8963_regs.h"

/** Config status states */
enum Ak8963ConfStatus {
  AK_CONF_UNINIT,
  AK_CONF_MODE,
  AK_CONF_DONE
};

/** Normal status states */
enum Ak8963Status {
  AK_STATUS_IDLE,
  AK_STATUS_READ,
  AK_STATUS_DONE
};

/** Default Ak8963 structure */
struct Ak8963 {
  struct i2c_periph *i2c_p;           ///< peripheral used for communcation
  struct i2c_transaction i2c_trans;   ///< i2c transaction used for communication with the ak8936
  bool_t initialized;                 ///< config done flag

  enum Ak8963Status status;           ///< main status
  enum Ak8963ConfStatus init_status;  ///< init status

  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
};

// Functions
extern void ak8963_init(struct Ak8963 *ak, struct i2c_periph *i2c_p, uint8_t addr);
extern void ak8963_configure(struct Ak8963 *ak);
extern void ak8963_event(struct Ak8963 *ak);
extern void ak8963_read(struct Ak8963 *ak);

/// convenience function: read or start configuration if not already initialized
static inline void ak8963_periodic(struct Ak8963 *ak)
{
  if (ak->initialized) {
    ak8963_read(ak);
  } else {
    ak8963_configure(ak);
  }
}

#endif /* AK8963_H */

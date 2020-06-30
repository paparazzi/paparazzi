/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/ist8310.c
 *
 * Driver for the Isentek IST8310 magnetometer.
 */

#ifndef IST8310_H
#define IST8310_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* Address and register definitions */
#include "peripherals/ist8310_regs.h"

/** Config status states */
enum IST8310ConfStatus {
  IST_CONF_UNINIT,
  IST_CONF_CNTL1,
  IST_CONF_CNTL3,
  IST_CONF_CNTL4,
  IST_CONF_DONE
};

/** Normal status states */
enum IST8310Status {
  IST_STATUS_IDLE,
  IST_STATUS_READ,
  IST_STATUS_DONE
};

/** Default IST8310 structure */
struct IST8310 {
  struct i2c_periph *i2c_p;             ///< peripheral used for communcation
  struct i2c_transaction i2c_trans;     ///< i2c transaction used for communication with the ist8310
  bool initialized;                     ///< config done flag

  enum IST8310Status status;            ///< main status
  enum IST8310ConfStatus init_status;   ///< init status

  volatile bool data_available;         ///< data ready flag
  union {
    struct Int16Vect3 vect;             ///< data vector in mag coordinate system
    int16_t value[3];                   ///< data values accessible by channel index
  } data;
};

// Functions
extern void ist8310_init(struct IST8310 *ist, struct i2c_periph *i2c_p, uint8_t addr);
extern void ist8310_configure(struct IST8310 *ist);
extern void ist8310_event(struct IST8310 *ist);
extern void ist8310_read(struct IST8310 *ist);

/// convenience function: read or start configuration if not already initialized
static inline void ist8310_periodic(struct IST8310 *ist)
{
  if (ist->initialized) {
    ist8310_read(ist);
  } else {
    ist8310_configure(ist);
  }
}

#endif /* IST8310_H */

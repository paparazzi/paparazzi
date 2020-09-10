/*
 * Copyright (C) 2020 OpenUAS <noreply@openuas.org>
 * Thanks to Michal Podhradsky for SF11 work done
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

/** @file modules/lidar/tfmini_i2c.h
 *  @brief Driver for the TFMini ranging device connected via I2C port
 *
 * Driver for the I2C and S version of the TFMini(and TFLuna) based range sensor
 * Reads sensor using I2Cinput and outputs the distance to object or ground
 *
 */
#ifndef TFMINI_I2C_H
#define TFMINI_I2C_H

#include "stdbool.h"
#include "mcu_periph/i2c.h"

enum TFMiniI2CStatus {
  TFMINI_I2C_ACQUIRE,
  TFMINI_I2C_PARSE
};

struct TFMiniI2C {
  struct i2c_transaction trans;
  uint8_t addr;
  enum TFMiniI2CStatus status;
  uint16_t raw_dist; ///< raw non scaled value from sensor
  uint16_t strength; ///< strength of reflect signal, not implemented ATM
  float dist; ///< Sacled distance measured in [m]
  float offset;   ///< offset to what one considers a zero distance to sensor in [m]
  bool update_agl; ///< Do or don't update AGL ABI message
  bool compensate_rotation; //< Do or not compensate for range value when main body is rotated
};

extern struct TFMiniI2C tfmini_i2c;

extern void tfmini_i2c_init(void);
extern void tfmini_i2c_event(void);
extern void tfmini_i2c_periodic(void);
extern void tfmini_i2c_downlink(void);

#endif /* TFMINI_I2C_H */

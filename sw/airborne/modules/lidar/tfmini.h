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
 *
 */

/** @file modules/lidar/tfmini.h
 *  @brief driver for the TFMini lidar
 *
 */
#ifndef LIDAR_TFMINI_H
#define LIDAR_TFMINI_H

#include "std.h"
#include "mcu_periph/i2c.h"

enum TFMiniParseStatus {
  TFMINI_INITIALIZE,
  TFMINI_PARSE_HEAD,
  TFMINI_PARSE_HEAD2,
  TFMINI_PARSE_DIST_L,
  TFMINI_PARSE_DIST_H,
  TFMINI_PARSE_STRENGTH_L,
  TFMINI_PARSE_STRENGTH_H,
  TFMINI_PARSE_MODE,
  TFMINI_PARSE_BYTE7,
  TFMINI_PARSE_CHECKSUM
};

struct TFMini {
  struct link_device *device;
  enum TFMiniParseStatus parse_status;
  uint8_t parse_crc;
  uint16_t raw_dist;
  uint16_t raw_strength;
  uint8_t raw_mode;

  uint16_t strength;
  float distance; // [m]
  uint8_t mode;
  bool update_agl;
  bool compensate_rotation;
};

extern struct TFMini tfmini;

extern void tfmini_init(void);
extern void tfmini_event(void);
extern void tfmini_downlink(void);

#endif /* LIDAR_TFMINI_H */


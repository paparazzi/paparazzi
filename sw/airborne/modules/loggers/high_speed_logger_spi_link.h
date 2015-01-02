/*
 * Copyright (C) 2005-2013 The Paparazzi Team
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

#ifndef HIGH_SPEED_LOGGER_SPI_LINK_H_
#define HIGH_SPEED_LOGGER_SPI_LINK_H_

#include "std.h"

extern void high_speed_logger_spi_link_init(void);
extern void high_speed_logger_spi_link_periodic(void);

#define PACKED __attribute__((__packed__))

struct PACKED high_speed_logger_spi_link_data {
  int32_t id;         // 1
  int32_t gyro_p;     // 2
  int32_t gyro_q;
  int32_t gyro_r;
  int32_t acc_x;      // 5
  int32_t acc_y;
  int32_t acc_z;
  int32_t mag_x;      // 8
  int32_t mag_y;
  int32_t mag_z;
  int32_t phi;        // 11
  int32_t theta;
  int32_t psi;
  int32_t extra1;     // 14
  int32_t extra2;     // 15
  int32_t extra3;     // 16
};

#endif /* HIGH_SPEED_LOGGER_SPI_LINK_H_ */

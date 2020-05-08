/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/peripherals/pmw3901.h"
 * @author Tom van Dijk
 * Low-level driver for PMW3901 optical flow sensor
 *
 * Some notes about the sensor:
 * - The sensor is extremely poorly documented:
 *   - There are no units or delta_t's specified for the delta_x and _y registers.
 *   - The datasheet does not specify whether delta_x and _y are rates [something/s]
 *     or pixel counts. In the latter case, the datasheet does not specify when
 *     delta_x and _y are reset to 0.
 *   - The squal quality(?) register is not described.
 *
 * Based on crazyflie-firmware::kalman_core.c::512
 * - delta_x, _y are pixel counts, not rates.
 * - crazyflie uses scaling factor of thetapix/Npix = 0,002443389 rad/px,
 *   corresponding to a focal length of ~409 px.
 */

#ifndef PMW3901_H_
#define PMW3901_H_

#include "mcu_periph/spi.h"

#include <stdbool.h>
#include <math.h>


#define SPI_BUFFER_SIZE 8


enum pmw3901_state {
  PMW3901_IDLE,
  PMW3901_READ_MOTION,
  PMW3901_READ_DELTAXLOW,
  PMW3901_READ_DELTAXHIGH,
  PMW3901_READ_DELTAYLOW,
  PMW3901_READ_DELTAYHIGH,
};

struct pmw3901_t {
  struct spi_periph *periph;
  struct spi_transaction trans;
  volatile uint8_t spi_input_buf[SPI_BUFFER_SIZE];
  volatile uint8_t spi_output_buf[SPI_BUFFER_SIZE];
  enum pmw3901_state state;
  uint8_t readwrite_state;
  uint32_t readwrite_timeout;
  int16_t delta_x;
  int16_t delta_y;
  bool data_available;
  float rad_per_px;
};

void pmw3901_init(struct pmw3901_t *pmw, struct spi_periph *periph, uint8_t slave_idx);

void pmw3901_event(struct pmw3901_t *pmw);

bool pmw3901_is_idle(struct pmw3901_t *pmw);
void pmw3901_start_read(struct pmw3901_t *pmw);
bool pmw3901_data_available(struct pmw3901_t *pmw);
bool pmw3901_get_data(struct pmw3901_t *pmw, int16_t *delta_x, int16_t *delta_y);



#endif // PMW3901_H_

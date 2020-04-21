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
 * @file "modules/sensors/opticflow_pmw3901.c"
 * @author Tom van Dijk
 * Driver for PMW3901 optical flow sensor
 */

#include "modules/sensors/opticflow_pmw3901.h"

#include "peripherals/pmw3901.h"

struct pmw3901_t pmw;

extern void opticflow_pmw3901_init(void) {
  pmw3901_init(&pmw, &OPTICFLOW_PMW3901_SPI_DEV, OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
}

void opticflow_pmw3901_event(void) {

}



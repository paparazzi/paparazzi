/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/meteo/cloud_sim.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic cloud simulation for testing adaptive navigation patterns
 */

#ifndef CLOUD_SIM_H
#define CLOUD_SIM_H

#include "std.h"
#include "math/pprz_algebra_float.h"

#define CLOUD_SIM_WP      0
#define CLOUD_SIM_POLYGON 1

struct CloudSim {
  bool reset;
  uint8_t mode;
  struct FloatVect2 speed;
  float radius; ///< radius in WP mode
};

extern struct CloudSim cloud_sim;

extern void cloud_sim_init(void);
extern void cloud_sim_detect(void);
extern void cloud_sim_move(void);
extern void cloud_sim_reset(bool reset);

#endif  // CLOUD_SIM_H

/*
 * Copyright (C) 2025 Noah Wechtler <noahwechtler@tudelft.nl>
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
 * @file "modules/checks/airspeed_consistency.h"
 * @author Noah Wechtler <noahwechtler@tudelft.nl>
 * Check the consistency of airspeed measurements while flying circles
 */

#ifndef AIRSPEED_CONSISTENCY_H
#define AIRSPEED_CONSISTENCY_H

#ifndef AIRSPEED_CONSISTENCY_BUFFER_SIZE
#define AIRSPEED_CONSISTENCY_BUFFER_SIZE 255
#endif

#include <std.h>
#include "math/pprz_circfit_float.h"
#include "filters/low_pass_filter.h"

extern bool asc_reset;

struct asc_t {
  bool filled;
  uint16_t i; 
  float gs_N[AIRSPEED_CONSISTENCY_BUFFER_SIZE];
  float gs_E[AIRSPEED_CONSISTENCY_BUFFER_SIZE];
  float as_N[AIRSPEED_CONSISTENCY_BUFFER_SIZE];
  float as_E[AIRSPEED_CONSISTENCY_BUFFER_SIZE];
  struct circle_t as;
  struct circle_t gs;
  enum CircFitStatus_t as_circ_status;
  enum CircFitStatus_t gs_circ_status;
  float ratio;
};

extern void airspeed_consistency_init(void);
extern void airspeed_consistency_periodic(void);
extern void airspeed_consistency_reset(bool reset); // Reset the airspeed consistency check
float airspeed_consistency_get_gs_to_as_ratio(void); // Ratio between ground speed and airspeed fit

#endif // AIRSPEED_CONSISTENCY_H
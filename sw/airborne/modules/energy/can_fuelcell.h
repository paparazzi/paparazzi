/*
 * Copyright (C) 2024 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/energy/can_fuelcell.c"
 * @author F. van Tienen
 * Fuel-cell data through CAN bus
 */

#ifndef CAN_FUELCELL_H
#define CAN_FUELCELL_H

#include "std.h"

extern void can_fuelcell_init(void);
extern void can_fuelcell_periodic(void); // for simulation

#endif

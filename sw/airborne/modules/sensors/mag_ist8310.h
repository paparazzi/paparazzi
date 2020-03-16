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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/sensors/mag_ist8310.h
 *
 * Module wrapper for Isentek IST8310 magnetometers.
 */

#ifndef MAG_IST8310_H
#define MAG_IST8310_H

#include "peripherals/ist8310.h"

extern struct IST8310 mag_ist8310;

extern void mag_ist8310_module_init(void);
extern void mag_ist8310_module_periodic(void);
extern void mag_ist8310_module_event(void);
extern void mag_ist8310_report(void);

#endif // MAG_IST8310_H

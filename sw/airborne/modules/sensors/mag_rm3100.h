/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/sensors/mag_rm3100.h
 *
 * Module wrapper for PNI RM3100 magnetometers.
 */

#ifndef MAG_RM3100_H
#define MAG_RM3100_H

#include "peripherals/rm3100.h"

extern struct Rm3100 mag_rm3100;

extern void mag_rm3100_module_init(void);
extern void mag_rm3100_module_periodic(void);
extern void mag_rm3100_module_event(void);
extern void mag_rm3100_report(void);

#endif // MAG_RM3100_H

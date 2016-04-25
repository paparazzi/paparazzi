/*
 * Copyright (C) 2013 Martin Mueller <martinmm@pfump.org>
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/sensors/airspeed_uADC.h
 * UART interface for Aeroprobe uADC air data computer.
 *
 */

#ifndef AIRSPEED_uADC_H
#define AIRSPEED_uADC_H

#include "std.h"

extern void airspeed_uADC_init(void);
extern void airspeed_uADC_event(void);
extern void airspeed_uADC_periodic(void);

#endif

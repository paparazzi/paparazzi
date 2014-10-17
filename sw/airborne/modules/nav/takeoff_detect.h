/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi

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
 *
 */

/**
 * @file modules/nav/takeoff_detect.h
 *
 * Automatic takeoff assistance for fixed-wing.
 * The planes's launching can be triggered/aborted
 * by pointing the nose up or down for a given time.
 */

#ifndef TAKEOFF_DETECT_H
#define TAKEOFF_DETECT_H

/** Init function */
extern void takeoff_detect_init(void);

/** Start function called once before periodic
 */
extern void takeoff_detect_start(void);

/** Periodic call
 *
 * - can be enabled or disabled from settings
 * - enabled by default, disable himself after launch
 */
extern void takeoff_detect_periodic(void);

#endif


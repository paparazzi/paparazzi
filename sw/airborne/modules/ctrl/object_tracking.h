/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/ctrl/object_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control a rotorcraft heading to track an object detected by a camera
 */

#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

#include "std.h"

/** max turn rate in control mode in rad/s
 */
extern float object_tracking_rate;

/** max turn rate in search mode in rad/s
 */
extern float object_tracking_search_rate;

/** init function
 */
extern void object_tracking_init(void);

/** run function
 *
 * should be called in a flight plan stay block using pre_call
 *
 * ex:
 *  <block name="Track Object">
 *    <stay wp="STDBY" pre_call="object_tracking_run()"/>
 *  </block>
 *
 */
extern void object_tracking_run(void);

#endif


/*
 * Copyright (C) 2017 K. N. McGuire
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
/*
 * @file "modules/range_finder/laser_range_array.h"
 * @author K. N. McGuire
 * Reads out values through uart of an laser range ring (array), containing multiple ToF IR laser range modules
 */

#ifndef LASER_RANGE_ARRAY_H
#define LASER_RANGE_ARRAY_H

extern void laser_range_array_init(void);
extern void laser_range_array_event(void);

#endif


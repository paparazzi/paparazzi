/*
 * Copyright (C)
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
 * @file "modules/laser_range_array/laser_range_array.h"
 * @author
 * Reads out values through uart of an laser range ring (array), containing multiple ToF IR laser range modules
 */

#ifndef LASER_RANGE_ARRAY_H
#define LASER_RANGE_ARRAY_H

#include "std.h"
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

/* Main magneto pitot strcuture */
struct laser_range_array_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                 ///< If we received a message
};

extern void laser_ring_array_init(void);
extern void laser_range_array_event(void);

#endif


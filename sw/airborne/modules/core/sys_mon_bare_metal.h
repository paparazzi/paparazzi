/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file sys_mon.h
 *
 * System monitoring
 * return cpu load, average exec time, ...
 */

#ifndef SYS_MON_BARE_METAL_H
#define SYS_MON_BARE_METAL_H

#include "core/sys_mon.h"

struct SysMon {
  uint8_t  cpu_load;
  uint16_t periodic_time;      ///< in usec
  uint16_t periodic_time_min;  ///< in usec
  uint16_t periodic_time_max;  ///< in usec
  uint16_t periodic_cycle;     ///< in usec
  uint16_t periodic_cycle_min; ///< in usec
  uint16_t periodic_cycle_max; ///< in usec
  uint16_t event_number;
  float cpu_time; // in secs since startup
};

extern struct SysMon sys_mon;

#endif /* SYS_MON_BARE_METAL_H */

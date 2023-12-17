/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 */

/**
 * @file modules/energy/electrical.h
 *
 * Interface for electrical status: supply voltage, current, battery status, etc.
 */

#ifndef ELECTRICAL_H
#define ELECTRICAL_H

#include "std.h"
#include "generated/airframe.h"

/** low battery level in Volts (for 3S LiPo) */
#ifndef LOW_BAT_LEVEL
#define LOW_BAT_LEVEL 10.5
#endif

/** critical battery level in Volts (for 3S LiPo) */
#ifndef CRITIC_BAT_LEVEL
#define CRITIC_BAT_LEVEL 9.8
#endif

struct Electrical {
  float vsupply;        ///< supply voltage in V
  float vboard;         ///< board voltage in V
  float current;        ///< current in A
  float charge;         ///< consumed electric charge in Ah
  float energy;         ///< consumed energy in Wh
  bool  bat_low;        ///< battery low status
  bool  bat_critical;   ///< battery critical status

  uint32_t avg_power;   ///< average power sum
  uint32_t avg_cnt;     ///< average power counter
};

extern struct Electrical electrical;

extern void electrical_init(void);
extern void electrical_periodic(void);
extern void electrical_avg_reset(float var);

#endif /* ELECTRICAL_H */

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
 * @file subsystems/electrical.h
 *
 * Interface for electrical status: supply voltage, current, battery status, etc.
 */

#ifndef SUBSYSTEMS_ELECTRICAL_H
#define SUBSYSTEMS_ELECTRICAL_H

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

  uint16_t vsupply;       ///< supply voltage in decivolts
  int32_t  current;       ///< current in milliamps
  int32_t  consumed;      ///< consumption in mAh
  float    energy;        ///< consumed energy in mAh
  bool_t   bat_low;       ///< battery low status
  bool_t   bat_critical;  ///< battery critical status

};

extern struct Electrical electrical;

extern void electrical_init(void);
extern void electrical_periodic(void);

#endif /* SUBSYSTEMS_ELECTRICAL_H */

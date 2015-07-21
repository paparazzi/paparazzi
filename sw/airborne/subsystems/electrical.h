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

#ifndef BAT_CHECKER_DELAY
#define BAT_CHECKER_DELAY 5
#endif

#define ELECTRICAL_PERIODIC_FREQ 10

#ifndef MIN_BAT_LEVEL
#define MIN_BAT_LEVEL 3
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


static inline void electrical_checks(void) {



  static uint32_t bat_low_counter = 0;
  static uint32_t bat_critical_counter = 0;
  static bool_t vsupply_check_started = FALSE;

// mAh = mA * dt (10Hz -> hours)
  electrical.energy += ((float)electrical.current) / 3600.0f / ELECTRICAL_PERIODIC_FREQ;

  /*if valid voltage is seen then start checking. Set min level to 0 to always start*/
  if (electrical.vsupply >= MIN_BAT_LEVEL * 10) {
    vsupply_check_started = TRUE;
  }

  if (vsupply_check_started) {
    if (electrical.vsupply < LOW_BAT_LEVEL * 10) {
      if (bat_low_counter > 0) {
        bat_low_counter--;
      }
      if (bat_low_counter == 0) {
        electrical.bat_low = TRUE;
      }
    } else {
      // reset battery low status and counter
      bat_low_counter = BAT_CHECKER_DELAY * ELECTRICAL_PERIODIC_FREQ;
      electrical.bat_low = FALSE;
    }

    if (electrical.vsupply < CRITIC_BAT_LEVEL * 10) {
      if (bat_critical_counter > 0) {
        bat_critical_counter--;
      }
      if (bat_critical_counter == 0) {
        electrical.bat_critical = TRUE;
      }
    } else {
      // reset battery critical status and counter
      bat_critical_counter = BAT_CHECKER_DELAY * ELECTRICAL_PERIODIC_FREQ;
      electrical.bat_critical = FALSE;
    }
  }


}

#endif /* SUBSYSTEMS_ELECTRICAL_H */

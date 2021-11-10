/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/fixedwing/autopilot_rc_helpers.h
 *
 * Some helper functions to check RC sticks.
 */

#ifndef AUTOPILOT_RC_HELPERS_H
#define AUTOPILOT_RC_HELPERS_H

#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"

/** RC mode switch position helper
 *  switch positions threshold are evenly spaced
 *
 *  @param[in] chan RC mode channel number
 *  @param[in] pos switch position to be tested
 *  @param[in] max maximum number of position of the switch
 *  @return true if current position is the same as requested (and RC status is OK)
 */
static inline bool rc_mode_switch(uint8_t chan, uint8_t pos, uint8_t max)
{
  if (radio_control.status != RC_OK) return false;
  if (pos >= max) return false;
  int32_t v = (int32_t)radio_control.values[chan] - MIN_PPRZ;
  // round final value
  int32_t p = (((((int32_t)max - 1) * 10 * v) / (MAX_PPRZ - MIN_PPRZ)) + 5) / 10;
  Bound(p, 0, max - 1); // just in case
  return pos == (uint8_t)p;
}

/** Convenience macro for 3way switch
 */
#ifdef RADIO_MODE
#define RCMode0() rc_mode_switch(RADIO_MODE, 0, 3)
#define RCMode1() rc_mode_switch(RADIO_MODE, 1, 3)
#define RCMode2() rc_mode_switch(RADIO_MODE, 2, 3)
#endif


#endif /* AUTOPILOT_RC_HELPERS_H */

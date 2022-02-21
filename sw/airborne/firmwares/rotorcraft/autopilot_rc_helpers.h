/*
 * Copyright (C) 2012 The Paparazzi Team
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
 * @file firmwares/rotorcraft/autopilot_rc_helpers.h
 *
 * Some helper functions to check RC sticks.
 */

#ifndef AUTOPILOT_RC_HELPERS_H
#define AUTOPILOT_RC_HELPERS_H

#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "generated/modules.h"

#define AUTOPILOT_THROTTLE_THRESHOLD      (MAX_PPRZ / 20)
#define AUTOPILOT_YAW_THRESHOLD           (MAX_PPRZ * 19 / 20)
#ifndef AUTOPILOT_STICK_CENTER_THRESHOLD
#define AUTOPILOT_STICK_CENTER_THRESHOLD  (MAX_PPRZ * 1 / 20)
#endif

#define THROTTLE_STICK_DOWN()                                           \
  (radio_control.values[RADIO_THROTTLE] < AUTOPILOT_THROTTLE_THRESHOLD)
#define YAW_STICK_PUSHED()                                      \
  (radio_control.values[RADIO_YAW] > AUTOPILOT_YAW_THRESHOLD ||  \
   radio_control.values[RADIO_YAW] < -AUTOPILOT_YAW_THRESHOLD)
#define YAW_STICK_CENTERED()                                            \
  (radio_control.values[RADIO_YAW] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_YAW] > -AUTOPILOT_STICK_CENTER_THRESHOLD)
#define PITCH_STICK_CENTERED()                                          \
  (radio_control.values[RADIO_PITCH] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_PITCH] > -AUTOPILOT_STICK_CENTER_THRESHOLD)
#define ROLL_STICK_CENTERED()                                           \
  (radio_control.values[RADIO_ROLL] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_ROLL] > -AUTOPILOT_STICK_CENTER_THRESHOLD)

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

static inline bool rc_attitude_sticks_centered(void)
{
  return ROLL_STICK_CENTERED() && PITCH_STICK_CENTERED() && YAW_STICK_CENTERED();
}

#ifdef RADIO_KILL_SWITCH
static inline bool kill_switch_is_on(void)
{
  if (radio_control.values[RADIO_KILL_SWITCH] < 0) {
    return true;
  } else {
    return false;
  }
}
#else
static inline bool kill_switch_is_on(void)
{
  return false;
}
#endif

static inline uint8_t percent_from_rc(int channel)
{
  int per = (MAX_PPRZ + (int32_t)radio_control.values[channel]) * 50 / MAX_PPRZ;
  if (per < 0) {
    per = 0;
  } else if (per > 100) {
    per = 100;
  }
  return per;
}


#endif /* AUTOPILOT_RC_HELPERS_H */

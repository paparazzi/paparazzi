/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * @file arch/sim/modules/radio_control/ppm_arch.c
 *
 * PPM radio control, simulator specific.
 *
 */

#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/ppm.h"
#include "nps_radio_control.h"
#include <inttypes.h>


void ppm_arch_init(void)
{
}

#ifdef RADIO_CONTROL
#define PPM_OF_NPS(_nps, _neutral, _min, _max)                          \
  ((_nps) >= 0 ? (_neutral) + (_nps) * ((_max)-(_neutral)) : (_neutral) + (_nps) * ((_neutral)- (_min)))

void radio_control_feed(void)
{
  ppm_pulses[RADIO_ROLL]     = PPM_OF_NPS(nps_radio_control.roll,       \
                                          RADIO_ROLL_NEUTRAL,          \
                                          RADIO_ROLL_MIN,              \
                                          RADIO_ROLL_MAX);
  ppm_pulses[RADIO_PITCH]    = PPM_OF_NPS(nps_radio_control.pitch,      \
                                          RADIO_PITCH_NEUTRAL,         \
                                          RADIO_PITCH_MIN,             \
                                          RADIO_PITCH_MAX);
  ppm_pulses[RADIO_YAW]      = PPM_OF_NPS(nps_radio_control.yaw,        \
                                          RADIO_YAW_NEUTRAL,           \
                                          RADIO_YAW_MIN,               \
                                          RADIO_YAW_MAX);
  ppm_pulses[RADIO_THROTTLE] = PPM_OF_NPS(nps_radio_control.throttle,   \
                                          RADIO_THROTTLE_NEUTRAL,      \
                                          RADIO_THROTTLE_MIN,          \
                                          RADIO_THROTTLE_MAX);
  ppm_pulses[RADIO_MODE]     = PPM_OF_NPS(nps_radio_control.mode,       \
                                          RADIO_MODE_NEUTRAL,          \
                                          RADIO_MODE_MIN,              \
                                          RADIO_MODE_MAX);
  ppm_frame_available = true;
}
#else //RADIO_CONTROL
void radio_control_feed(void) {}
#endif //RADIO_CONTROL


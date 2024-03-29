/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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
 * @file arch/sim/modules/radio_control/spektrum_arch.c
 *
 * Simulator implementation for spektrum radio control.
 *
 */

#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/spektrum.h"
#include "modules/core/abi.h"
#include "std.h"
#include <inttypes.h>

#include "nps_radio_control.h"

static bool spektrum_available;

void spektrum_init(void)
{
  spektrum_available = false;
}

void spektrum_event(void)
{
  if (spektrum_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    radio_control.status = RC_OK;
    AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_SPEKTRUM_ID, &radio_control);
  }
  spektrum_available = false;
}

void spektrum_try_bind(void) {}

#ifdef RADIO_CONTROL
void radio_control_feed(void)
{
  radio_control.values[RADIO_ROLL]     = nps_radio_control.roll * MAX_PPRZ;
  radio_control.values[RADIO_PITCH]    = nps_radio_control.pitch * MAX_PPRZ;
  radio_control.values[RADIO_YAW]      = nps_radio_control.yaw * MAX_PPRZ;
  radio_control.values[RADIO_THROTTLE] = nps_radio_control.throttle * MAX_PPRZ;
  radio_control.values[RADIO_MODE]     = nps_radio_control.mode * MAX_PPRZ;
  spektrum_available = true;
}
#else //RADIO_CONTROL
void radio_control_feed(void) {}
#endif //RADIO_CONTROL


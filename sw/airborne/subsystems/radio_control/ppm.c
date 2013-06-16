/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"

uint16_t ppm_pulses[ PPM_NB_CHANNEL ];
volatile bool_t ppm_frame_available;

#if DOWNLINK
#ifdef FBW
#define DOWNLINK_TELEMETRY &telemetry_Fbw
#else
#define DOWNLINK_TELEMETRY DefaultPeriodic
#endif

#include "subsystems/datalink/telemetry.h"

static void send_ppm(void) {
  uint16_t ppm_pulses_usec[RADIO_CONTROL_NB_CHANNEL];
  for (int i=0;i<RADIO_CONTROL_NB_CHANNEL;i++)
    ppm_pulses_usec[i] = USEC_OF_RC_PPM_TICKS(ppm_pulses[i]);
  DOWNLINK_SEND_PPM(DefaultChannel, DefaultDevice,
      &radio_control.frame_rate, PPM_NB_CHANNEL, ppm_pulses_usec);
}
#endif

void radio_control_impl_init(void) {
  ppm_frame_available = FALSE;
  ppm_arch_init();

#if DOWNLINK
  register_periodic_telemetry(DOWNLINK_TELEMETRY, "PPM", send_ppm);
#endif
}

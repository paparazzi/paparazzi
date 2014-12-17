/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file subsystems/radio_control/superbitrf_rc.c
 * DSM2 and DSMX radio control implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#include "superbitrf_rc.h"
#include "subsystems/radio_control.h"

/**
 * Initialization
 */
//#if DATALINK == SUPERBITRF
//void radio_control_impl_init(void) {}
//#else
void radio_control_impl_init(void)
{
  superbitrf_init();
}
//#endif

/** normalize superbitrf rc_values to radio values */
static void superbitrf_rc_normalize(int16_t *in, int16_t *out, uint8_t count)
{
  uint8_t i;
  for (i = 0; i < count; i++) {
    if (i == RADIO_THROTTLE) {
      out[i] = (in[i] + MAX_PPRZ) / 2;
      Bound(out[i], 0, MAX_PPRZ);
    } else {
      out[i] = -in[i];
      Bound(out[i], MIN_PPRZ, MAX_PPRZ);
    }
  }
}

void radio_control_impl_event(void (* _received_frame_handler)(void))
{
  cyrf6936_event(&superbitrf.cyrf6936);
  superbitrf_event();
  if (superbitrf.rc_frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    radio_control.radio_ok_cpt = 0;
    radio_control.status = RC_OK;
    superbitrf_rc_normalize(superbitrf.rc_values, radio_control.values,
                            superbitrf.num_channels);
    _received_frame_handler();
    superbitrf.rc_frame_available = FALSE;
  }
}

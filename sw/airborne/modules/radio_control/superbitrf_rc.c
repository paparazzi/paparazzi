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
 * @file modules/radio_control/superbitrf_rc.c
 * DSM2 and DSMX radio control implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#include "superbitrf_rc.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"

#if RADIO_CONTROL_NB_CHANNEL < SUPERBITRF_RC_NB_CHANNEL
#error "RADIO_CONTROL_NB_CHANNEL mustn't be lower than 14. X-Plus channel expansion is not (yet) usable"
#endif

/**
 * Initialization
 */
void superbitrf_rc_init(void)
{
  superbitrf_init();
  radio_control.nb_channel = SUPERBITRF_RC_NB_CHANNEL;
}

/** normalize superbitrf rc_values to radio values */
static void superbitrf_rc_normalize(int16_t *in, int16_t *out, uint8_t count)
{
  uint8_t i;
  for (i = 0; i < count; i++) {
    if (i == RADIO_THROTTLE) {
      out[i] = (in[i] + MAX_PPRZ) / 2;
      Bound(out[i], 0, MAX_PPRZ);
    } else {
      out[i] = in[i];
      Bound(out[i], MIN_PPRZ, MAX_PPRZ);
    }
  }
}

void superbitrf_rc_event(void)
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
    AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_SUPERBITRF_RC_ID, &radio_control);
    superbitrf.rc_frame_available = false;
  }
}

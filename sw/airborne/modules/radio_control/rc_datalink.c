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
 * @file modules/radio_control/rc_datalink.c
 *
 * Radio control input via datalink.
 */

#include "modules/radio_control/rc_datalink.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"
#include "pprzlink/dl_protocol.h"

int8_t rc_dl_values[ RC_DL_NB_CHANNEL ];
volatile bool rc_dl_frame_available;


void rc_datalink_init(void)
{
  radio_control.nb_channel = RC_DL_NB_CHANNEL;
  rc_dl_frame_available = false;
}

void rc_datalink_parse_RC_UP(uint8_t *buf)
{
#ifdef RADIO_CONTROL_DATALINK_LED
  LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
  parse_rc_up_datalink(DL_RC_UP_channels_length(buf),
      DL_RC_UP_channels(buf));
}

void parse_rc_up_datalink(
  int8_t n,
  int8_t *channels)
{
  for (int i = 0; i < n; i++) {
    rc_dl_values[i] = channels[i];
  }
  rc_dl_frame_available = true;
}

/**
 * Normalize rc_dl_values to radio values.
 */
static void rc_datalink_normalize(int8_t *in, int16_t *out)
{
  for (int i = 0; i < RC_DL_NB_CHANNEL; i++) {
    out[i] = (MAX_PPRZ / 128) * in[i];
    if (i == RADIO_THROTTLE) {
      Bound(out[i], 0, MAX_PPRZ);
    } else {
      Bound(out[i], MIN_PPRZ, MAX_PPRZ);
    }
  }
}

void rc_datalink_event(void)
{
  if (rc_dl_frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    radio_control.radio_ok_cpt = 0;
    radio_control.status = RC_OK;
    rc_datalink_normalize(rc_dl_values, radio_control.values);
    AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_DATALINK_ID, &radio_control);
    rc_dl_frame_available = false;
  }
}

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

void rc_datalink_parse_RC_3CH(uint8_t *buf)
{
#ifdef RADIO_CONTROL_DATALINK_LED
  LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
  parse_rc_3ch_datalink(
      DL_RC_3CH_throttle_mode(buf),
      DL_RC_3CH_roll(buf),
      DL_RC_3CH_pitch(buf));
}

void rc_datalink_parse_RC_4CH(uint8_t *buf)
{
#ifdef RADIO_CONTROL_DATALINK_LED
  LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
  parse_rc_4ch_datalink(DL_RC_4CH_mode(buf),
      DL_RC_4CH_throttle(buf),
      DL_RC_4CH_roll(buf),
      DL_RC_4CH_pitch(buf),
      DL_RC_4CH_yaw(buf));
}

void parse_rc_3ch_datalink(uint8_t throttle_mode,
                           int8_t roll,
                           int8_t pitch)
{
  uint8_t throttle = ((throttle_mode & 0xFC) >> 2) * (128 / 64);
  uint8_t mode = throttle_mode & 0x03;

  rc_dl_values[RADIO_ROLL] = roll;
  rc_dl_values[RADIO_PITCH] = pitch;
  rc_dl_values[RADIO_THROTTLE] = (int8_t)throttle;
  rc_dl_values[RADIO_MODE] = (int8_t)mode;
  rc_dl_values[RADIO_YAW] = 0;

  rc_dl_frame_available = true;
}

void parse_rc_4ch_datalink(
  uint8_t mode,
  uint8_t throttle,
  int8_t roll,
  int8_t pitch,
  int8_t yaw)
{
  rc_dl_values[RADIO_MODE] = (int8_t)mode;
  rc_dl_values[RADIO_THROTTLE] = (int8_t)throttle;
  rc_dl_values[RADIO_ROLL] = roll;
  rc_dl_values[RADIO_PITCH] = pitch;
  rc_dl_values[RADIO_YAW] = yaw;

  rc_dl_frame_available = true;
}

/**
 * Normalize rc_dl_values to radio values.
 */
static void rc_datalink_normalize(int8_t *in, int16_t *out)
{
  out[RADIO_ROLL] = (MAX_PPRZ / 128) * in[RADIO_ROLL];
  Bound(out[RADIO_ROLL], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_PITCH] = (MAX_PPRZ / 128) * in[RADIO_PITCH];
  Bound(out[RADIO_PITCH], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_YAW] = (MAX_PPRZ / 128) * in[RADIO_YAW];
  Bound(out[RADIO_YAW], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_THROTTLE] = ((MAX_PPRZ / 128) * in[RADIO_THROTTLE]);
  Bound(out[RADIO_THROTTLE], 0, MAX_PPRZ);
  out[RADIO_MODE] = MAX_PPRZ * (in[RADIO_MODE] - 1);
  Bound(out[RADIO_MODE], MIN_PPRZ, MAX_PPRZ);
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

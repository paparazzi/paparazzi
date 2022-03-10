/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/radio_control/rc_intermcu.c
 *
 * Radio control input via intermcu.
 */

#include "modules/radio_control/rc_intermcu.h"
#include "modules/radio_control/radio_control.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/core/abi.h"

struct _rc_intermcu {
  uint16_t values[RC_IMCU_NB_CHANNEL];
  uint8_t status;
  uint8_t frame_rate;
};

static struct _rc_intermcu rc_intermcu;

void rc_intermcu_init(void)
{
  radio_control.nb_channel = RC_IMCU_NB_CHANNEL;
  rc_intermcu.status = RC_REALLY_LOST;
  rc_intermcu.frame_rate = 0;
  for (int i = 0; i < RC_IMCU_NB_CHANNEL; i++) {
    rc_intermcu.values[i] = 0;
  }
}


void rc_intermcu_parse_msg(uint8_t *buf)
{
  uint8_t size = DL_IMCU_RADIO_COMMANDS_values_length(buf);
  //rc_intermcu.status = DL_IMCU_RADIO_COMMANDS_status(imcu_msg_buf); TODO change status on FBW side to have RC status here
  for (uint8_t i = 0; i < size; i++) {
    rc_intermcu.values[i] = DL_IMCU_RADIO_COMMANDS_values(buf)[i];
    radio_control.values[i] = rc_intermcu.values[i]; // for now a simple copy
  }

  radio_control.frame_cpt++;
  radio_control.time_since_last_frame = 0;
  radio_control.radio_ok_cpt = 0;
  radio_control.status = RC_OK;
  AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_INTERMCU_ID, &radio_control);
}

void rc_intermcu_parse_fbw_status(uint8_t *buf)
{
  // update status and frame rate, send message for mode update
  rc_intermcu.status = DL_IMCU_FBW_STATUS_rc_status(buf);
  rc_intermcu.frame_rate = DL_IMCU_FBW_STATUS_frame_rate(buf);
  AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_INTERMCU_ID, &radio_control);
}


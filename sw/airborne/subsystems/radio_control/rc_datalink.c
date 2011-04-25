/*
 * $Id$
 *
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

#include "subsystems/radio_control/rc_datalink.h"
#include "subsystems/radio_control.h"

int8_t rc_dl_values[ RC_DL_NB_CHANNEL ];
volatile bool_t rc_dl_frame_available;


void radio_control_impl_init(void) {
  rc_dl_frame_available = FALSE;
}


void parse_rc_3ch_datalink( uint8_t throttle_mode,
                        int8_t roll,
                        int8_t pitch)
{
  uint8_t throttle = ((throttle_mode & 0xFC)>>2)*(128/64);
  uint8_t mode = throttle_mode & 0x03;

  rc_dl_values[RADIO_ROLL] = roll;
  rc_dl_values[RADIO_PITCH] = pitch;
  rc_dl_values[RADIO_THROTTLE] = (int8_t)throttle;
  rc_dl_values[RADIO_MODE] = (int8_t)mode;

  rc_dl_frame_available = TRUE;
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

  rc_dl_frame_available = TRUE;
}


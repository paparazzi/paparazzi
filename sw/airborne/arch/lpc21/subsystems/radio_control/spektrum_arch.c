/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
#include "subsystems/radio_control/spektrum_arch.h"

bool_t   rc_spk_parser_status;
uint8_t  rc_spk_parser_idx;
uint8_t  rc_spk_parser_buf[RADIO_CONTROL_NB_CHANNEL * 2];
const int16_t rc_spk_throw[RADIO_CONTROL_NB_CHANNEL] = RC_SPK_THROWS;

void radio_control_impl_init(void)
{
  rc_spk_parser_status = RC_SPK_STA_UNINIT;
  rc_spk_parser_idx = 0;
}

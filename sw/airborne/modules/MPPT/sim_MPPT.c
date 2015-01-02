/*
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz
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
 *
 */


#include "MPPT.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"



uint8_t MPPT_mode;

static int16_t MPPT_data[NB_DATA];

void MPPT_init(void)
{
  uint8_t i = 0;

  for (i = 0; i < NB_DATA; i++) {
    MPPT_data[i] = 42 + i;
  }
}

void MPPT_periodic(void)
{
  MPPT_data[MPPT_ITOTAL_INDEX] = MPPT_data[MPPT_IBAT_INDEX] + MPPT_data[MPPT_ICONV_INDEX];

  RunOnceEvery(8, DOWNLINK_SEND_MPPT(DefaultChannel, DefaultDevice, NB_DATA, MPPT_data));
}

/*  $Id$
 *
 * (c) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef LINK_AUTOPILOT_H
#define LINK_AUTOPILOT_H

#include <inttypes.h>

#include "std.h"
#include "radio.h"
#include "airframe.h"

/*
 * System clock in MHz.
 */
#define CLOCK		16

typedef int16_t pprz_t; // type of commands

/* !!!!!!!!!!!!!!!!!!! Value used in gen_airframe.ml !!!!!!!!!!!!!!!!! */
#define MAX_PPRZ (600 * CLOCK)
#define MIN_PPRZ -MAX_PPRZ

struct inter_mcu_msg {
  int16_t channels[RADIO_CTL_NB];  
  uint8_t ppm_cpt;
  uint8_t status;
  uint8_t nb_err;
  uint8_t vsupply; /* 1e-1 V */
};

// Status bits from FBW to AUTOPILOT
#define STATUS_RADIO_OK 0
#define RADIO_REALLY_LOST 1
#define STATUS_MODE_AUTO 2
#define STATUS_MODE_FAILSAFE 3
#define AVERAGED_CHANNELS_SENT 4
#define MASK_FBW_CHANGED 0xf

// Statut bits from AUTOPILOT to FBW
#define STATUS_AUTO_OK  0

#define FRAME_LENGTH (sizeof(struct inter_mcu_msg)+1)

#define TRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)

#endif // LINK_AUTOPILOT_H

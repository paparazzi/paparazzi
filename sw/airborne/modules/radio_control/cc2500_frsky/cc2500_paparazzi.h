/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
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

#ifndef RADIO_CONTROL_CC2500_PAPARAZZI_H
#define RADIO_CONTROL_CC2500_PAPARAZZI_H

#define RC_PPM_TICKS_OF_USEC(_x) (_x)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_x) (_x)
#define USEC_OF_RC_PPM_TICKS(_x) (_x)

#include "generated/airframe.h"
#include "generated/radio.h"

#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL RADIO_CTL_NB
#endif

extern void radio_control_cc2500_init(void);
extern void radio_control_cc2500_event(void);

#endif // RADIO_CONTROL_CC2500_PAPARAZZI_H


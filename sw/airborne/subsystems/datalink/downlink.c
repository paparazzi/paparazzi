/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

/** \file downlink.c
 *  \brief Common code for AP and FBW telemetry
 *
 */


#include "subsystems/datalink/downlink.h"

struct downlink downlink;

void downlink_init(void)
{
  downlink.nb_ovrn = 0;
  downlink.nb_bytes = 0;
  downlink.nb_msgs = 0;

#if defined DATALINK
#if DATALINK == PPRZ
  pprz_transport_init();
#endif
#if DATALINK == XBEE
  xbee_init();
#endif
#if DATALINK == W5100
  w5100_init();
#endif
#endif

#if SITL
  ivy_transport_init();
#endif

}


/*
 * Paparazzi $Id: pprz_transport.h 4870 2010-04-24 03:02:39Z poine $
 *
 * Copyright (C) 2010  ENAC
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

/** \file mavlink_transport.h
 *  \Using MAVLINK protocol
 *
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include "subsystems/datalink/datalink.h"
#include "mavlink_transport.h"

#include "mavlink_downlink.h"

/* Datalink Event */

/*
#define MavlinkDatalinkEvent() {			                  \
  MavlinkCheckAndParse(PPRZ_UART, mavlink_tp); \
  DlCheckAndParse();                                  \
}
*/
#define MavlinkDatalinkEvent() {}



#endif /* MAVLINK_H */


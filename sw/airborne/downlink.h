/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <inttypes.h>
#include "pprz_transport.h"

extern uint8_t downlink_nb_ovrn;

#define __Transport(dev, _x) dev##_x
#define _Transport(dev, _x) __Transport(dev, _x)
#define Transport(_x) _Transport(DOWNLINK_TRANSPORT, _x)

#define DownlinkSizeOf(_x) Transport(SizeOf(_x))

#define DownlinkCheckFreeSpace(_x) Transport(CheckFreeSpace(_x))

#define DownlinkPutUint8(_x) Transport(PutUint8(_x))

#define DownlinkPutInt8ByAddr(_x) Transport(PutInt8ByAddr(_x))
#define DownlinkPutUint8ByAddr(_x) Transport(PutUint8ByAddr(_x))
#define DownlinkPutInt16ByAddr(_x) Transport(PutInt16ByAddr(_x))
#define DownlinkPutUint16ByAddr(_x) Transport(PutUint16ByAddr(_x))
#define DownlinkPutInt32ByAddr(_x) Transport(PutInt32ByAddr(_x))
#define DownlinkPutUint32ByAddr(_x) Transport(PutUint32ByAddr(_x))
#define DownlinkPutFloatByAddr(_x) Transport(PutFloatByAddr(_x))

#define DonwlinkOverrun() downlink_nb_ovrn++;

#define DownlinkStartMessage(msg_id, payload_len) { \
  Transport(Header(payload_len)); \
  Transport(PutUint8(AC_ID)); \
  Transport(PutUint8(msg_id)); \
}

#define DownlinkEndMessage() Transport(Trailer())

#endif /* DOWNLINK_H */

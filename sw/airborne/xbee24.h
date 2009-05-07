/*
 * $Id$
 *  
 * Copyright (C) 2009  ENAC, Pascal Brisset
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

/* Configuration for 2.4GHz "series 1" and 900MHz modules */

#ifndef XBEE24_H
#define XBEE24_H

#define XBEE_TX_ID 0x01 /* 16 bits address */
#define XBEE_RX_ID 0x81 /* 16 bits address */
#define XBEE_RFDATA_OFFSET 5

#define XBeeTransportPutTXHeader() { \
  XBeeTransportPutUint8(XBEE_TX_ID); \
  XBeeTransportPutUint8(NO_FRAME_ID); \
  XBeeTransportPutUint8(GROUND_STATION_ADDR >> 8); \
  XBeeTransportPutUint8(GROUND_STATION_ADDR & 0xff); \
  XBeeTransportPutUint8(TX_OPTIONS); \
}

/* 4 = frame_id + addr_msb + addr_lsb + options */
#define XBeeTransportSizeOf(_x) XBeeAPISizeOf(_x+4)

#define XbeeGetRSSI() { xbee_rssi = xbee_payload[3]; }

#endif // XBEE24_H

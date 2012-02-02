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

/* Configuration for 868MHz modules */

#ifndef XBEE868_H
#define XBEE868_H

#define XBEE_TX_ID 0x10
#define XBEE_RX_ID 0x90
#define XBEE_RFDATA_OFFSET 12

#define XBeeTransportPutTXHeader(_dev)  { \
  XBeeTransportPutUint8(_dev, XBEE_TX_ID); \
  XBeeTransportPutUint8(_dev, NO_FRAME_ID); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, GROUND_STATION_ADDR >> 8); \
  XBeeTransportPutUint8(_dev, GROUND_STATION_ADDR & 0xff); \
  XBeeTransportPutUint8(_dev, 0xff); \
  XBeeTransportPutUint8(_dev, 0xfe); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, TX_OPTIONS); \
}

/* 13 = frame_id + addr==8 + 3 + options */
#define XBeeTransportSizeOf(_dev, _x) XBeeAPISizeOf(_dev, _x+13)

#define XbeeGetRSSI(_payload) {}

#endif // XBEE868_H

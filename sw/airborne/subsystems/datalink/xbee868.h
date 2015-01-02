/*
 * Copyright (C) 2009  ENAC, Pascal Brisset
 * Copyright (C) 2014  Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file subsystems/datalink/xbee868.h
 * Configuration for 868MHz modules
 */

#ifndef XBEE868_H
#define XBEE868_H

#define XBEE_TX_ID 0x10
#define XBEE_RX_ID 0x90
#define XBEE_RFDATA_OFFSET 12

#define XBEE_TX_OVERHEAD 13
#define XBEE_TX_HEADER { \
    XBEE_TX_ID, \
    NO_FRAME_ID, \
    0x00, \
    0x00, \
    0x00, \
    0x00, \
    0x00, \
    0x00, \
    (GROUND_STATION_ADDR >> 8), \
    (GROUND_STATION_ADDR & 0xff), \
    0xff, \
    0xfe, \
    0x00, \
    TX_OPTIONS \
  }

#define XbeeGetRSSI(_payload) {}

#endif // XBEE868_H

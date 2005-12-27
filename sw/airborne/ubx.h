/*
 * Paparazzi autopilot $Id$
 *  
 * Copyright (C) 2004  Pascal Brisset, Antoine Drouin
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

/*
 * UBX protocol specific code
 *
*/


#ifndef UBX_H
#define UBX_H

extern uint8_t send_ck_a, send_ck_b;
#define UbxInitCheksum() { send_ck_a = send_ck_b = 0; }
#define UpdateChecksum(c) { send_ck_a += c; send_ck_b += send_ck_a; }
#define UbxTrailer() { GpsUartSend1(send_ck_a);  GpsUartSend1(send_ck_b); }

#define UbxSend1(c) { uint8_t i8=c; GpsUartSend1(i8); UpdateChecksum(i8); }
#define UbxSend2(c) { uint16_t i16=c; UbxSend1(i16&0xff); UbxSend1(i16 >> 8); }
#define UbxSend1ByAddr(x) { UbxSend1(*x); }
#define UbxSend2ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); }
#define UbxSend4ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); UbxSend1(*(x+2)); UbxSend1(*(x+3)); }

#define UbxHeader(nav_id, msg_id, len) { \
  GpsUartSend1(UBX_SYNC1); \
  GpsUartSend1(UBX_SYNC2); \
  UbxInitCheksum(); \
  UbxSend1(nav_id); \
  UbxSend1(msg_id); \
  UbxSend2(len); \
}


 
#include "ubx_protocol.h"

#define GPS_FIX_VALID(gps_mode) (gps_mode == 3)

extern void parse_ubx( uint8_t c );

#define GpsParse(_gps_buffer, _gps_buffer_size) { \
  uint8_t i; \
  for(i = 0; i < _gps_buffer_size; i++) { \
    parse_ubx(_gps_buffer[i]); \
  } \
}

#endif /* UBX_H */

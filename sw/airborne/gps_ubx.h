/*
 * Paparazzi autopilot $Id$
 *
 * Copyright (C) 2004-2006  Pascal Brisset, Antoine Drouin
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

/** \file gps_ubx.h
 * \brief UBX protocol specific code
 *
*/


#ifndef UBX_H
#define UBX_H

#define GPS_NB_CHANNELS 16

extern uint16_t gps_reset;

extern uint8_t ubx_id, ubx_class;
extern uint16_t ubx_len;
#define UBX_MAX_PAYLOAD 255
extern uint8_t ubx_msg_buf[UBX_MAX_PAYLOAD];

/** The function to be called when a characted friom the device is available */
extern void parse_ubx( uint8_t c );

#define GpsParse(_gps_buffer, _gps_buffer_size) { \
  uint8_t i; \
  for(i = 0; i < _gps_buffer_size; i++) { \
    parse_ubx(_gps_buffer[i]); \
  } \
}

#define GpsFixValid() (gps_mode == 3)

#define CFG_RST_BBR_Hotstart  0x0000
#define CFG_RST_BBR_Warmstart 0x0001
#define CFG_RST_BBR_Coldstart 0xffff

#define CFG_RST_Reset_Hardware 0x00
#define CFG_RST_Reset_Controlled 0x01
#define CFG_RST_Reset_Controlled_GPS_only 0x02
#define CFG_RST_Reset_Controlled_GPS_stop 0x08
#define CFG_RST_Reset_Controlled_GPS_start 0x09

#define NAV_DYN_STATIONARY  1
#define NAV_DYN_PEDESTRIAN  2
#define NAV_DYN_AUTOMOTIVE  3
#define NAV_DYN_SEA         4
#define NAV_DYN_AIRBORNE_1G 5
#define NAV_DYN_AIRBORNE_2G 6
#define NAV_DYN_AIRBORNE_4G 7

#define NAV5_DYN_PORTABLE    0
#define NAV5_DYN_FIXED       1
#define NAV5_DYN_STATIONARY  2
#define NAV5_DYN_PEDESTRIAN  3
#define NAV5_DYN_AUTOMOTIVE  4
#define NAV5_DYN_SEA         5
#define NAV5_DYN_AIRBORNE_1G 6
#define NAV5_DYN_AIRBORNE_2G 7
#define NAV5_DYN_AIRBORNE_4G 8

#define NAV5_2D_ONLY 1
#define NAV5_3D_ONLY 2
#define NAV5_AUTO    3


extern void ubxsend_cfg_rst(uint16_t, uint8_t);

#define gps_ubx_Reset(_val) { \
  gps_reset = _val; \
  if (gps_reset > CFG_RST_BBR_Warmstart) \
    gps_reset = CFG_RST_BBR_Coldstart; \
  ubxsend_cfg_rst(gps_reset, CFG_RST_Reset_Controlled); \
}

#ifdef GPS_TIMESTAMP
uint32_t itow_from_ticks(uint32_t clock_ticks);
#endif

#endif /* UBX_H */

/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

#include "subsystems/gps.h"
#include "subsystems/abi.h"

#include "fms/fms_network.h"
#include <string.h>
#include <stdio.h>

#if GPS_USE_LATLONG
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

//Check if variables are set and else define them
#ifndef GPS_UDP_HOST
#define GPS_UDP_HOST            192.168.1.2
#endif

#define GPS_UDP_MSG_LEN         (11*4)

//Define the buffer, check bytes and FmsNetwork
unsigned char gps_udp_read_buffer[256];
struct FmsNetwork *gps_network = NULL;

void gps_impl_init(void)
{
  gps.fix = GPS_FIX_NONE;
  gps_network = network_new(STRINGIFY(GPS_UDP_HOST), 6000 /*out*/, 7000 /*in*/, TRUE);
}

#define STX 99

#define UDP_GPS_INT(_udp_gps_payload) (int32_t)(*((uint8_t*)_udp_gps_payload)|*((uint8_t*)_udp_gps_payload+1)<<8|((int32_t)*((uint8_t*)_udp_gps_payload+2))<<16|((int32_t)*((uint8_t*)_udp_gps_payload+3))<<24)

void gps_parse(void)
{
  if (gps_network == NULL) { return; }

  //Read from the network
  int size = network_read(gps_network, &gps_udp_read_buffer[0], 256);

  if (size > 0) {
    // Correct MSG
    if ((size == GPS_UDP_MSG_LEN) && (gps_udp_read_buffer[0] == STX)) {
      gps.lla_pos.lat = UDP_GPS_INT(gps_udp_read_buffer + 4);
      gps.lla_pos.lon = UDP_GPS_INT(gps_udp_read_buffer + 8);
      gps.lla_pos.alt = UDP_GPS_INT(gps_udp_read_buffer + 12);
      gps.hmsl        = UDP_GPS_INT(gps_udp_read_buffer + 16);

      gps.ecef_pos.x = UDP_GPS_INT(gps_udp_read_buffer + 20);
      gps.ecef_pos.y = UDP_GPS_INT(gps_udp_read_buffer + 24);
      gps.ecef_pos.z = UDP_GPS_INT(gps_udp_read_buffer + 28);

      gps.ecef_vel.x = UDP_GPS_INT(gps_udp_read_buffer + 32);
      gps.ecef_vel.y = UDP_GPS_INT(gps_udp_read_buffer + 36);
      gps.ecef_vel.z = UDP_GPS_INT(gps_udp_read_buffer + 40);

      gps.fix = GPS_FIX_3D;

#if GPS_USE_LATLONG
      // Computes from (lat, long) in the referenced UTM zone
      struct LlaCoor_f lla_f;
      LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
      struct UtmCoor_f utm_f;
      utm_f.zone = nav_utm_zone0;
      // convert to utm
      utm_of_lla_f(&utm_f, &lla_f);
      // copy results of utm conversion
      gps.utm_pos.east = utm_f.east * 100;
      gps.utm_pos.north = utm_f.north * 100;
      gps.utm_pos.alt = gps.lla_pos.alt;
      gps.utm_pos.zone = nav_utm_zone0;
#endif

      // publish new GPS data
      uint32_t now_ts = get_sys_time_usec();
      gps.last_msg_ticks = sys_time.nb_sec_rem;
      gps.last_msg_time = sys_time.nb_sec;
      if (gps.fix == GPS_FIX_3D) {
        gps.last_3dfix_ticks = sys_time.nb_sec_rem;
        gps.last_3dfix_time = sys_time.nb_sec;
      }
      AbiSendMsgGPS(GPS_UDP_ID, now_ts, &gps);

    } else {
      printf("gps_udp error: msg len invalid %d bytes\n", size);
    }
    memset(&gps_udp_read_buffer[0], 0, sizeof(gps_udp_read_buffer));
  }
}


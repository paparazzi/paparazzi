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

//Check if variables are set and else define them
#ifndef GPS_UDP_HOST
#define GPS_UDP_HOST            192.168.1.2
#endif

#define GPS_UDP_MSG_LEN         (11*4)

//Define the buffer, check bytes and FmsNetwork
unsigned char gps_udp_read_buffer[256];
struct FmsNetwork *gps_network = NULL;

void gps_udp_init(void)
{
  gps_udp.fix = GPS_FIX_NONE;
  gps_network = network_new(STRINGIFY(GPS_UDP_HOST), 6000 /*out*/, 7000 /*in*/, TRUE);
}

#define STX 99

#define UDP_GPS_INT(_udp_gps_payload) (int32_t)(*((uint8_t*)_udp_gps_payload)|*((uint8_t*)_udp_gps_payload+1)<<8|((int32_t)*((uint8_t*)_udp_gps_payload+2))<<16|((int32_t)*((uint8_t*)_udp_gps_payload+3))<<24)

void gps_udp_parse(void)
{
  if (gps_network == NULL) { return; }

  //Read from the network
  int size = network_read(gps_network, &gps_udp_read_buffer[0], 256);

  if (size > 0) {
    // Correct MSG
    if ((size == GPS_UDP_MSG_LEN) && (gps_udp_read_buffer[0] == STX)) {
      gps_udp.lla_pos.lat = UDP_GPS_INT(gps_udp_read_buffer + 4);
      gps_udp.lla_pos.lon = UDP_GPS_INT(gps_udp_read_buffer + 8);
      gps_udp.lla_pos.alt = UDP_GPS_INT(gps_udp_read_buffer + 12);
      SetBit(gps_udp.valid_fields, GPS_VALID_POS_LLA_BIT);

      gps_udp.hmsl        = UDP_GPS_INT(gps_udp_read_buffer + 16);
      SetBit(gps_udp.valid_fields, GPS_VALID_HMSL_BIT);

      gps_udp.ecef_pos.x = UDP_GPS_INT(gps_udp_read_buffer + 20);
      gps_udp.ecef_pos.y = UDP_GPS_INT(gps_udp_read_buffer + 24);
      gps_udp.ecef_pos.z = UDP_GPS_INT(gps_udp_read_buffer + 28);
      SetBit(gps_udp.valid_fields, GPS_VALID_POS_ECEF_BIT);

      gps_udp.ecef_vel.x = UDP_GPS_INT(gps_udp_read_buffer + 32);
      gps_udp.ecef_vel.y = UDP_GPS_INT(gps_udp_read_buffer + 36);
      gps_udp.ecef_vel.z = UDP_GPS_INT(gps_udp_read_buffer + 40);
      SetBit(gps_udp.valid_fields, GPS_VALID_VEL_ECEF_BIT);

      gps_udp.fix = GPS_FIX_3D;

      // publish new GPS data
      uint32_t now_ts = get_sys_time_usec();
      gps_udp.last_msg_ticks = sys_time.nb_sec_rem;
      gps_udp.last_msg_time = sys_time.nb_sec;
      if (gps_udp.fix == GPS_FIX_3D) {
        gps_udp.last_3dfix_ticks = sys_time.nb_sec_rem;
        gps_udp.last_3dfix_time = sys_time.nb_sec;
      }
      AbiSendMsgGPS(GPS_UDP_ID, now_ts, &gps_udp);

    } else {
      printf("gps_udp error: msg len invalid %d bytes\n", size);
    }
    memset(&gps_udp_read_buffer[0], 0, sizeof(gps_udp_read_buffer));
  }
}

/*
 * register callbacks & structs
 */
void gps_udp_register(void)
{
  gps_register_impl(gps_udp_init, gps_udp_parse, GPS_UDP_ID);
}

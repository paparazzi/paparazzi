/*
 * Copyright (C) 2012 Freek van Tienen
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


#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "led.h"

#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "gps_sirf.h"

struct GpsSirf gps_sirf;
void sirf_parse_2(void);
void sirf_parse_41(void);

void sirf_gps_impl_init(void)
{
  gps_sirf.msg_available = FALSE;
  gps_sirf.pos_available = FALSE;
  gps_sirf.msg_len = 0;
  gps_sirf.read_state = 0;
}

void gps_sirf_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  gps_sirf.state.last_msg_ticks = sys_time.nb_sec_rem;
  gps_sirf.state.last_msg_time = sys_time.nb_sec;
  sirf_parse_msg();
  if (gps_sirf.pos_available) {
    if (gps_sirf.state.fix == GPS_FIX_3D) {
      gps_sirf.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_sirf.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_SIRF_ID, now_ts, &gps);
  }
  gps_sirf.msg_available = FALSE;
}

void sirf_parse_char(uint8_t c)
{
  switch (gps_sirf.read_state) {
    case UNINIT:
      if (c == 0xA0) {
        gps_sirf.msg_len = 0;
        gps_sirf.msg_buf[gps_sirf.msg_len] = c;
        gps_sirf.msg_len++;
        gps_sirf.read_state = GOT_A0;
      }
      break;
    case GOT_A0:
      if (c == 0xA2) {
        gps_sirf.msg_buf[gps_sirf.msg_len] = c;
        gps_sirf.msg_len++;
        gps_sirf.read_state = GOT_A2;
      } else {
        goto restart;
      }
      break;
    case GOT_A2:
      gps_sirf.msg_buf[gps_sirf.msg_len] = c;
      gps_sirf.msg_len++;
      if (c == 0xB0) {
        gps_sirf.read_state = GOT_B0;
      }
      break;
    case GOT_B0:
      if (c == 0xB3) {
        gps_sirf.msg_buf[gps_sirf.msg_len] = c;
        gps_sirf.msg_len++;
        gps_sirf.msg_available = TRUE;
      } else {
        goto restart;
      }
      break;
    default:
      break;
  }
  return;

restart:
  gps_sirf.read_state = UNINIT;
}

int start_time = 0;
int ticks = 0;
int start_time2 = 0;
int ticks2 = 0;

void sirf_parse_41(void)
{
  struct sirf_msg_41 *p = (struct sirf_msg_41 *)&gps_sirf.msg_buf[4];

  gps_sirf.state.tow = Invert4Bytes(p->tow);
  gps_sirf.state.hmsl = Invert4Bytes(p->alt_msl) * 10;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_HMSL_BIT);
  gps_sirf.state.num_sv = p->num_sat;
  gps_sirf.state.nb_channels = p ->num_sat;

  /* read latitude, longitude and altitude from packet */
  gps_sirf.state.lla_pos.lat = Invert4Bytes(p->latitude);
  gps_sirf.state.lla_pos.lon = Invert4Bytes(p->longitude);
  gps_sirf.state.lla_pos.alt = Invert4Bytes(p->alt_ellipsoid) * 10;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_POS_LLA_BIT);

  gps_sirf.state.sacc = (Invert2Bytes(p->ehve) >> 16);
  gps_sirf.state.course = RadOfDeg(Invert2Bytes(p->cog)) * pow(10, 5);
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_COURSE_BIT);
  gps_sirf.state.gspeed = RadOfDeg(Invert2Bytes(p->sog)) * pow(10, 5);
  gps_sirf.state.cacc = RadOfDeg(Invert2Bytes(p->heading_err)) * pow(10, 5);
  gps_sirf.state.pacc = Invert4Bytes(p->ehpe);
  gps_sirf.state.pdop = p->hdop * 20;

  if ((p->nav_type >> 8 & 0x7) >= 0x4) {
    gps_sirf.state.fix = GPS_FIX_3D;
  } else if ((p->nav_type >> 8 & 0x7) >= 0x1) {
    gps_sirf.state.fix = GPS_FIX_2D;
  } else {
    gps_sirf.state.fix = GPS_FIX_NONE;
  }


  //Let gps_sirf know we have a position update
  gps_sirf.pos_available = TRUE;
}

void sirf_parse_2(void)
{
  struct sirf_msg_2 *p = (struct sirf_msg_2 *)&gps_sirf.msg_buf[4];

  gps_sirf.state.week = Invert2Bytes(p->week);

  gps_sirf.state.ecef_pos.x = Invert4Bytes(p->x_pos) * 100;
  gps_sirf.state.ecef_pos.y = Invert4Bytes(p->y_pos) * 100;
  gps_sirf.state.ecef_pos.z = Invert4Bytes(p->z_pos) * 100;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_POS_ECEF_BIT);

  gps_sirf.state.ecef_vel.x = (Invert2Bytes(p->vx) >> 16) * 100 / 8;
  gps_sirf.state.ecef_vel.y = (Invert2Bytes(p->vy) >> 16) * 100 / 8;
  gps_sirf.state.ecef_vel.z = (Invert2Bytes(p->vz) >> 16) * 100 / 8;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  if (gps_sirf.state.fix == GPS_FIX_3D) {
    ticks++;
#if DEBUG_SIRF
    printf("GPS %i %i %i %i\n", ticks, (sys_time.nb_sec - start_time), ticks2, (sys_time.nb_sec - start_time2));
#endif
  } else if (sys_time.nb_sec - gps_sirf.state.last_3dfix_time > 10) {
    start_time = sys_time.nb_sec;
    ticks = 0;
  }

}

void sirf_parse_msg(void)
{
  //Set position available to false and check if it is a valid message
  gps_sirf.pos_available = FALSE;
  if (gps_sirf.msg_len < 8) {
    return;
  }

  if (start_time2 == 0) {
    start_time2 = sys_time.nb_sec;
  }
  ticks2++;

  //Check the message id and parse the message
  uint8_t message_id = gps_sirf.msg_buf[4];
  switch (message_id) {
    case 0x29:
      sirf_parse_41();
      break;
    case 0x02:
      sirf_parse_2();
      break;
  }

}

/*
 * register callbacks & structs
 */
void sirf_gps_register(void)
{
#ifdef GPS_SECONDARY_SIRF
  gps_register_impl(sirf_gps_impl_init, sirf_gps_event, GPS_SIRF_ID, 1);
#else
  gps_register_impl(sirf_gps_impl_init, sirf_gps_event, GPS_SIRF_ID, 0);
#endif
}

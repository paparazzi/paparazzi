/*
 *
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

#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#endif
#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "gps_sirf.h"

struct GpsSirf gps_sirf;
void sirf_parse_2(void);
void sirf_parse_41(void);

void gps_impl_init( void ) {
  gps_sirf.msg_available = FALSE;
  gps_sirf.pos_available = FALSE;
  gps_sirf.msg_len = 0;
  gps_sirf.read_state = 0;
}

void sirf_parse_char(uint8_t c) {
	switch(gps_sirf.read_state) {
	case UNINIT:
		if(c == 0xA0) {
			gps_sirf.msg_len = 0;
			gps_sirf.msg_buf[gps_sirf.msg_len] = c;
			gps_sirf.msg_len++;
			gps_sirf.read_state = GOT_A0;
		}
		break;
	case GOT_A0:
		if(c == 0xA2) {
			gps_sirf.msg_buf[gps_sirf.msg_len] = c;
			gps_sirf.msg_len++;
			gps_sirf.read_state = GOT_A2;
		}
		else
			goto restart;
		break;
	case GOT_A2:
		gps_sirf.msg_buf[gps_sirf.msg_len] = c;
		gps_sirf.msg_len++;
		if(c == 0xB0)
			gps_sirf.read_state = GOT_B0;
		break;
	case GOT_B0:
		if(c == 0xB3) {
			gps_sirf.msg_buf[gps_sirf.msg_len] = c;
			gps_sirf.msg_len++;
			gps_sirf.msg_available = TRUE;
		}
		else
			goto restart;
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

void sirf_parse_41(void) {
	struct sirf_msg_41* p = (struct sirf_msg_41*)&gps_sirf.msg_buf[4];

	gps.tow = Invert4Bytes(p->tow);
	gps.hmsl = Invert4Bytes(p->alt_msl)*10;
	gps.num_sv = p->num_sat;
	gps.nb_channels = p ->num_sat;

	/* read latitude, longitude and altitude from packet */
	gps.lla_pos.lat = RadOfDeg(Invert4Bytes(p->latitude));
	gps.lla_pos.lon = RadOfDeg(Invert4Bytes(p->longitude));
	gps.lla_pos.alt = Invert4Bytes(p->alt_ellipsoid) * 10;

	#if GPS_USE_LATLONG
	  /* convert to utm */
	  struct UtmCoor_f utm_f;
	  utm_f.zone = nav_utm_zone0;
	  utm_of_lla_f(&utm_f, &lla_pos);

	  /* copy results of utm conversion */
	  gps.utm_pos.east = utm_f.east*100;
	  gps.utm_pos.north = utm_f.north*100;
	  gps.utm_pos.alt = gps.lla_pos.alt;
	  gps.utm_pos.zone = nav_utm_zone0;
	#endif

	gps.sacc = (Invert2Bytes(p->ehve)>>16);
	gps.course = RadOfDeg(Invert2Bytes(p->cog))*pow(10, 5);
	gps.gspeed = RadOfDeg(Invert2Bytes(p->sog))*pow(10, 5);
	gps.cacc = RadOfDeg(Invert2Bytes(p->heading_err))*pow(10, 5);
	gps.pacc = Invert4Bytes(p->ehpe);
	gps.pdop = p->hdop * 20;

	if ((p->nav_type >> 8 & 0x7) >= 0x4)
		gps.fix = GPS_FIX_3D;
	else if((p->nav_type >> 8 & 0x7) >= 0x1)
		gps.fix = GPS_FIX_2D;
	else
		gps.fix = GPS_FIX_NONE;


	//Let gps_sirf know we have a position update
	gps_sirf.pos_available = TRUE;
}

void sirf_parse_2(void) {
	struct sirf_msg_2* p = (struct sirf_msg_2*)&gps_sirf.msg_buf[4];

	gps.week = Invert2Bytes(p->week);

	gps.ecef_pos.x = Invert4Bytes(p->x_pos) * 100;
	gps.ecef_pos.y = Invert4Bytes(p->y_pos) * 100;
	gps.ecef_pos.z = Invert4Bytes(p->z_pos) * 100;

	gps.ecef_vel.x = (Invert2Bytes(p->vx)>>16)*100/8;
	gps.ecef_vel.y = (Invert2Bytes(p->vy)>>16)*100/8;
	gps.ecef_vel.z = (Invert2Bytes(p->vz)>>16)*100/8;

	if(gps.fix == GPS_FIX_3D) {
		ticks++;
		printf("GPS %d %d %d %d\n", ticks, (sys_time.nb_sec - start_time), ticks2, (sys_time.nb_sec - start_time2));
	}
	else if(sys_time.nb_sec - gps.last_fix_time > 10) {
		start_time = sys_time.nb_sec;
		ticks = 0;
	}

	//struct EnuCoor_f enu; /* speed NED in cm/s */
	/*enu_of_ecef_point_f(&enu, def, ecef);
	ENU_OF_TO_NED(*ned, enu);*/
}

void sirf_parse_msg(void) {
	//Set position available to false and check if it is a valid message
	gps_sirf.pos_available = FALSE;
	if(gps_sirf.msg_len < 8)
		return;

	if(start_time2 == 0)
		start_time2 = sys_time.nb_sec;
	ticks2++;

	//Check the message id and parse the message
	uint8_t message_id = gps_sirf.msg_buf[4];
	switch(message_id) {
	case 0x29:
		sirf_parse_41();
		break;
	case 0x02:
		sirf_parse_2();
		break;
	}

	//FAKE
	/*gps.fix = GPS_FIX_3D;
	gps.lla_pos.lat = RadOfDeg(520013040);
	gps.lla_pos.lon = RadOfDeg(43718690);
	gps.lla_pos.alt = 0;
	ecef_of_lla_i(&gps.ecef_pos, &gps.lla_pos);
	//gps.ecef_pos.x = Invert4Bytes(p->x_pos) * 100;
	//gps.ecef_pos.y = Invert4Bytes(p->y_pos) * 100;
	//gps.ecef_pos.z = Invert4Bytes(p->z_pos) * 100;

	gps.ecef_vel.x = 0;
	gps.ecef_vel.y = 0;
	gps.ecef_vel.z = 0;
	gps_sirf.pos_available = TRUE;*/
}

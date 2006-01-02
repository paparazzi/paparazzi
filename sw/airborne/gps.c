/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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

/** \file gps.c
 *  \brief GPS hardware independent handling
 *
 */

#include "autopilot.h"
#include "gps.h"
#include "sys_time.h"
#include "flight_plan.h"
#include "estimator.h"
#include "downlink.h"
#include "infrared.h"

uint16_t last_gps_msg_t;	/** cputime of the last gps message */

void estimator_update_state_gps( void ) {
  float gps_east = gps_utm_east / 100. - NAV_UTM_EAST0;
  float gps_north = gps_utm_north / 100. - NAV_UTM_NORTH0;
  float falt = gps_alt / 100.;
  EstimatorSetPos(gps_east, gps_north, falt);
  float fspeed = gps_gspeed / 100.;
  float fclimb = gps_climb / 100.;
  float fcourse = RadOfDeg(gps_course / 10.);
  EstimatorSetSpeedPol(fspeed, fcourse, fclimb);
  
#ifdef INFRARED
  if (estimator_flight_time)
    estimator_update_ir_estim();
#endif
}

/**Send by downlink the GPS and rad_of_ir messages with \a DOWNLINK_SEND_GPS
 * and \a DOWNLINK_SEND_RAD_OF_IR \n
 * If \a estimator_flight_time is null and \a estimator_hspeed_mod is greater
 * than \a MIN_SPEED_FOR_TAKEOFF, set the \a estimator_flight_time to 1 and \a
 * launch to true (which is not set in non auto launch. Then call
 * \a DOWNLINK_SEND_TAKEOFF
 */
void use_gps_pos( void ) {
  DOWNLINK_SEND_GPS(&gps_mode, &gps_utm_east, &gps_utm_north, &gps_course, &gps_alt, &gps_gspeed,&gps_climb, &gps_itow, &gps_utm_zone, &gps_nb_ovrn);
  if (GPS_FIX_VALID(gps_mode)) {
    last_gps_msg_t = cpu_time;
    estimator_update_state_gps();
  }
  
  static uint8_t i;
  if (i == gps_nb_channels) i = 0;
  if (i < gps_nb_channels && gps_svinfos[i].cno > 0) 
    DOWNLINK_SEND_SVINFO(&i, &gps_svinfos[i].svid, &gps_svinfos[i].flags, &gps_svinfos[i].qi, &gps_svinfos[i].cno, &gps_svinfos[i].elev, &gps_svinfos[i].azim);
  i++;
}

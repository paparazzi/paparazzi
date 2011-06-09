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

#include <stdlib.h>

#include "gps.h"
#include "latlong.h"
#ifndef FMS_PERIODIC_FREQ
#include "sys_time.h"
#endif
#include "generated/airframe.h"
#include "generated/periodic.h"

#ifdef USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif


/** For backward compatibility */
#ifndef DOWNLINK_GPS_DEVICE
#define DOWNLINK_GPS_DEVICE DOWNLINK_AP_DEVICE
#endif



#ifdef FMS_PERIODIC_FREQ
#include "messages2.h"
#include "firmwares/beth/overo_gcs_com.h"
#define DefaultChannel gcs_com.udp_transport
#else
#define DOWNLINK_DEVICE DOWNLINK_GPS_DEVICE
#include "messages.h"
#include "downlink.h"
#endif

uint16_t last_gps_msg_t;	/** cputime of the last gps message */
bool_t gps_verbose_downlink;

#ifdef UGEAR
volatile uint16_t gps_msg_received_counter;
#endif

void gps_downlink( void ) {

#  ifdef DOWNLINK_GPS_1Hz
#  warning "Deperecated DOWNLINK_GPS_1Hz: Please Use Telemetry xml to reduce data rate"
#  endif


#ifndef PERIOD_GPS_DefaultChannel_0
  gps_send();
#endif
}

void gps_send( void ) {

  DOWNLINK_SEND_GPS(DefaultChannel, &gps_mode, &gps_utm_east, &gps_utm_north, &gps_course, &gps_alt, &gps_gspeed,&gps_climb, &gps_week, &gps_itow, &gps_utm_zone, &gps_nb_ovrn);

  static uint8_t i;
  static uint8_t last_cnos[GPS_NB_CHANNELS];
  if (i == gps_nb_channels) i = 0;
  if (i < gps_nb_channels && gps_svinfos[i].cno > 0 && gps_svinfos[i].cno != last_cnos[i]) {
    DOWNLINK_SEND_SVINFO(DefaultChannel, &i, &gps_svinfos[i].svid, &gps_svinfos[i].flags, &gps_svinfos[i].qi, &gps_svinfos[i].cno, &gps_svinfos[i].elev, &gps_svinfos[i].azim);
    last_cnos[i] = gps_svinfos[i].cno;
  }

  if (gps_verbose_downlink) {
    uint8_t j;
    for(j = 0; j < gps_nb_channels; j++) {
      uint8_t cno = gps_svinfos[j].cno;
      if (cno > 0 && j != i && abs(cno-last_cnos[j]) >= 2) {
        DOWNLINK_SEND_SVINFO(DefaultChannel, &j, &gps_svinfos[j].svid, &gps_svinfos[j].flags, &gps_svinfos[j].qi, &cno, &gps_svinfos[j].elev, &gps_svinfos[j].azim);
        last_cnos[j] = gps_svinfos[j].cno;
      }
    }
  }
  i++;
}

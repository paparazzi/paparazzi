/*
 * Copyright (C) 2010 The Paparazzi Team
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


#include "dc.h"

// Variables with boot defaults
uint8_t dc_autoshoot_meter_grid = 100;
uint8_t dc_autoshoot_quartersec_period = 2;
dc_autoshoot_type dc_autoshoot = DC_AUTOSHOOT_STOP;




#ifdef SENSOR_SYNC_SEND

uint16_t dc_photo_nr = 0;

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include "estimator.h"
#include "subsystems/gps.h"

  void dc_send_shot_position(void)
  {
    int16_t phi = DegOfRad(estimator_phi*10.0f);
    int16_t theta = DegOfRad(estimator_theta*10.0f);
    float gps_z = ((float)gps.hmsl) / 1000.0f;
    DOWNLINK_SEND_DC_SHOT(DefaultChannel, &dc_photo_nr, &gps.utm_pos.east, &gps.utm_pos.north, &gps_z, &gps.utm_pos.zone, &phi, &theta,  &gps.course, &gps.gspeed, &gps.tow);
    dc_photo_nr++;
  }

#endif



/*
#ifndef DC_GPS_TRIGGER_START
#define DC_GPS_TRIGGER_START 1
#endif
#ifndef DC_GPS_TRIGGER_STOP
#define DC_GPS_TRIGGER_STOP 3
#endif


static inline void dc_shoot_on_gps( void ) {
  static uint8_t gps_msg_counter = 0;

  if (dc_shoot > 0)
  {

    if (gps_msg_counter == 0)
    {
      DC_PUSH(DC_SHUTTER_LED);

      dc_send_shot_position();
    }
    else if (gps_msg_counter == DC_GPS_TRIGGER_START)
    {
      DC_RELEASE(DC_SHUTTER_LED);
    }

    gps_msg_counter++;
    if (gps_msg_counter >= DC_GPS_TRIGGER_STOP)
      gps_msg_counter = 0;
  }
}
*/


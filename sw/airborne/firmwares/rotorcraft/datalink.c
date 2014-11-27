/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/**
 * @file firmwares/rotorcraft/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define DATALINK_C
#define MODULES_DATALINK_C

#include "subsystems/datalink/datalink.h"

#include "generated/modules.h"

#include "generated/settings.h"
#include "subsystems/datalink/downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "mcu_periph/uart.h"

#ifdef BOOZ_FMS_TYPE
#include "booz_fms.h"
#endif

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#if defined GPS_DATALINK
#include "subsystems/gps/gps_datalink.h"
#endif

#include "firmwares/rotorcraft/navigation.h"

#include "math/pprz_geodetic_int.h"
#include "state.h"
#include "led.h"

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {

  datalink_time = 0;

  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {

  case  DL_PING:
    {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

  case DL_SETTING :
    {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

  case DL_GET_SETTING :
    {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;

#if defined USE_NAVIGATION
  case DL_BLOCK :
    {
      if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) break;
      nav_goto_block(DL_BLOCK_block_id(dl_buffer));
    }
    break;

  case DL_MOVE_WP :
    {
      uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);
      if (ac_id != AC_ID) break;
      if (stateIsLocalCoordinateValid()) {
        uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
        struct LlaCoor_i lla;
        lla.lat = DL_MOVE_WP_lat(dl_buffer);
        lla.lon = DL_MOVE_WP_lon(dl_buffer);
        /* WP_alt from message is alt above MSL in cm
         * lla.alt is above ellipsoid in mm
         */
        lla.alt = DL_MOVE_WP_alt(dl_buffer)*10 - state.ned_origin_i.hmsl +
          state.ned_origin_i.lla.alt;
        nav_move_waypoint_lla(wp_id, &lla);
      }
    }
    break;
#endif /* USE_NAVIGATION */
#ifdef RADIO_CONTROL_TYPE_DATALINK
  case DL_RC_3CH :
#ifdef RADIO_CONTROL_DATALINK_LED
    LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
    parse_rc_3ch_datalink(
        DL_RC_3CH_throttle_mode(dl_buffer),
        DL_RC_3CH_roll(dl_buffer),
        DL_RC_3CH_pitch(dl_buffer));
    break;
  case DL_RC_4CH :
    if (DL_RC_4CH_ac_id(dl_buffer) == AC_ID) {
#ifdef RADIO_CONTROL_DATALINK_LED
      LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
      parse_rc_4ch_datalink(DL_RC_4CH_mode(dl_buffer),
                            DL_RC_4CH_throttle(dl_buffer),
                            DL_RC_4CH_roll(dl_buffer),
                            DL_RC_4CH_pitch(dl_buffer),
                            DL_RC_4CH_yaw(dl_buffer));
    }
    break;
#endif // RADIO_CONTROL_TYPE_DATALINK
#if defined GPS_DATALINK
  case DL_REMOTE_GPS :
    // Check if the GPS is for this AC
    if (DL_REMOTE_GPS_ac_id(dl_buffer) != AC_ID) break;

    // Parse the GPS
    parse_gps_datalink(
      DL_REMOTE_GPS_numsv(dl_buffer),
      DL_REMOTE_GPS_ecef_x(dl_buffer),
      DL_REMOTE_GPS_ecef_y(dl_buffer),
      DL_REMOTE_GPS_ecef_z(dl_buffer),
      DL_REMOTE_GPS_lat(dl_buffer),
      DL_REMOTE_GPS_lon(dl_buffer),
      DL_REMOTE_GPS_alt(dl_buffer),
      DL_REMOTE_GPS_hmsl(dl_buffer),
      DL_REMOTE_GPS_ecef_xd(dl_buffer),
      DL_REMOTE_GPS_ecef_yd(dl_buffer),
      DL_REMOTE_GPS_ecef_zd(dl_buffer),
      DL_REMOTE_GPS_tow(dl_buffer),
      DL_REMOTE_GPS_course(dl_buffer));
    break;
#endif
  default:
    break;
  }
  /* Parse modules datalink */
  modules_parse_datalink(msg_id);
}

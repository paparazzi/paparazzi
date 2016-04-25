/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
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
 * @file subsystems/datalink/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define DATALINK_C
#define MODULES_DATALINK_C

#include "datalink.h"
#include "subsystems/datalink/downlink.h"

#include "generated/modules.h"
#include "generated/settings.h"

#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#if USE_GPS
#include "subsystems/gps.h"
#endif
#if defined GPS_DATALINK
#include "subsystems/gps/gps_datalink.h"
#endif

#ifdef TRAFFIC_INFO
#include "subsystems/navigation/traffic_info.h"
#endif

#ifdef RADIO_CONTROL_DATALINK_LED
#include "led.h"
#endif

#define MOfCm(_x) (((float)(_x))/100.)

#if USE_NPS
bool datalink_enabled = true;
#endif

void dl_parse_msg(void)
{
  uint8_t sender_id = SenderIdOfPprzMsg(dl_buffer);
  uint8_t msg_id = IdOfPprzMsg(dl_buffer);

  /* parse telemetry messages coming from other AC */
  if (sender_id != 0) {
    switch (msg_id) {
#if 0   // TODO waiting for telemetry macros to be made in pprzlink
#ifdef TRAFFIC_INFO
      case DL_GPS_SMALL: {
        uint32_t multiplex_speed = DL_GPS_SMALL_multiplex_speed(dl_buffer);

        // Position in ENU coordinates
        int16_t course = (int16_t)((multiplex_speed >> 21) & 0x7FF); // bits 31-21 course in decideg
        if (course & 0x400) {
          course |= 0xFFFFF800;  // fix for twos complements
        }
        int16_t gspeed = (int16_t)((multiplex_speed >> 10) & 0x7FF); // bits 20-10 ground speed cm/s
        if (gspeed & 0x400) {
          gspeed |= 0xFFFFF800;  // fix for twos complements
        }
        int16_t climb = (int16_t)(multiplex_speed >> 2 & 0x7FF); // bits 9-0 z climb speed in cm/s
        if (climb & 0x400) {
          climb |= 0xFFFFF800;  // fix for twos complements
        }

        set_ac_info(sender_id,
                    MOfCm(DL_GPS_SMALL_utm_east(dl_buffer)),    /*m*/
                    MOfCm(DL_GPS_SMALL_utm_north(dl_buffer)),   /*m*/
                    RadOfDeg(((float)course) / 10.),            /*rad(CW)*/
                    MOfCm(DL_GPS_SMALL_alt(dl_buffer)),         /*m*/
                    MOfCm(gspeed),                              /*m/s*/
                    MOfCm(climb),                               /*m/s*/
                    gps_tow_from_sys_ticks(sys_time.nb_tick));
      }
      break;

      case DL_GPS: {
        set_ac_info(sender_id,
          MOfCm(DL_GPS_utm_east(dl_buffer)),    /*m*/
          MOfCm(DL_GPS_utm_north(dl_buffer)),   /*m*/
          RadOfDeg(((float)DL_GPS_course(dl_buffer)) / 10.), /*rad(CW)*/
          MOfCm(DL_GPS_alt(dl_buffer)),        /*m*/
          MOfCm(DL_GPS_speed(dl_buffer)),       /*m/s*/
          MOfCm(DL_GPS_climb(dl_buffer)),       /*m/s*/
          (uint32_t)DL_GPS_itow(dl_buffer));
      }
      break;
      case DL_GPS_LLA: {
        set_ac_info_lla(sender_id,
                        DL_GPS_LLA_lat(dl_buffer),    /*1e7deg*/
                        DL_GPS_LLA_lon(dl_buffer),    /*1e7deg*/
                        DL_GPS_LLA_alt(dl_buffer),    /*mm*/
                        DL_GPS_LLA_course(dl_buffer), /*decideg*/
                        DL_GPS_LLA_speed(dl_buffer),  /*cm/s*/
                        DL_GPS_LLA_climb(dl_buffer),  /*cm/s*/
                        DL_GPS_LLA_itow(dl_buffer));  /*ms*/
      }
      break;
#endif /* TRAFFIC_INFO */

#ifdef TCAS
      case DL_TCAS_RA: {
        if (DL_TCAS_RESOLVE_ac_id(dl_buffer) == AC_ID && SenderIdOfMsg(dl_buffer) != AC_ID) {
          uint8_t ac_id_conflict = SenderIdOfMsg(dl_buffer);
          tcas_acs_status[the_acs_id[ac_id_conflict]].resolve = DL_TCAS_RA_resolve(dl_buffer);
        }
      }
#endif /* TCAS */
#endif
      default: {
        break;
      }
    }
    return;  // msg was telemetry not datalink so return
  }

  /* parse telemetry messages coming from ground station */
  switch (msg_id) {
    case  DL_PING: {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

    case DL_SETTING : {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

    case DL_GET_SETTING : {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;

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

#if USE_GPS
#ifdef GPS_DATALINK
    case DL_REMOTE_GPS_SMALL : {
      // Check if the GPS is for this AC
      if (DL_REMOTE_GPS_SMALL_ac_id(dl_buffer) != AC_ID) { break; }

      parse_gps_datalink_small(
        DL_REMOTE_GPS_SMALL_heading(dl_buffer),
        DL_REMOTE_GPS_SMALL_pos_xyz(dl_buffer),
        DL_REMOTE_GPS_SMALL_speed_xyz(dl_buffer),
        DL_REMOTE_GPS_SMALL_tow(dl_buffer));
    }
    break;

    case DL_REMOTE_GPS : {
      // Check if the GPS is for this AC
      if (DL_REMOTE_GPS_ac_id(dl_buffer) != AC_ID) { break; }

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
    }
    break;
#endif // GPS_DATALINK

    case DL_GPS_INJECT : {
      // Check if the GPS is for this AC
      if (DL_GPS_INJECT_ac_id(dl_buffer) != AC_ID) { break; }

      // GPS parse data
      gps_inject_data(
        DL_GPS_INJECT_packet_id(dl_buffer),
        DL_GPS_INJECT_data_length(dl_buffer),
        DL_GPS_INJECT_data(dl_buffer)
      );
    }
    break;
#endif  // USE_GPS

#ifdef TRAFFIC_INFO
    case DL_ACINFO: {
      if (DL_ACINFO_ac_id(dl_buffer) == AC_ID) { break; }
      uint8_t id = DL_ACINFO_ac_id(dl_buffer);
      float ux = MOfCm(DL_ACINFO_utm_east(dl_buffer));
      float uy = MOfCm(DL_ACINFO_utm_north(dl_buffer));
      float a = MOfCm(DL_ACINFO_alt(dl_buffer));
      float c = RadOfDeg(((float)DL_ACINFO_course(dl_buffer)) / 10.);
      float s = MOfCm(DL_ACINFO_speed(dl_buffer));
      float cl = MOfCm(DL_ACINFO_climb(dl_buffer));
      uint32_t t = DL_ACINFO_itow(dl_buffer);
      set_ac_info(id, ux, uy, c, a, s, cl, t);
    }
    break;
#endif
    default:
      break;
  }
  /* Parse firmware specific datalink */
  firmware_parse_msg();

  /* Parse modules datalink */
  modules_parse_datalink(msg_id);
}

/* default emtpy WEAK implementation for firmwares without an extra firmware_parse_msg */
WEAK void firmware_parse_msg(void)
{
}

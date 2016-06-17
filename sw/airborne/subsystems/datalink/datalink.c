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

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#if USE_GPS
#include "subsystems/gps.h"
#endif

#ifdef RADIO_CONTROL_DATALINK_LED
#include "led.h"
#endif

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
      default: {
        break;
      }
    }
  } else {
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

      default:
        break;
    }
  }
  /* Parse firmware specific datalink */
  firmware_parse_msg();

  /* Parse modules datalink */
  modules_parse_datalink(msg_id);
}

/* default empty WEAK implementation for firmwares without an extra firmware_parse_msg */
WEAK void firmware_parse_msg(void)
{
}
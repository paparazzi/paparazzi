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
 * @file modules/datalink/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define DATALINK_C
#define MODULES_DATALINK_C

#include "datalink.h"
#include "modules/datalink/downlink.h"

#include "generated/modules.h"
#include "generated/settings.h"

#include "pprzlink/messages.h"

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "modules/radio_control/radio_control.h"
#endif

#if USE_GPS
#include "modules/gps/gps.h"
#endif

#ifdef RADIO_CONTROL_DATALINK_LED
#include "led.h"
#endif

#if USE_NPS
bool datalink_enabled = true;
#endif


void dl_parse_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);

  /* parse telemetry messages coming from other AC */
  if (sender_id != 0) {
    switch (msg_id) {
      default: {
        break;
      }
    }
  } else {
#if PPRZLINK_DEFAULT_VER == 2
    // Check that the message is really a datalink message
    if (pprzlink_get_msg_class_id(buf) == DL_datalink_CLASS_ID) {
#endif
      /* parse datalink messages coming from ground station */
      switch (msg_id) {
        case  DL_PING: {
#if PPRZLINK_DEFAULT_VER == 2
          // Reply to the sender of the message
          struct pprzlink_msg msg;
          msg.trans = trans;
          msg.dev = dev;
          msg.sender_id = AC_ID;
          msg.receiver_id = sender_id;
          msg.component_id = 0;
          pprzlink_msg_send_PONG(&msg);
#else
          pprz_msg_send_PONG(trans, dev, AC_ID);
#endif
        }
        break;

        case DL_SETTING : {
          if (DL_SETTING_ac_id(buf) != AC_ID) { break; }
          uint8_t i = DL_SETTING_index(buf);
          float var = DL_SETTING_value(buf);
          DlSetting(i, var);
#if PPRZLINK_DEFAULT_VER == 2
          // Reply to the sender of the message
          struct pprzlink_msg msg;
          msg.trans = trans;
          msg.dev = dev;
          msg.sender_id = AC_ID;
          msg.receiver_id = sender_id;
          msg.component_id = 0;
          pprzlink_msg_send_DL_VALUE(&msg, &i, &var);
#else
          pprz_msg_send_DL_VALUE(trans, dev, AC_ID, &i, &var);
#endif
        }
        break;

        case DL_GET_SETTING : {
          if (DL_GET_SETTING_ac_id(buf) != AC_ID) { break; }
          uint8_t i = DL_GET_SETTING_index(buf);
          float val = settings_get_value(i);
#if PPRZLINK_DEFAULT_VER == 2
          // Reply to the sender of the message
          struct pprzlink_msg msg;
          msg.trans = trans;
          msg.dev = dev;
          msg.sender_id = AC_ID;
          msg.receiver_id = sender_id;
          msg.component_id = 0;
          pprzlink_msg_send_DL_VALUE(&msg, &i, &val);
#else
          pprz_msg_send_DL_VALUE(trans, dev, AC_ID, &i, &val);
#endif
        }
        break;

#ifdef RADIO_CONTROL_TYPE_DATALINK
        case DL_RC_3CH :
#ifdef RADIO_CONTROL_DATALINK_LED
          LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
          parse_rc_3ch_datalink(
            DL_RC_3CH_throttle_mode(buf),
            DL_RC_3CH_roll(buf),
            DL_RC_3CH_pitch(buf));
          break;
        case DL_RC_4CH :
          if (DL_RC_4CH_ac_id(buf) == AC_ID) {
#ifdef RADIO_CONTROL_DATALINK_LED
            LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
            parse_rc_4ch_datalink(DL_RC_4CH_mode(buf),
                                  DL_RC_4CH_throttle(buf),
                                  DL_RC_4CH_roll(buf),
                                  DL_RC_4CH_pitch(buf),
                                  DL_RC_4CH_yaw(buf));
          }
          break;
#endif // RADIO_CONTROL_TYPE_DATALINK

#if USE_GPS
        case DL_GPS_INJECT : {
          // Check if the GPS is for this AC
          if (DL_GPS_INJECT_ac_id(buf) != AC_ID) { break; }

          // GPS parse data
          gps_inject_data(
            DL_GPS_INJECT_packet_id(buf),
            DL_GPS_INJECT_data_length(buf),
            DL_GPS_INJECT_data(buf)
          );
        }
        break;
#if USE_GPS_UBX_RTCM
        case DL_RTCM_INJECT : {
          // GPS parse data
          gps_inject_data(DL_RTCM_INJECT_packet_id(buf),
                          DL_RTCM_INJECT_data_length(buf),
                          DL_RTCM_INJECT_data(buf));
        }
        break;
#endif  // USE_GPS_UBX_RTCM
#endif  // USE_GPS

        default:
          break;
      }
#if PPRZLINK_DEFAULT_VER == 2
    }
#endif
  }
  /* Parse firmware specific datalink */
  firmware_parse_msg(dev, trans, buf);

  /* Parse modules datalink */
  modules_parse_datalink(msg_id, dev, trans, buf);
}

/* default empty WEAK implementation for firmwares without an extra firmware_parse_msg */
WEAK void firmware_parse_msg(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf __attribute__((unused)))
{
}

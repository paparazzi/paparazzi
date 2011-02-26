/*
 * $Id$
 *
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

#define DATALINK_C
#define MODULES_DATALINK_C

#include "datalink.h"

#include "generated/modules.h"

#include "generated/settings.h"
#include "downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "mcu_periph/uart.h"

#ifdef BOOZ_FMS_TYPE
#include "booz_fms.h"
#endif

#include "firmwares/rotorcraft/navigation.h"

#include "math/pprz_geodetic_int.h"
#include "subsystems/ins.h"

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {

  datalink_time = 0;

  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {

  case  DL_PING:
    {
      DOWNLINK_SEND_PONG(DefaultChannel);
    }
    break;

  case DL_SETTING :
    {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, &i, &var);
    }
    break;

  case DL_GET_SETTING :
    {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, &i, &val);
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
      uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
      struct LlaCoor_i lla;
      struct EnuCoor_i enu;
      lla.lat = INT32_RAD_OF_DEG(DL_MOVE_WP_lat(dl_buffer));
      lla.lon = INT32_RAD_OF_DEG(DL_MOVE_WP_lon(dl_buffer));
      /* WP_alt is in cm, lla.alt in mm */
      lla.alt = DL_MOVE_WP_alt(dl_buffer)*10 - ins_ltp_def.hmsl + ins_ltp_def.lla.alt;
      enu_of_lla_point_i(&enu,&ins_ltp_def,&lla);
      enu.x = POS_BFP_OF_REAL(enu.x)/100;
      enu.y = POS_BFP_OF_REAL(enu.y)/100;
      enu.z = POS_BFP_OF_REAL(enu.z)/100;
      VECT3_ASSIGN(waypoints[wp_id], enu.x, enu.y, enu.z);
      DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, &wp_id, &enu.x, &enu.y, &enu.z);
    }
    break;
#endif /* USE_NAVIGATION */
  default:
    break;
  }
  /* Parse modules datalink */
  modules_parse_datalink(msg_id);
}

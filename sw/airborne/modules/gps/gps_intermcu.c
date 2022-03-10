/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/gps/gps_intermcu.c
 * @brief GPS system based on intermcu
 */

#include "modules/gps/gps_intermcu.h"
#include "modules/core/abi.h"
#include "pprzlink/intermcu_msg.h"

struct GpsState gps_imcu;

/** GPS initialization */
void gps_intermcu_init(void)
{
  gps_imcu.fix = GPS_FIX_NONE;
  gps_imcu.pdop = 0;
  gps_imcu.sacc = 0;
  gps_imcu.pacc = 0;
  gps_imcu.cacc = 0;
  gps_imcu.comp_id = GPS_IMCU_ID;
}

void gps_intermcu_parse_IMCU_REMOTE_GPS(uint8_t *buf)
{
  uint32_t now_ts = get_sys_time_usec();

  gps_imcu.ecef_pos.x = DL_IMCU_REMOTE_GPS_ecef_x(buf);
  gps_imcu.ecef_pos.y = DL_IMCU_REMOTE_GPS_ecef_y(buf);
  gps_imcu.ecef_pos.z = DL_IMCU_REMOTE_GPS_ecef_z(buf);
  SetBit(gps_imcu.valid_fields, GPS_VALID_POS_ECEF_BIT);

  gps_imcu.lla_pos.alt = DL_IMCU_REMOTE_GPS_alt(buf);
  gps_imcu.hmsl = DL_IMCU_REMOTE_GPS_hmsl(buf);
  SetBit(gps_imcu.valid_fields, GPS_VALID_HMSL_BIT);

  gps_imcu.ecef_vel.x = DL_IMCU_REMOTE_GPS_ecef_xd(buf);
  gps_imcu.ecef_vel.y = DL_IMCU_REMOTE_GPS_ecef_yd(buf);
  gps_imcu.ecef_vel.z = DL_IMCU_REMOTE_GPS_ecef_zd(buf);
  SetBit(gps_imcu.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  gps_imcu.course = DL_IMCU_REMOTE_GPS_course(buf);
  gps_imcu.gspeed = DL_IMCU_REMOTE_GPS_gspeed(buf);
  SetBit(gps_imcu.valid_fields, GPS_VALID_COURSE_BIT);

  gps_imcu.pacc = DL_IMCU_REMOTE_GPS_pacc(buf);
  gps_imcu.sacc = DL_IMCU_REMOTE_GPS_sacc(buf);
  gps_imcu.num_sv = DL_IMCU_REMOTE_GPS_numsv(buf);
  gps_imcu.fix = DL_IMCU_REMOTE_GPS_fix(buf);

  // set gps msg time
  gps_imcu.last_msg_ticks = sys_time.nb_sec_rem;
  gps_imcu.last_msg_time = sys_time.nb_sec;
  if (gps_imcu.fix >= GPS_FIX_3D) {
    gps_imcu.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps_imcu.last_3dfix_time = sys_time.nb_sec;
  }

  AbiSendMsgGPS(GPS_IMCU_ID, now_ts, &gps_imcu);
}


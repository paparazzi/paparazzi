/*
 * Copyright (C) 2014 Sergey Krukowski <softsr@yahoo.de>
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
 * @file modules/gps/gps_sim_hitl.c
 * GPS subsystem simulation from rotorcrafts horizontal/vertical reference system
 */

#include "modules/gps/gps_sim_hitl.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"

#include "state.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "autopilot.h"

bool gps_available;
uint32_t gps_sim_hitl_timer;

void gps_sim_hitl_init(void)
{
  gps.fix = GPS_FIX_NONE;
}

void gps_sim_hitl_event(void)
{
  if (SysTimeTimer(gps_sim_hitl_timer) > 100000) {
    SysTimeTimerStart(gps_sim_hitl_timer);
    gps.last_msg_ticks = sys_time.nb_sec_rem;
    gps.last_msg_time = sys_time.nb_sec;
    if (state.ned_initialized_i) {
      if (!autopilot_in_flight()) {
        struct Int32Vect2 zero_vector;
        INT_VECT2_ZERO(zero_vector);
        gh_set_ref(zero_vector, zero_vector, zero_vector);
        INT_VECT2_ZERO(guidance_h.ref.pos);
        INT_VECT2_ZERO(guidance_h.ref.speed);
        INT_VECT2_ZERO(guidance_h.ref.accel);
        gv_set_ref(0, 0, 0);
        guidance_v_z_ref = 0;
        guidance_v_zd_ref = 0;
        guidance_v_zdd_ref = 0;
      }
      struct NedCoor_i ned_c;
      ned_c.x = guidance_h.ref.pos.x * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM;
      ned_c.y = guidance_h.ref.pos.y * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM;
      ned_c.z = guidance_v_z_ref * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM;
      ecef_of_ned_point_i(&gps.ecef_pos, &state.ned_origin_i, &ned_c);
      gps.lla_pos.alt = state.ned_origin_i.lla.alt - ned_c.z;
      gps.hmsl = state.ned_origin_i.hmsl - ned_c.z;
      ned_c.x = guidance_h.ref.speed.x * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM;
      ned_c.y = guidance_h.ref.speed.y * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM;
      ned_c.z = guidance_v_zd_ref * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM;
      ecef_of_ned_vect_i(&gps.ecef_vel, &state.ned_origin_i, &ned_c);
      gps.fix = GPS_FIX_3D;
      gps.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps.last_3dfix_time = sys_time.nb_sec;
      gps_available = true;
    }
    else {
      struct Int32Vect2 zero_vector;
      INT_VECT2_ZERO(zero_vector);
      gh_set_ref(zero_vector, zero_vector, zero_vector);
      gv_set_ref(0, 0, 0);
    }

    // publish gps data
    uint32_t now_ts = get_sys_time_usec();
    gps.last_msg_ticks = sys_time.nb_sec_rem;
    gps.last_msg_time = sys_time.nb_sec;
    if (gps.fix == GPS_FIX_3D) {
      gps.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps);
  }
}


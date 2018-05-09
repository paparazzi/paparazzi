/*
 * Copyright (C) Mario Coppola
 *
 * This file is part of paparazzi
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
 * @file "modules/relativelocalizationfilter/relativelocalizationfilter.c"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#include "relative_localization_filter.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h" // To get current velocity and height

#include "modules/datalink/extra_pprz_dl.h"
#include "subsystems/abi.h"

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int32_t ID_array[RL_NUAVS - 1];
uint32_t latest_update_time[RL_NUAVS - 1];
uint8_t number_filters;
struct discrete_ekf ekf_rl[RL_NUAVS - 1];
float range_array[RL_NUAVS - 1];
uint8_t pprzmsg_cnt;

static abi_event range_communication_event;
static void range_msg_callback(uint8_t sender_id __attribute__((unused)),
                               uint8_t ac_id, float range, float tracked_v_north, float tracked_v_east, float tracked_h)
{
  int idx = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found)

  // Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
  if (!int32_vect_find(ID_array, ac_id, &idx, RL_NUAVS - 1) &&
      (number_filters < (RL_NUAVS - 1))) {
    ID_array[number_filters] = ac_id;
    discrete_ekf_new(&ekf_rl[number_filters]);
    number_filters++;
  } else if (idx != -1) {
    range_array[idx] = range;
    ekf_rl[idx].dt = (get_sys_time_usec() - latest_update_time[idx]) / pow(10, 6); // Update the time between messages

    // Measurement Vector Z = [range owvVx(NED) ownVy(NED) tracked_v_north(NED) tracked_v_east(NED) dh]
    float Z[EKF_M] = {range, stateGetSpeedEnu_f()->y, stateGetSpeedEnu_f()->x, tracked_v_north, tracked_v_east, tracked_h - stateGetPositionEnu_f()->z};

    discrete_ekf_predict(&ekf_rl[idx]);
    discrete_ekf_update(&ekf_rl[idx], Z);
  }

  latest_update_time[idx] = get_sys_time_usec();
};

// It is best to send the data of each tracked drone separately to avoid overloading
static void send_relative_localization_data(struct transport_tx *trans, struct link_device *dev)
{
  pprzmsg_cnt++;
  if (pprzmsg_cnt >= number_filters) {
    pprzmsg_cnt = 0;
  }

  pprz_msg_send_RLFILTER(trans, dev, AC_ID,
                         &ID_array[pprzmsg_cnt], &range_array[pprzmsg_cnt],
                         &ekf_rl[pprzmsg_cnt].X[0], &ekf_rl[pprzmsg_cnt].X[1], // x y (tracked wrt own)
                         &ekf_rl[pprzmsg_cnt].X[2], &ekf_rl[pprzmsg_cnt].X[3], // vx vy (own)
                         &ekf_rl[pprzmsg_cnt].X[4], &ekf_rl[pprzmsg_cnt].X[5], // vx vy (tracked)
                         &ekf_rl[pprzmsg_cnt].X[6]); // height separation
};

void relative_localization_filter_init(void)
{
  int32_vect_set_value(ID_array, 5, RL_NUAVS - 1);
  number_filters = 0;
  pprzmsg_cnt = 0;

  AbiBindMsgUWB_COMMUNICATION(UWB_COMM_ID, &range_communication_event, range_msg_callback);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RLFILTER, send_relative_localization_data);
};

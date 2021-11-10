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
 * @file "modules/relative_localization_filter/relative_localization_filter.c"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#include "relative_localization_filter.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h" // To get current states

#include "modules/datalink/extra_pprz_dl.h"
#include "modules/core/abi.h"

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef RELATIVE_LOCALIZATION_N_UAVS
#define RELATIVE_LOCALIZATION_N_UAVS 4 // Maximum expected number of other UAVs
#endif

/*
 * RELATIVE_LOCALIZATION_NO_NORTH = 1 : The filter runs without a heading reference.
 * RELATIVE_LOCALIZATION_NO_NORTH = 0 : The filter runs while using a shared reference heading.
 */
#ifndef RELATIVE_LOCALIZATION_NO_NORTH
#define RELATIVE_LOCALIZATION_NO_NORTH 1
#endif

#if RELATIVE_LOCALIZATION_NO_NORTH
#include "discrete_ekf_no_north.h"
struct discrete_ekf_no_north ekf_rl[RELATIVE_LOCALIZATION_N_UAVS];
#else
#include "discrete_ekf.h"
struct discrete_ekf ekf_rl[RELATIVE_LOCALIZATION_N_UAVS];
#endif

int32_t id_array[RELATIVE_LOCALIZATION_N_UAVS]; // array of UWB IDs of all drones
uint32_t latest_update_time[RELATIVE_LOCALIZATION_N_UAVS];
uint8_t number_filters; // the number of filters running in parallel
float range_array[RELATIVE_LOCALIZATION_N_UAVS]; // an array to store the ranges at which the other MAVs are
uint8_t pprzmsg_cnt; // a counter to send paparazzi messages, which are sent in rotation

static abi_event range_communication_event;
static void range_msg_callback(uint8_t sender_id __attribute__((unused)), uint8_t ac_id,
                               float range, float trackedVx, float trackedVy, float trackedh,
                               float trackedAx, float trackedAy, float trackedYawr)
{
  int idx = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found)

  // Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
  if (!int32_vect_find(id_array, ac_id, &idx, RELATIVE_LOCALIZATION_N_UAVS) &&
      (number_filters < RELATIVE_LOCALIZATION_N_UAVS)) {
    id_array[number_filters] = ac_id;
#if RELATIVE_LOCALIZATION_NO_NORTH
    discrete_ekf_no_north_new(&ekf_rl[number_filters]);
#else
    discrete_ekf_new(&ekf_rl[number_filters]);
#endif
    number_filters++;
  } else if (idx != -1) {
    range_array[idx] = range;
    ekf_rl[idx].dt = (get_sys_time_usec() - latest_update_time[idx]) / pow(10, 6); // Update the time between messages

    float ownVx = stateGetSpeedNed_f()->x;
    float ownVy = stateGetSpeedNed_f()->y;
    float ownh  = stateGetPositionEnu_f()->z;
#if RELATIVE_LOCALIZATION_NO_NORTH
    float ownAx = stateGetAccelNed_f()->x;
    float ownAy = stateGetAccelNed_f()->y;
    float ownYawr = stateGetBodyRates_f()->r;
    float U[EKF_L] = {ownAx, ownAy, trackedAx, trackedAy, ownYawr, trackedYawr};
    float Z[EKF_M] = {range, ownh, trackedh, ownVx, ownVy, trackedVx, trackedVy};
    discrete_ekf_no_north_predict(&ekf_rl[idx], U);
    discrete_ekf_no_north_update(&ekf_rl[idx], Z);
#else
    // Measurement Vector Z = [range owvVx(NED) ownVy(NED) tracked_v_north(NED) tracked_v_east(NED) dh]
    float Z[EKF_M] = {range, ownVx, ownVy, trackedVx, trackedVy, trackedh - ownh};
    discrete_ekf_predict(&ekf_rl[idx]);
    discrete_ekf_update(&ekf_rl[idx], Z);
#endif

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

  if (number_filters > 0) {
    pprz_msg_send_RLFILTER(trans, dev, AC_ID,
                           &id_array[pprzmsg_cnt], &range_array[pprzmsg_cnt],
                           &ekf_rl[pprzmsg_cnt].X[0], &ekf_rl[pprzmsg_cnt].X[1], // x y (tracked wrt own)
                           &ekf_rl[pprzmsg_cnt].X[2], &ekf_rl[pprzmsg_cnt].X[3], // vx vy (own)
                           &ekf_rl[pprzmsg_cnt].X[4], &ekf_rl[pprzmsg_cnt].X[5], // vx vy (tracked)
                           &ekf_rl[pprzmsg_cnt].X[6]); // height separation
  }
};

void relative_localization_filter_init(void)
{
  int32_vect_set_value(id_array, RELATIVE_LOCALIZATION_N_UAVS + 1,
                       RELATIVE_LOCALIZATION_N_UAVS); // The id_array is initialized with non-existant IDs (assuming UWB IDs are 0,1,2...)
  number_filters = 0;
  pprzmsg_cnt = 0;

  AbiBindMsgUWB_COMMUNICATION(UWB_COMM_ID, &range_communication_event, range_msg_callback);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RLFILTER, send_relative_localization_data);
};

void relative_localization_filter_periodic(void)
{
  for (int i = 0; i < number_filters; i++) {
    // send id, x, y, z, vx, vy, vz(=0)
    AbiSendMsgRELATIVE_LOCALIZATION(RELATIVE_LOCALIZATION_ID, id_array[i], ekf_rl[i].X[0], ekf_rl[i].X[1], ekf_rl[i].X[6],
                                    ekf_rl[i].X[4], ekf_rl[i].X[5], 0.f);
  }
};

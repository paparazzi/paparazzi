/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/datalink/mavlink.c
 *  @brief Basic MAVLink datalink implementation
 */

#include "modules/datalink/mavlink.h"

#include "mavlink/paparazzi/mavlink.h"
#include "mcu_periph/sys_time.h"

#include "generated/airframe.h"
#include "generated/modules.h"
#include "generated/settings.h"

#include "subsystems/electrical.h"
#include "state.h"

mavlink_system_t mavlink_system;

static uint8_t mavlink_params_idx = NB_SETTING; /**< Transmitting parameters index */
static char mavlink_params[NB_SETTING][16] = SETTINGS; /**< Transmitting parameter names */

static inline void mavlink_send_heartbeat(void);
static inline void mavlink_send_sys_status(void);
static inline void mavlink_send_attitude(void);
static inline void mavlink_send_params(void);

//TODO FIXME
#define UAV_SENSORS (MAV_SYS_STATUS_SENSOR_3D_GYRO|MAV_SYS_STATUS_SENSOR_3D_ACCEL|MAV_SYS_STATUS_SENSOR_3D_MAG|MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)

/**
 * MAVLink initialization
 */
void mavlink_init(void)
{
  mavlink_system.sysid = AC_ID; // System ID, 1-255
  mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER; // Component/Subsystem ID, 1-255
}

/**
 * Periodic MAVLink calls
 */
void mavlink_periodic(void)
{
  RunOnceEvery(MODULES_FREQUENCY / 2, mavlink_send_heartbeat());
  RunOnceEvery(MODULES_FREQUENCY / 10, mavlink_send_sys_status());
  RunOnceEvery(MODULES_FREQUENCY / 10, mavlink_send_attitude());
  RunOnceEvery(MODULES_FREQUENCY / 10, mavlink_send_params());
}

/**
 * Event MAVLink calls
 */
void mavlink_event(void)
{
  int i, j;
  mavlink_message_t msg;
  mavlink_status_t status;

  // Check uplink
  while (MAVLink(ChAvailable())) {
    char test = MAVLink(Getch());

    // When we receive a message
    if (mavlink_parse_char(MAVLINK_COMM_0, test, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          break;

          /* When requesting data streams say we can't send them */
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
          mavlink_request_data_stream_t cmd;
          mavlink_msg_request_data_stream_decode(&msg, &cmd);

          mavlink_msg_data_stream_send(MAVLINK_COMM_0, cmd.req_stream_id, 0, 0);
          MAVLink(SendMessage());
          break;
        }

        /* Override channels with RC */
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
          mavlink_rc_channels_override_t cmd;
          mavlink_msg_rc_channels_override_decode(&msg, &cmd);
#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
          uint8_t thrust = (cmd.chan3_raw - 950) * 127 / 1100;
          int8_t roll = -(cmd.chan1_raw - 1500) * 255 / 1100 / 2;
          int8_t pitch = -(cmd.chan2_raw - 1500) * 255 / 1100 / 2;
          int8_t yaw = -(cmd.chan4_raw - 1500) * 255 / 1100;
          parse_rc_4ch_datalink(0, thrust, roll, pitch, yaw);
          //printf("RECEIVED: RC Channel Override for: %d/%d: throttle: %d; roll: %d; pitch: %d; yaw: %d;\r\n",
          // cmd.target_system, cmd.target_component, thrust, roll, pitch, yaw);
#endif
          break;
        }

        /* When a request is made of the parameters list */
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          mavlink_params_idx = 0;
          break;
        }

        /* When a request os made for a specific parameter */
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
          mavlink_param_request_read_t cmd;
          mavlink_msg_param_request_read_decode(&msg, &cmd);

          // First check param_index and search for the ID if needed
          if (cmd.param_index == -1) {

            // Go trough all the settings to search the ID
            for (i = 0; i < NB_SETTING; i++) {
              for (j = 0; j < 16; j++) {
                if (mavlink_params[i][j] != cmd.param_id[j]) {
                  break;
                }

                if (mavlink_params[i][j] == NULL) {
                  cmd.param_index = i;
                  break;
                }
              }

              if (mavlink_params[i][j] == NULL) {
                break;
              }
            }
          }

          mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                       mavlink_params[cmd.param_index],
                                       settings_get_value(cmd.param_index),
                                       MAV_PARAM_TYPE_REAL32,
                                       NB_SETTING,
                                       cmd.param_index);
          MAVLink(SendMessage());

          break;
        }

        default:
          //Do nothing
          //printf("Received message with id: %d\r\n", msg.msgid);
          break;
      }
    }

  }
}

/**
 * Send a heartbeat
 */
static inline void mavlink_send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
                             MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_PPZ,
                             MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                             0,
                             MAV_STATE_STANDBY);
  MAVLink(SendMessage());
}

/**
 * Send the system status
 */
static inline void mavlink_send_sys_status(void)
{
  mavlink_msg_sys_status_send(MAVLINK_COMM_0,
                              UAV_SENSORS,  // On-board sensors: present    (bitmap)
                              UAV_SENSORS,   // On-board sensors: active   (bitmap)
                              UAV_SENSORS,   // On-board sensors: state    (bitmap)
                              -1,//10*sys_mon.cpu_load, // System loadof main-loop time   (0=0% to 1000=100%)
                              100 * electrical.vsupply, // Battery voltage      (milivolts)
                              electrical.current / 10, // Battery current      (10x miliampere)
                              -1,     // Battery remaining      (0-100 in %)
                              0,      // Communication packet drops     (0=0% to 10000=100%)
                              0,      // Communication error(per packet)  (0=0% to 10000=100%)
                              0,      // Autopilot specific error 1
                              0,      // Autopilot specific error 2
                              0,      // Autopilot specific error 3
                              0);     // Autopilot specific error 4
  MAVLink(SendMessage());
}

/**
 * Send the attitude
 */
static inline void mavlink_send_attitude(void)
{
  mavlink_msg_attitude_send(MAVLINK_COMM_0,
                            sys_time.nb_sec * 100 + msec_of_sys_time_ticks(sys_time.nb_sec_rem),
                            stateGetNedToBodyEulers_f()->phi,     // Phi
                            stateGetNedToBodyEulers_f()->theta,   // Theta
                            stateGetNedToBodyEulers_f()->psi,     // Psi
                            stateGetBodyRates_f()->p,             // p
                            stateGetBodyRates_f()->q,             // q
                            stateGetBodyRates_f()->r);            // r
  MAVLink(SendMessage());
}

/**
 * Send the parameters
 */
static inline void mavlink_send_params(void)
{
  if (mavlink_params_idx >= NB_SETTING) {
    return;
  }

  mavlink_msg_param_value_send(MAVLINK_COMM_0,
                               mavlink_params[mavlink_params_idx],
                               settings_get_value(mavlink_params_idx),
                               MAV_PARAM_TYPE_REAL32,
                               NB_SETTING,
                               mavlink_params_idx);
  MAVLink(SendMessage());

  mavlink_params_idx++;
}
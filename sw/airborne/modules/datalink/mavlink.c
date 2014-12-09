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

// include mavlink headers, but ignore some warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "mavlink/paparazzi/mavlink.h"
#pragma GCC diagnostic pop

#include "generated/airframe.h"
#include "generated/modules.h"
#include "generated/settings.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/electrical.h"
#include "state.h"

mavlink_system_t mavlink_system;

static uint8_t mavlink_params_idx = NB_SETTING; /**< Transmitting parameters index */
static char mavlink_params[NB_SETTING][16] = SETTINGS; /**< Transmitting parameter names */

static inline void mavlink_send_heartbeat(void);
static inline void mavlink_send_sys_status(void);
static inline void mavlink_send_attitude(void);
static inline void mavlink_send_local_position_ned(void);
static inline void mavlink_send_global_position_int(void);
static inline void mavlink_send_params(void);
static inline void mavlink_send_autopilot_version(void);
static inline void mavlink_send_attitude_quaternion(void);
static inline void mavlink_send_gps_raw_int(void);
static inline void mavlink_send_rc_channels(void);
static inline void mavlink_send_battery_status(void);


/// TODO: FIXME
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
 * Periodic MAVLink calls.
 * Called at MAVLINK_PERIODIC_FREQUENCY (set in module xml to 10Hz)
 */
void mavlink_periodic(void)
{
  RunOnceEvery(2, mavlink_send_heartbeat());
  RunOnceEvery(5, mavlink_send_sys_status());
  RunOnceEvery(5, mavlink_send_attitude());
  RunOnceEvery(5, mavlink_send_params());
  RunOnceEvery(4, mavlink_send_local_position_ned());
  RunOnceEvery(5, mavlink_send_global_position_int());
  RunOnceEvery(6, mavlink_send_gps_raw_int());
  RunOnceEvery(5, mavlink_send_attitude_quaternion());
  RunOnceEvery(5, mavlink_send_rc_channels());
  RunOnceEvery(21, mavlink_send_battery_status());
  RunOnceEvery(32, mavlink_send_autopilot_version());
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

                if (mavlink_params[i][j] == '\0') {
                  cmd.param_index = i;
                  break;
                }
              }

              if (mavlink_params[i][j] == '\0') {
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
                            get_sys_time_msec(),
                            stateGetNedToBodyEulers_f()->phi,     // Phi
                            stateGetNedToBodyEulers_f()->theta,   // Theta
                            stateGetNedToBodyEulers_f()->psi,     // Psi
                            stateGetBodyRates_f()->p,             // p
                            stateGetBodyRates_f()->q,             // q
                            stateGetBodyRates_f()->r);            // r
  MAVLink(SendMessage());
}

static inline void mavlink_send_local_position_ned(void)
{
  mavlink_msg_local_position_ned_send(MAVLINK_COMM_0,
                                      get_sys_time_msec(),
                                      stateGetPositionNed_f()->x,
                                      stateGetPositionNed_f()->y,
                                      stateGetPositionNed_f()->z,
                                      stateGetSpeedNed_f()->x,
                                      stateGetSpeedNed_f()->y,
                                      stateGetSpeedNed_f()->z);
  MAVLink(SendMessage());
}

static inline void mavlink_send_global_position_int(void)
{
  float heading = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  if (heading < 0.) {
    heading += 360;
  }
  uint16_t compass_heading = heading * 100;
  int32_t relative_alt = stateGetPositionLla_i()->alt - state.ned_origin_f.hmsl;
  /// TODO: check/ask what coordinate system vel is supposed to be in, not clear from docs
  mavlink_msg_global_position_int_send(MAVLINK_COMM_0,
                                       get_sys_time_msec(),
                                       stateGetPositionLla_i()->lat,
                                       stateGetPositionLla_i()->lon,
                                       stateGetPositionLla_i()->alt,
                                       relative_alt,
                                       stateGetSpeedNed_f()->x * 100,
                                       stateGetSpeedNed_f()->y * 100,
                                       stateGetSpeedNed_f()->z * 100,
                                       compass_heading);
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

static inline void mavlink_send_autopilot_version(void)
{
  /// TODO: fill in versions correctly, how should they be encoded?
  static uint8_t custom_version[8];
  mavlink_msg_autopilot_version_send(MAVLINK_COMM_0,
                                     0,  // capabilities,
                                     54, // version
                                     custom_version);
}

static inline void mavlink_send_attitude_quaternion(void)
{
  /// TODO: check if same quaternion rotation or inverse
  mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0,
                                       get_sys_time_msec(),
                                       stateGetNedToBodyQuat_f()->qi,
                                       stateGetNedToBodyQuat_f()->qx,
                                       stateGetNedToBodyQuat_f()->qy,
                                       stateGetNedToBodyQuat_f()->qz,
                                       stateGetBodyRates_f()->p,
                                       stateGetBodyRates_f()->q,
                                       stateGetBodyRates_f()->r);
}

#if USE_GPS
#include "subsystems/gps.h"
#endif

static inline void mavlink_send_gps_raw_int(void)
{
#if USE_GPS
  int32_t course = DegOfRad(gps.course) / 1e5;
  if (course < 0) {
    course += 36000;
  }
  mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0,
                               get_sys_time_usec(),
                               gps.fix,
                               gps.lla_pos.lat,
                               gps.lla_pos.lon,
                               gps.lla_pos.alt,
                               gps.pdop,
                               UINT16_MAX, // VDOP
                               gps.gspeed,
                               course,
                               gps.num_sv);
#endif
}

#if defined RADIO_CONTROL
#include "subsystems/radio_control.h"
// since they really want PPM values, use a hack to check if are using ppm subsystem
#ifdef PPM_PULSE_TYPE_POSITIVE
#define RC_CHANNELS RADIO_CTL_NB
/// macro to get ppm_pulses or UINT16_MAX if channel not available
#define PPM_PULSES(_i) ((_i) < RADIO_CTL_NB ? ppm_pulses[(_i)] : UINT16_MAX)
#else
// use radio_control.values for now...
#define RC_CHANNELS RADIO_CONTROL_NB_CHANNEL
#define PPM_OF_PPRZ(_v) ((_v) / 19.2 + 1500)
#define PPM_PULSES(_i) ((_i) < RADIO_CONTROL_NB_CHANNEL ? PPM_OF_PPRZ(radio_control.values[(_i)]) + MAX_PPRZ : UINT16_MAX)
#endif
#endif

static inline void mavlink_send_rc_channels(void)
{
#if defined RADIO_CONTROL
  mavlink_msg_rc_channels_send(MAVLINK_COMM_0,
                               get_sys_time_msec(),
                               RC_CHANNELS,
                               PPM_PULSES(0), PPM_PULSES(1), PPM_PULSES(2),
                               PPM_PULSES(3), PPM_PULSES(4), PPM_PULSES(5),
                               PPM_PULSES(6), PPM_PULSES(7), PPM_PULSES(8),
                               PPM_PULSES(9), PPM_PULSES(10), PPM_PULSES(11),
                               PPM_PULSES(12), PPM_PULSES(13), PPM_PULSES(14),
                               PPM_PULSES(15), PPM_PULSES(16), PPM_PULSES(17),
                               255); // rssi unknown
#else
  mavlink_msg_rc_channels_send(MAVLINK_COMM_0,
                               get_sys_time_msec(),
                               0, // zero channels available
                               UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                               UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                               UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                               UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                               UINT16_MAX, UINT16_MAX, 255);
#endif
}

#include "subsystems/electrical.h"
static inline void mavlink_send_battery_status(void)
{
  static uint16_t voltages[10];
  // we simply only set one cell for now
  voltages[0] = electrical.vsupply * 10;
  /// TODO: check what all these fields are supposed to represent
  mavlink_msg_battery_status_send(MAVLINK_COMM_0,
                                  0, // id
                                  0, // battery_function
                                  0, // type
                                  INT16_MAX, // unknown temperature
                                  voltages, // cell voltages
                                  electrical.current / 10,
                                  electrical.consumed,
                                  electrical.energy, // check scaling
                                  -1); // remaining percentage not estimated
}

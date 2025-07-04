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
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wuninitialized"
#include "mavlink/ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#if PERIODIC_TELEMETRY
#define PERIODIC_C_MAVLINK
#include "modules/datalink/telemetry_common.h"
#include "generated/periodic_telemetry.h"
#ifndef TELEMETRY_MAVLINK_NB_MSG
#warning Using hardcoded msg periods. To customize specify a <process name="Mavlink" type="mavlink"> in your telemetry file.
#endif
#endif

#include "generated/airframe.h"
#include "generated/modules.h"
#include "generated/settings.h"
#include "generated/flight_plan.h"

#include "mcu_periph/sys_time.h"
#include "modules/energy/electrical.h"
#include "state.h"
#include "pprz_version.h"
#include "autopilot.h"
#include "autopilot_guided.h"

#if defined RADIO_CONTROL
#include "modules/radio_control/radio_control.h"
#endif

// Change the autopilot identification code: by default identify as PPZ autopilot: alternatively as MAV_AUTOPILOT_ARDUPILOTMEGA
#ifndef MAV_AUTOPILOT_ID
#define MAV_AUTOPILOT_ID MAV_AUTOPILOT_PPZ
#endif

#include "modules/datalink/missionlib/mission_manager.h"

// for UINT16_MAX
#include <stdint.h>

mavlink_system_t mavlink_system;

static uint8_t mavlink_params_idx = NB_SETTING; /**< Transmitting parameters index */
/** mavlink parameter names.
 * 16 chars + 1 NULL termination.
 */
static char mavlink_param_names[NB_SETTING][16 + 1] = SETTINGS_NAMES_SHORT;
static uint8_t custom_version[8]; /**< first 8 bytes (16 chars) of GIT SHA1 */

mavlink_mission_mgr mission_mgr;

void mavlink_common_message_handler(const mavlink_message_t *msg);

static void mavlink_send_extended_sys_state(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_heartbeat(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_sys_status(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_system_time(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_attitude(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_local_position_ned(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_global_position_int(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_params(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_autopilot_version(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_attitude_quaternion(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_gps_raw_int(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_rc_channels(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_battery_status(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_gps_global_origin(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_gps_status(struct transport_tx *trans, struct link_device *dev);
static void mavlink_send_vfr_hud(struct transport_tx *trans, struct link_device *dev);


/**
 * MAVLink initialization
 */

#ifndef MAVLINK_SYSID
#define MAVLINK_SYSID AC_ID
#endif

#if PERIODIC_TELEMETRY && defined TELEMETRY_MAVLINK_NB_MSG
struct telemetry_cb_slots mavlink_cbs[TELEMETRY_MAVLINK_NB_MSG] = TELEMETRY_MAVLINK_CBS;
struct periodic_telemetry mavlink_telemetry = { TELEMETRY_MAVLINK_NB_MSG, mavlink_cbs };
#endif

void mavlink_init(void)
{
  mavlink_system.sysid = MAVLINK_SYSID; // System ID, 1-255
  mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1; // Component/Subsystem ID, 1-255

  get_pprz_git_version(custom_version);

  mavlink_mission_init(&mission_mgr);

#if PERIODIC_TELEMETRY && defined TELEMETRY_MAVLINK_NB_MSG
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_HEARTBEAT, mavlink_send_heartbeat);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_SYS_STATUS, mavlink_send_sys_status);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_EXTENDED_SYS_STATE, mavlink_send_extended_sys_state);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_SYSTEM_TIME, mavlink_send_system_time);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_ATTITUDE, mavlink_send_attitude);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, mavlink_send_attitude_quaternion);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_LOCAL_POSITION_NED, mavlink_send_local_position_ned);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, mavlink_send_global_position_int);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_GPS_RAW_INT, mavlink_send_gps_raw_int);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_RC_CHANNELS, mavlink_send_rc_channels);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_BATTERY_STATUS, mavlink_send_battery_status);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_AUTOPILOT_VERSION, mavlink_send_autopilot_version);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, mavlink_send_gps_global_origin);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_GPS_STATUS, mavlink_send_gps_status);
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_VFR_HUD, mavlink_send_vfr_hud);
#endif
}

/**
 * Send periodic mavlink messages as defined in Mavlink process of telemetry xml file.
 * Called at TELEMETRY_FREQUENCY
 */
void mavlink_periodic_telemetry(void)
{
#if PERIODIC_TELEMETRY && defined TELEMETRY_MAVLINK_NB_MSG
  // send periodic mavlink messages as defined in the Mavlink process of the telemetry xml file
  // transport and device not used here yet...
  periodic_telemetry_send_Mavlink(&mavlink_telemetry, NULL, NULL);
#endif
}

/**
 * Periodic MAVLink calls.
 * Called at MAVLINK_PERIODIC_FREQ (set in module xml to 10Hz)
 */
void mavlink_periodic(void)
{
#if !defined (TELEMETRY_MAVLINK_NB_MSG)
  // use these hardcoded periods if no Mavlink process in telemetry file
  RunOnceEvery(2, mavlink_send_heartbeat(NULL, NULL));
  RunOnceEvery(5, mavlink_send_sys_status(NULL, NULL));
  RunOnceEvery(20, mavlink_send_system_time(NULL, NULL));
  RunOnceEvery(10, mavlink_send_attitude(NULL, NULL));
  RunOnceEvery(5, mavlink_send_attitude_quaternion(NULL, NULL));
  RunOnceEvery(4, mavlink_send_local_position_ned(NULL, NULL));
  RunOnceEvery(5, mavlink_send_global_position_int(NULL, NULL));
  RunOnceEvery(6, mavlink_send_gps_raw_int(NULL, NULL));
  RunOnceEvery(5, mavlink_send_rc_channels(NULL, NULL));
  RunOnceEvery(21, mavlink_send_battery_status(NULL, NULL));
  RunOnceEvery(32, mavlink_send_autopilot_version(NULL, NULL));
  RunOnceEvery(33, mavlink_send_gps_global_origin(NULL, NULL));
  RunOnceEvery(10, mavlink_send_gps_status(NULL, NULL));
  RunOnceEvery(10, mavlink_send_vfr_hud(NULL, NULL));
#endif

  // if requested, send params at 5Hz until all sent
  RunOnceEvery(2, mavlink_send_params(NULL, NULL));

  mavlink_mission_periodic();
}

static int16_t settings_idx_from_param_id(char *param_id)
{
  int i, j;
  int16_t settings_idx = -1;

  // Go trough all the settings to search the ID
  for (i = 0; i < NB_SETTING; i++) {
    for (j = 0; j < 16; j++) {
      if (mavlink_param_names[i][j] != param_id[j]) {
        break;
      }

      if (mavlink_param_names[i][j] == '\0') {
        settings_idx = i;
        return settings_idx;
      }
    }

    if (mavlink_param_names[i][j] == '\0') {
      break;
    }
  }
  return settings_idx;
}

/**
 * Event MAVLink calls
 */
void mavlink_event(void)
{
  mavlink_message_t msg;
  mavlink_status_t status;

  // Check uplink
  while (MAVLinkChAvailable()) {
    char test = MAVLinkGetch();

    // When we receive a message
    if (mavlink_parse_char(MAVLINK_COMM_0, test, &msg, &status)) {
      mavlink_common_message_handler(&msg);
      mavlink_mission_message_handler(&msg);
    }
  }
}

void mavlink_common_message_handler(const mavlink_message_t *msg)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_PLAY_TUNE:
    case MAVLINK_MSG_ID_PLAY_TUNE_V2:
        DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("PLAY TUNE"), "PLAY TUNE");
      break;

    case MAVLINK_MSG_ID_HEARTBEAT:
      break;

      /* When requesting data streams say we can't send them */
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
      mavlink_request_data_stream_t cmd;
      mavlink_msg_request_data_stream_decode(msg, &cmd);

      mavlink_msg_data_stream_send(MAVLINK_COMM_0, cmd.req_stream_id, 0, 0);
      MAVLinkSendMessage();
      break;
    }

    /* Override channels with RC */
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
      mavlink_rc_channels_override_t cmd;
      mavlink_msg_rc_channels_override_decode(msg, &cmd);
#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
      uint8_t thrust = (cmd.chan3_raw - 950) * 127 / 1100;
      int8_t roll = -(cmd.chan1_raw - 1500) * 255 / 1100 / 2;
      int8_t pitch = -(cmd.chan2_raw - 1500) * 255 / 1100 / 2;
      int8_t yaw = -(cmd.chan4_raw - 1500) * 255 / 1100;
      int8_t temp_rc[4] = {roll, pitch, yaw, thrust};
      parse_rc_up_datalink(4, temp_rc);
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
      mavlink_msg_param_request_read_decode(msg, &cmd);

      // First check param_index and search for the ID if needed
      if (cmd.param_index == -1) {
        cmd.param_index = settings_idx_from_param_id(cmd.param_id);
      }

      // Send message only if the param_index was found (Coverity Scan)
      if (cmd.param_index > -1) {
        mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                     mavlink_param_names[cmd.param_index],
                                     settings_get_value(cmd.param_index),
                                     MAV_PARAM_TYPE_REAL32,
                                     NB_SETTING,
                                     cmd.param_index);
        MAVLinkSendMessage();
      }
      break;
    }

    case MAVLINK_MSG_ID_PARAM_SET: {
      mavlink_param_set_t set;
      mavlink_msg_param_set_decode(msg, &set);

      // Check if this message is for this system
      if ((uint8_t) set.target_system == AC_ID) {
        int16_t idx = settings_idx_from_param_id(set.param_id);

        // setting found
        if (idx >= 0) {
          // Only write if new value is NOT "not-a-number"
          // AND is NOT infinity
          if (set.param_type == MAV_PARAM_TYPE_REAL32 &&
              !isnan(set.param_value) && !isinf(set.param_value)) {
            DlSetting(idx, set.param_value);
            // Report back new value
            mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                         mavlink_param_names[idx],
                                         settings_get_value(idx),
                                         MAV_PARAM_TYPE_REAL32,
                                         NB_SETTING,
                                         idx);
            MAVLinkSendMessage();
          }
        }
      }
    }
    break;

#ifdef ROTORCRAFT_FIRMWARE
    /* only for rotorcraft */
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(msg, &cmd);
      // Check if this message is for this system
      if ((uint8_t) cmd.target_system == AC_ID) {
        uint8_t result = MAV_RESULT_UNSUPPORTED;
        switch (cmd.command) {

          case MAV_CMD_SET_MESSAGE_INTERVAL:
          case MAV_CMD_DO_SET_MODE: {
            char error_msg[200];
            int rc = snprintf(error_msg, 200, "Do set %d: %0.0f %0.0f (%d)", cmd.command, cmd.param1, cmd.param2, cmd.target_system);
            if (rc > 0) {
              DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, error_msg);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
          }

          case MAV_CMD_NAV_GUIDED_ENABLE:
            MAVLINK_DEBUG("got cmd NAV_GUIDED_ENABLE: %f\n", cmd.param1);
            result = MAV_RESULT_FAILED;
            if (cmd.param1 > 0.5) {
              autopilot_set_mode(AP_MODE_GUIDED);
              if (autopilot_get_mode() == AP_MODE_GUIDED) {
                result = MAV_RESULT_ACCEPTED;
              }
            }
            else {
              // turn guided mode off - to what? maybe NAV? or MODE_AUTO2?
            }
            break;

          case MAV_CMD_COMPONENT_ARM_DISARM:
            /* supposed to use this command to arm or SET_MODE?? */
            MAVLINK_DEBUG("got cmd COMPONENT_ARM_DISARM: %f\n", cmd.param1);
            result = MAV_RESULT_FAILED;
            if (cmd.param1 > 0.5) {
              autopilot_set_motors_on(TRUE);
              if (autopilot_get_motors_on())
                result = MAV_RESULT_ACCEPTED;
            }
            else {
              autopilot_set_motors_on(FALSE);
              if (!autopilot_get_motors_on())
                result = MAV_RESULT_ACCEPTED;
            }
            break;

          default:
            break;
        }
        // confirm command with result
        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, result, 0, UINT8_MAX, msg->sysid, msg->compid);
        MAVLinkSendMessage();
      }
      break;
    }

#ifdef WP_ML_global_target
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
      mavlink_set_position_target_global_int_t target;
      mavlink_msg_set_position_target_global_int_decode(msg, &target);

      // Check if this message is for this system
      //if (target.target_system == AC_ID) {
        MAVLINK_DEBUG("SET_POSITION_TARGET_GLOBAL_INT, type_mask: %d, frame: %d\n", target.type_mask, target.coordinate_frame);
        /* if position and yaw bits are not set to ignored, use only position for now */
        if (target.coordinate_frame == MAV_FRAME_GLOBAL || target.coordinate_frame == MAV_FRAME_GLOBAL_INT) {
          MAVLINK_DEBUG("set position target, frame MAV_FRAME_GLOBAL %f \n", target.alt);
          struct NedCoor_i ned;
          //struct NedCoor_f ned_f;
          struct LlaCoor_i lla;
          lla.lat = target.lat_int;
          lla.lon = target.lon_int;
          lla.alt = MM_OF_M(target.alt);
          struct LtpDef_i *origin = stateGetNedOrigin_i();
          if (origin == NULL) {
            MAVLINK_DEBUG("NED origin not set, cannot set target position\n");
            return;
          }
          ned_of_lla_point_i(&ned, origin, &lla);
          //NED_FLOAT_OF_BFP(ned_f, ned);
          //autopilot_guided_goto_ned(ned_f.x, ned_f.y, ned_f.z, target.yaw);
          waypoint_set_latlon(WP_ML_global_target, &lla);
          waypoint_set_alt(WP_ML_global_target, target.alt);

          // Downlink the new waypoint
          uint8_t wp_id = WP_ML_global_target;
          int32_t hmsl = lla.alt - stateGetNedOrigin_i()->alt + stateGetHmslOrigin_i();
          DOWNLINK_SEND_WP_MOVED_LLA(DefaultChannel, DefaultDevice, &wp_id,
                                    &lla.lat, &lla.lon, &hmsl);

        }
      break;
    }
#endif

    case MAVLINK_MSG_ID_SET_MODE: {
      mavlink_set_mode_t mode;
      mavlink_msg_set_mode_decode(msg, &mode);
      if (mode.target_system == AC_ID) {
        MAVLINK_DEBUG("got SET_MODE: base_mode:%d\n", mode.base_mode);
        if (mode.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
          autopilot_set_motors_on(TRUE);
        }
        else {
          autopilot_set_motors_on(FALSE);
        }
        if (mode.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) {
          autopilot_set_mode(AP_MODE_GUIDED);
        }
        else if (mode.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) {
          autopilot_set_mode(AP_MODE_NAV);
        }
      }
      break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {
      mavlink_set_position_target_local_ned_t target;
      mavlink_msg_set_position_target_local_ned_decode(msg, &target);
      // Check if this message is for this system
      if (target.target_system == AC_ID) {
        MAVLINK_DEBUG("SET_POSITION_TARGET_LOCAL_NED, byte_mask: %d\n", target.type_mask);
        /* if position and yaw bits are not set to ignored, use only position for now */
        if (!(target.type_mask & 0b1110000000100000)) {
          switch (target.coordinate_frame) {
            case MAV_FRAME_LOCAL_NED:
              MAVLINK_DEBUG("set position target, frame LOCAL_NED\n");
              autopilot_guided_goto_ned(target.x, target.y, target.z, target.yaw);
              break;
            case MAV_FRAME_LOCAL_OFFSET_NED:
              MAVLINK_DEBUG("set position target, frame LOCAL_OFFSET_NED\n");
              autopilot_guided_goto_ned_relative(target.x, target.y, target.z, target.yaw);
              break;
            case MAV_FRAME_BODY_OFFSET_NED:
              MAVLINK_DEBUG("set position target, frame BODY_OFFSET_NED\n");
              autopilot_guided_goto_body_relative(target.x, target.y, target.z, target.yaw);
              break;
            default:
              break;
          }
        }
        else if (!(target.type_mask & 0b0001110000100000)) {
          /* position is set to ignore, but velocity not */
          switch (target.coordinate_frame) {
            case MAV_FRAME_LOCAL_NED:
              MAVLINK_DEBUG("set velocity target, frame LOCAL_NED\n");
              autopilot_guided_move_ned(target.vx, target.vy, target.vz, target.yaw);
              break;
            default:
              break;
          }
        }
      }
      break;
    }
#endif

    default:
      //Do nothing
      MAVLINK_DEBUG("Received message with id: %d\r\n", msg->msgid);
      break;
  }
}

/* ignore the unused-parameter warnings */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
 * Send a heartbeat
 */
static void mavlink_send_heartbeat(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mav_state = MAV_STATE_CALIBRATING;
  uint8_t mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // Ardupilot custom mode enabled
  uint32_t custom_mode = 0;
#if defined(FIXEDWING_FIRMWARE)
  uint8_t mav_type = MAV_TYPE_FIXED_WING;
  switch (autopilot_get_mode()) {
    case AP_MODE_MANUAL:
      mav_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    case AP_MODE_AUTO1:
      mav_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    case AP_MODE_AUTO2:
      mav_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
      break;
    case AP_MODE_HOME:
      mav_mode |= MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED;
      break;
    default:
      break;
  }
#elif defined(ROTORCRAFT_FIRMWARE)
  uint8_t mav_type = MAV_TYPE_QUADROTOR;
  switch (autopilot_get_mode()) {
    case AP_MODE_HOME:
      mav_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
      break;
    case AP_MODE_RATE_DIRECT:
    case AP_MODE_RATE_RC_CLIMB:
    case AP_MODE_RATE_Z_HOLD:
    case AP_MODE_RC_DIRECT:
      mav_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    case AP_MODE_ATTITUDE_DIRECT:
    case AP_MODE_ATTITUDE_CLIMB:
    case AP_MODE_ATTITUDE_Z_HOLD:
    case AP_MODE_ATTITUDE_RC_CLIMB:
    case AP_MODE_HOVER_DIRECT:
    case AP_MODE_HOVER_CLIMB:
    case AP_MODE_HOVER_Z_HOLD:
    case AP_MODE_CARE_FREE_DIRECT:
      mav_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    case AP_MODE_NAV:
      mav_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
      custom_mode = PLANE_MODE_GUIDED;
      break;
    case AP_MODE_GUIDED:
      mav_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
      custom_mode = PLANE_MODE_GUIDED;
    default:
      break;
  }
#else
#error "mavlink datalink: unsupported firmware"
#endif
  if (stateIsAttitudeValid()) {
    if (autopilot_throttle_killed()) {
      mav_state = MAV_STATE_STANDBY;
    } else {
      mav_state = MAV_STATE_ACTIVE;
      mav_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
  }
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
                             mav_type,
                             MAV_AUTOPILOT_ID,
                             mav_mode,
                             custom_mode,
                             mav_state);
  MAVLinkSendMessage();
}

/*
 * Send extended system state
*/
static void mavlink_send_extended_sys_state(struct transport_tx *trans, struct link_device *dev) {
  uint8_t landed_state = autopilot_in_flight()? MAV_LANDED_STATE_IN_AIR : MAV_LANDED_STATE_ON_GROUND;
  mavlink_msg_extended_sys_state_send(MAVLINK_COMM_0, MAV_VTOL_STATE_UNDEFINED, landed_state);
  MAVLinkSendMessage();
}

/**
 * Send the system status
 */
static void mavlink_send_sys_status(struct transport_tx *trans, struct link_device *dev)
{
  /// TODO: FIXME
#define UAV_SENSORS (MAV_SYS_STATUS_SENSOR_3D_GYRO|MAV_SYS_STATUS_SENSOR_3D_ACCEL|MAV_SYS_STATUS_SENSOR_3D_MAG|MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)

  mavlink_msg_sys_status_send(MAVLINK_COMM_0,
                              UAV_SENSORS,  // On-board sensors: present    (bitmap)
                              UAV_SENSORS,   // On-board sensors: active   (bitmap)
                              UAV_SENSORS,   // On-board sensors: state    (bitmap)
                              -1,//10*sys_mon.cpu_load, // System loadof main-loop time   (0=0% to 1000=100%)
                              electrical.vsupply * 1000.f, // Battery voltage (milivolts)
                              electrical.current * 100.f,  // Battery current (10x miliampere)
                              -1,     // Battery remaining      (0-100 in %)
                              0,      // Communication packet drops     (0=0% to 10000=100%)
                              0,      // Communication error(per packet)  (0=0% to 10000=100%)
                              0,      // Autopilot specific error 1
                              0,      // Autopilot specific error 2
                              0,      // Autopilot specific error 3
                              0,      // Autopilot specific error 4
                              0,
                              0,
                              0);     
  MAVLinkSendMessage();
}

/**
 * Send SYSTEM_TIME
 * - time_unix_usec
 * - time_boot_ms
 */
static void mavlink_send_system_time(struct transport_tx *trans, struct link_device *dev)
{
  mavlink_msg_system_time_send(MAVLINK_COMM_0, 0, get_sys_time_msec());
  MAVLinkSendMessage();
}

/**
 * Send the attitude
 */
static void mavlink_send_attitude(struct transport_tx *trans, struct link_device *dev)
{
  mavlink_msg_attitude_send(MAVLINK_COMM_0,
                            get_sys_time_msec(),
                            stateGetNedToBodyEulers_f()->phi,     // Phi
                            stateGetNedToBodyEulers_f()->theta,   // Theta
                            stateGetNedToBodyEulers_f()->psi,     // Psi
                            stateGetBodyRates_f()->p,             // p
                            stateGetBodyRates_f()->q,             // q
                            stateGetBodyRates_f()->r);            // r
  MAVLinkSendMessage();
}

static void mavlink_send_local_position_ned(struct transport_tx *trans, struct link_device *dev)
{
  mavlink_msg_local_position_ned_send(MAVLINK_COMM_0,
                                      get_sys_time_msec(),
                                      stateGetPositionNed_f()->x,
                                      stateGetPositionNed_f()->y,
                                      stateGetPositionNed_f()->z,
                                      stateGetSpeedNed_f()->x,
                                      stateGetSpeedNed_f()->y,
                                      stateGetSpeedNed_f()->z);
  MAVLinkSendMessage();
}

static void mavlink_send_global_position_int(struct transport_tx *trans, struct link_device *dev)
{
  float heading = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  if (heading < 0.) {
    heading += 360;
  }
  uint16_t compass_heading = heading * 100;
  int32_t relative_alt = stateGetPositionLla_i()->alt - stateGetLlaOrigin_i().alt;
  int32_t origin_alt = 0;
  struct LtpDef_i *ned_origin = stateGetNedOrigin_i();
  if (ned_origin != NULL) {
    origin_alt = ned_origin->lla.alt;
  }
  int32_t hmsl_alt = stateGetHmslOrigin_i() - origin_alt;
  /// TODO: check/ask what coordinate system vel is supposed to be in, not clear from docs
  mavlink_msg_global_position_int_send(MAVLINK_COMM_0,
                                       get_sys_time_msec(),
                                       stateGetPositionLla_i()->lat,
                                       stateGetPositionLla_i()->lon,
                                       stateGetPositionLla_i()->alt + hmsl_alt,
                                       relative_alt,
                                       stateGetSpeedNed_f()->x * 100,
                                       stateGetSpeedNed_f()->y * 100,
                                       stateGetSpeedNed_f()->z * 100,
                                       compass_heading);
  MAVLinkSendMessage();
}

static void mavlink_send_gps_global_origin(struct transport_tx *trans, struct link_device *dev)
{
  if (state.ned_initialized_i) {
    mavlink_msg_gps_global_origin_send(MAVLINK_COMM_0,
                                       stateGetLlaOrigin_i().lat,
                                       stateGetLlaOrigin_i().lon,
                                       stateGetHmslOrigin_i(),
                                       get_sys_time_usec());
    MAVLinkSendMessage();
  }
}

/**
 * Send the parameters
 */
static void mavlink_send_params(struct transport_tx *trans, struct link_device *dev)
{
  if (mavlink_params_idx >= NB_SETTING) {
    return;
  }

  mavlink_msg_param_value_send(MAVLINK_COMM_0,
                               mavlink_param_names[mavlink_params_idx],
                               settings_get_value(mavlink_params_idx),
                               MAV_PARAM_TYPE_REAL32,
                               NB_SETTING,
                               mavlink_params_idx);
  MAVLinkSendMessage();

  mavlink_params_idx++;
}

static void mavlink_send_autopilot_version(struct transport_tx *trans, struct link_device *dev)
{
  /// TODO: fill in versions correctly, how should they be encoded?
  static uint32_t ver = PPRZ_VERSION_INT;
  static uint64_t sha;
  get_pprz_git_version((uint8_t *)&sha);
  mavlink_msg_autopilot_version_send(MAVLINK_COMM_0,
                                     18446744073709551615U, //uint64_t capabilities,
                                     ver, //uint32_t flight_sw_version,
                                     0, //uint32_t middleware_sw_version,
                                     0, //uint32_t os_sw_version,
                                     0, //uint32_t board_version,
                                     0, //const uint8_t *flight_custom_version,
                                     0, //const uint8_t *middleware_custom_version,
                                     0, //const uint8_t *os_custom_version,
                                     0, //uint16_t vendor_id,
                                     0, //uint16_t product_id,
                                     sha, //uint64_t uid
                                     0);
  MAVLinkSendMessage();
}

static void mavlink_send_attitude_quaternion(struct transport_tx *trans, struct link_device *dev)
{
  float repr_offset_q[4] = {0, 0, 0, 0};
  mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0,
                                       get_sys_time_msec(),
                                       stateGetNedToBodyQuat_f()->qi,
                                       stateGetNedToBodyQuat_f()->qx,
                                       stateGetNedToBodyQuat_f()->qy,
                                       stateGetNedToBodyQuat_f()->qz,
                                       stateGetBodyRates_f()->p,
                                       stateGetBodyRates_f()->q,
                                       stateGetBodyRates_f()->r,
                                       repr_offset_q);
  MAVLinkSendMessage();
}

#if USE_GPS
#include "modules/gps/gps.h"
#endif

static void mavlink_send_gps_raw_int(struct transport_tx *trans, struct link_device *dev)
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
                               gps.hmsl,
                               gps.pdop,
                               UINT16_MAX, // VDOP
                               gps.gspeed,
                               course,
                               gps.num_sv,
                               gps.lla_pos.alt,
                               gps.hacc,
                               gps.vacc,
                               gps.sacc,
                               0,
                               0);
  MAVLinkSendMessage();
#endif
}

/**
 * Send gps status.
 * The GPS status message consists of:
 *  - satellites visible
 *  - satellite pnr[] (ignored)
 *  - satellite used[] (ignored)
 *  - satellite elevation[] (ignored)
 *  - satellite azimuth[] (ignored)
 *  - satellite snr[] (ignored)
 */
static void mavlink_send_gps_status(struct transport_tx *trans, struct link_device *dev)
{
#if USE_GPS
  uint8_t nb_sats = Min(GPS_NB_CHANNELS, 20);
  uint8_t prn[20];
  uint8_t used[20];
  uint8_t elevation[20];
  uint8_t azimuth[20];
  uint8_t snr[20];
  for (uint8_t i = 0; i < nb_sats; i++) {
    prn[i] = gps.svinfos[i].svid;
    // set sat as used if cno > 0, not quite true though
    if (gps.svinfos[i].cno > 0) {
      used[i] = 1;
    }
    elevation[i] = gps.svinfos[i].elev;
    // convert azimuth to unsigned representation: 0: 0 deg, 255: 360 deg.
    uint8_t azim;
    if (gps.svinfos[i].azim < 0) {
      azim = (float)(-gps.svinfos[i].azim + 180) / 360 * 255;
    } else {
      azim = (float)gps.svinfos[i].azim / 360 * 255;
    }
    azimuth[i] = azim;
  }
  mavlink_msg_gps_status_send(MAVLINK_COMM_0,
                              gps.num_sv, prn, used, elevation, azimuth, snr);
  MAVLinkSendMessage();
#endif
}

#if defined RADIO_CONTROL
#include "modules/radio_control/radio_control.h"
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

static void mavlink_send_rc_channels(struct transport_tx *trans, struct link_device *dev)
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
  MAVLinkSendMessage();
}

#include "modules/energy/electrical.h"
static void mavlink_send_battery_status(struct transport_tx *trans, struct link_device *dev)
{
  static uint16_t voltages[14] = {0};
  // we simply only set one cell for now
  voltages[0] = electrical.vsupply * 1000.f;  // convert to mV
  /// TODO: check what all these fields are supposed to represent
  mavlink_msg_battery_status_send(MAVLINK_COMM_0,
                                  0, // id
                                  0, // battery_function
                                  0, // type
                                  INT16_MAX, // unknown temperature
                                  voltages, // cell voltages
                                  electrical.current * 100.f, // convert to deciA
                                  electrical.charge * 1000.f, // convert to mAh
                                  electrical.energy * 36, // convert to hecto Joule
                                  -1, // remaining percentage not estimated
                                  0,
                                  MAV_BATTERY_CHARGE_STATE_UNDEFINED,
                                  &voltages[10],
                                  MAV_BATTERY_MODE_UNKNOWN,
                                  0);
  MAVLinkSendMessage();
}

#include "modules/core/commands.h"
/**
 * Send Metrics typically displayed on a HUD for fixed wing aircraft.
 */
static void mavlink_send_vfr_hud(struct transport_tx *trans, struct link_device *dev)
{
  /* Current heading in degrees, in compass units (0..360, 0=north) */
  int16_t heading = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  /* Current throttle setting in integer percent, 0 to 100 */
  // is a 16bit unsigned int but supposed to be from 0 to 100??
  uint16_t throttle;
#ifdef COMMAND_THRUST
  throttle = commands[COMMAND_THRUST] / (MAX_PPRZ / 100);
#elif defined COMMAND_THROTTLE
  throttle = commands[COMMAND_THROTTLE] / (MAX_PPRZ / 100);
#endif
  float hmsl_alt = stateGetHmslOrigin_f() - stateGetLlaOrigin_f().alt;
  mavlink_msg_vfr_hud_send(MAVLINK_COMM_0,
                           Max(0,stateGetAirspeed_f()),
                           stateGetHorizontalSpeedNorm_f(), // groundspeed
                           heading,
                           throttle,
                           stateGetPositionLla_f()->alt + hmsl_alt,
                           stateGetSpeedNed_f()->z); // climb rate
  MAVLinkSendMessage();
}

/* end ignore unused-paramter */
#pragma GCC diagnostic pop

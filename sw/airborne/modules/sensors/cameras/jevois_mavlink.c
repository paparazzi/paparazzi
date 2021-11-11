/*
 * Copyright (C) MAVLab
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
 * @file "modules/sensors/cameras/jevois_mavlink.c"
 * @author MAVLab
 * Send sensor data to jevois and read commands from jevois
 */

#include "modules/sensors/cameras/jevois_mavlink.h"


#define DEBUG_PRINT printf
#include <stdio.h>


/*
 * MavLink protocol
 */

#include <mavlink/mavlink_types.h>
#include "mavlink/paparazzi/mavlink.h"

#include "modules/imu/imu.h"
#include "autopilot.h"
#include "generated/modules.h"

#include "modules/core/abi.h"
#include "modules/core/abi_sender_ids.h"

#include "std.h"
#include "subsystems/datalink/telemetry.h"


mavlink_system_t mavlink_system;

#ifndef MAVLINK_SYSID
#define MAVLINK_SYSID 1
#endif



static void mavlink_send_heartbeat(void);
static void mavlink_send_attitude(void);
static void mavlink_send_highres_imu(void);
static void mavlink_send_set_mode(void);


/*
 * Exported data
 */


struct visual_target_struct {
  int received;
  int count;
  int x;
  int y;
  int w;
  int h;
  int quality;
  int source;
} jevois_visual_target = {false, 0, 0, 0, 0, 0, 0};

struct vision_relative_position_struct {
  int received;
  int cnt;
  float x;
  float y;
  float z;
} jevois_vision_position = {false, 0, 0.0f, 0.0f, 0.0f};

/*
 * Paparazzi Module functions : filter functions
 */

#include "filters/low_pass_filter.h"

Butterworth2LowPass_int ax_filtered;
Butterworth2LowPass_int ay_filtered;
Butterworth2LowPass_int az_filtered;

void jevois_mavlink_filter_periodic(void)
{
  update_butterworth_2_low_pass_int(&ax_filtered, imu.accel.x);
  update_butterworth_2_low_pass_int(&ay_filtered, imu.accel.y);
  update_butterworth_2_low_pass_int(&az_filtered, imu.accel.z);
}

void jevois_mavlink_filter_init(void);
void jevois_mavlink_filter_init(void)
{
  init_butterworth_2_low_pass_int(&ax_filtered, 20.0, 1.0 / ((float)MODULES_FREQUENCY), imu.accel_unscaled.x);
  init_butterworth_2_low_pass_int(&ay_filtered, 20.0, 1.0 / ((float)MODULES_FREQUENCY), imu.accel_unscaled.y);
  init_butterworth_2_low_pass_int(&az_filtered, 20.0, 1.0 / ((float)MODULES_FREQUENCY), imu.accel_unscaled.z);
}

/*
 * Paparazzi Module functions : forward to telemetry
 */

// Send Manual Setpoint over telemetry using ROTORCRAFT_RADIO_CONTROL message

static void send_jevois_mavlink_visual_target(struct transport_tx *trans, struct link_device *dev)
{
  if (jevois_visual_target.received) {
    jevois_visual_target.received = false;
    uint16_t cnt = jevois_visual_target.count;
    int16_t tx = jevois_visual_target.x;
    int16_t ty = jevois_visual_target.y;
    int16_t w = jevois_visual_target.w;
    int16_t h = jevois_visual_target.h;
    uint16_t s = jevois_visual_target.source;
    pprz_msg_send_VISUALTARGET(trans, dev, AC_ID, &cnt,
                               &tx, &ty, &w, &h, &s);
  }
}

static void send_jevois_mavlink_visual_position(struct transport_tx *trans, struct link_device *dev)
{
  if (jevois_vision_position.received) {
    jevois_vision_position.received = false;
    uint16_t cnt = jevois_vision_position.cnt;
    float foo = 0;
    pprz_msg_send_VISION_POSITION_ESTIMATE(trans, dev, AC_ID,
                               &cnt,
                               &jevois_vision_position.x,
                               &jevois_vision_position.y,
                               &jevois_vision_position.z,
                               &foo, &foo );
  }
}

/*
 * Paparazzi Module functions : communications
 */

void jevois_mavlink_init(void)
{
  // Initialize Mavlink parser
  mavlink_system.sysid = MAVLINK_SYSID; // System ID, 1-255
  mavlink_system.compid = 0; // Component/Subsystem ID, 1-255

  // Initialize Anti-Aliasing Sensor Filter
  jevois_mavlink_filter_init();

  // Send telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUALTARGET, send_jevois_mavlink_visual_target);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISION_POSITION_ESTIMATE, send_jevois_mavlink_visual_position);
}

void jevois_mavlink_periodic(void)
{
  RunOnceEvery(100, mavlink_send_heartbeat());
  RunOnceEvery(2, mavlink_send_attitude());
  RunOnceEvery(1, mavlink_send_highres_imu());
  RunOnceEvery(1, mavlink_send_set_mode());
}

#ifndef JEVOIS_MAVLINK_ABI_ID
#define JEVOIS_MAVLINK_ABI_ID 34
#endif



void jevois_mavlink_event(void)
{
  mavlink_message_t msg;
  mavlink_status_t status;


  while (MAVLinkChAvailable()) {
    uint8_t c = MAVLinkGetch();
    if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
      //printf("msg.msgid is %d\n",msg.msgid);
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&msg, &heartbeat);
          // do something with heartbeat variable
          DEBUG_PRINT("[jevois mavlink] heartbeat\n");
        }
        break;

        case MAVLINK_MSG_ID_DEBUG: {
          mavlink_debug_t jevois_mavlink_debug;
          mavlink_msg_debug_decode(&msg, &jevois_mavlink_debug);

          DEBUG_PRINT("[jevois mavlink] debug value is %f\n", jevois_mavlink_debug.value);
          DEBUG_PRINT("[jevois mavlink] debug ind is %d\n", jevois_mavlink_debug.ind);
        }
        break;
        case MAVLINK_MSG_ID_HIGHRES_IMU: {
          mavlink_highres_imu_t jevois_mavlink_visual_target;
          mavlink_msg_highres_imu_decode(&msg, &jevois_mavlink_visual_target);

          jevois_visual_target.received = true;
          jevois_visual_target.count ++;
          jevois_visual_target.x = jevois_mavlink_visual_target.xacc;
          jevois_visual_target.y = jevois_mavlink_visual_target.yacc;
          jevois_visual_target.w = jevois_mavlink_visual_target.xmag;
          jevois_visual_target.h = jevois_mavlink_visual_target.ymag;
          jevois_visual_target.quality = jevois_mavlink_visual_target.abs_pressure;
          jevois_visual_target.source = jevois_mavlink_visual_target.diff_pressure;


          AbiSendMsgVISUAL_DETECTION(JEVOIS_MAVLINK_ABI_ID,
                                     jevois_visual_target.x,     // pixel-x
                                     jevois_visual_target.y,     // pixel-y
                                     jevois_visual_target.w,     // size
                                     jevois_visual_target.h,     // size
                                     jevois_visual_target.quality,    // quality
                                     jevois_visual_target.source);

          DEBUG_PRINT("[jevois mavlink] VISUAL_DETECTION %f,%f\n", jevois_mavlink_visual_target.xacc,
                      jevois_mavlink_visual_target.yacc);

        }
        break;

        case MAVLINK_MSG_ID_MANUAL_SETPOINT: {
          mavlink_manual_setpoint_t jevois_mavlink_manual_setpoint;
          mavlink_msg_manual_setpoint_decode(&msg, &jevois_mavlink_manual_setpoint);

          AbiSendMsgMANUAL_SETPOINT(JEVOIS_MAVLINK_ABI_ID, jevois_mavlink_manual_setpoint.thrust,
                                    jevois_mavlink_manual_setpoint.roll,
                                    jevois_mavlink_manual_setpoint.pitch,
                                    jevois_mavlink_manual_setpoint.yaw);

          DEBUG_PRINT("[jevois mavlink] phi_cmd = %f\n", DegOfRad(jevois_mavlink_manual_setpoint.roll));
          DEBUG_PRINT("[jevois mavlink] theta_cmd = %f\n", DegOfRad(jevois_mavlink_manual_setpoint.pitch));
          DEBUG_PRINT("[jevois mavlink] psi_cmd = %f\n", DegOfRad(jevois_mavlink_manual_setpoint.yaw));
          DEBUG_PRINT("[jevois mavlink] alt_cmd = %f\n", jevois_mavlink_manual_setpoint.thrust);

        }
        break;


        case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE: {
          mavlink_vision_position_estimate_t jevois_mavlink_vision_position_estimate;
          mavlink_msg_vision_position_estimate_decode(&msg, &jevois_mavlink_vision_position_estimate);

          jevois_vision_position.received = true;
          jevois_vision_position.cnt++;
          jevois_vision_position.x = jevois_mavlink_vision_position_estimate.x;
          jevois_vision_position.y = jevois_mavlink_vision_position_estimate.y;
          jevois_vision_position.z = jevois_mavlink_vision_position_estimate.z;

          AbiSendMsgRELATIVE_LOCALIZATION(JEVOIS_MAVLINK_ABI_ID,
                                          jevois_vision_position.cnt,
                                          jevois_vision_position.x,
                                          jevois_vision_position.y,
                                          jevois_vision_position.z,
                                          0.0f,
                                          0.0f,
                                          0.0f);

          DEBUG_PRINT("[jevois mavlink] VISION_POSITION_ESTIMATE %f,%f,%f \n", jevois_vision_position.x,
              jevois_vision_position.y, jevois_vision_position.z);

        }
        break;

      }
    }
  }
}


/////////////////////////////


#include "state.h"
#include "mcu_periph/sys_time.h"



static void mavlink_send_attitude(void)
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


static void mavlink_send_set_mode(void)
{
  mavlink_msg_set_mode_send(MAVLINK_COMM_0,
                            get_sys_time_msec(),
                            0, //autopilotMode.currentMode,
                            0
                           );
  MAVLinkSendMessage();
}

static void mavlink_send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
                             MAV_TYPE_FIXED_WING,
                             MAV_AUTOPILOT_PPZ,
                             MAV_MODE_FLAG_AUTO_ENABLED,
                             0, // custom_mode
                             MAV_STATE_CALIBRATING);
  MAVLinkSendMessage();
}


static void mavlink_send_highres_imu(void)
{
  mavlink_msg_highres_imu_send(MAVLINK_COMM_0,
                               get_sys_time_msec(),
                               get_butterworth_2_low_pass_int(&ax_filtered) / 1024.0,
                               get_butterworth_2_low_pass_int(&ay_filtered) / 1024.0,
                               get_butterworth_2_low_pass_int(&az_filtered) / 1024.0,
                               stateGetBodyRates_f()->p,
                               stateGetBodyRates_f()->q,
                               stateGetBodyRates_f()->r,
                               stateGetNedToBodyEulers_f()->phi,
                               stateGetNedToBodyEulers_f()->theta,
                               stateGetNedToBodyEulers_f()->psi,
                               0,
                               0,
                               0,
                               0,
                               0);
  MAVLinkSendMessage();
}


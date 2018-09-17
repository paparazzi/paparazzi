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

#include "subsystems/imu.h"
#include "autopilot.h"
#include "generated/modules.h"

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


mavlink_manual_setpoint_t jevois_mavlink_manual_setpoint;
mavlink_debug_t jevois_mavlink_debug;
mavlink_altitude_t jevois_mavlink_altitude;
mavlink_attitude_t jevois_mavlink_attitude;
mavlink_local_position_ned_t jevois_mavlink_local_position;



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
 * Paparazzi Module functions : communications
 */

void jevois_mavlink_init(void)
{
  mavlink_system.sysid = MAVLINK_SYSID; // System ID, 1-255
  mavlink_system.compid = 0; // Component/Subsystem ID, 1-255

  jevois_mavlink_filter_init();
}

void jevois_mavlink_periodic(void)
{
  RunOnceEvery(100, mavlink_send_heartbeat());
  RunOnceEvery(2, mavlink_send_attitude());
  RunOnceEvery(1, mavlink_send_highres_imu());
  RunOnceEvery(1, mavlink_send_set_mode());
}


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
        }
        break;

        case MAVLINK_MSG_ID_ATTITUDE: {

          // read attitude command from Jevois
          mavlink_msg_attitude_decode(&msg, &jevois_mavlink_attitude);

          DEBUG_PRINT("[jevois mavlink] phi_cmd = %f\n", DegOfRad(jevois_mavlink_attitude.roll));
          DEBUG_PRINT("[jevois mavlink] theta_cmd = %f\n", DegOfRad(jevois_mavlink_attitude.pitch));
          DEBUG_PRINT("[jevois mavlink] psi_cmd = %f\n", DegOfRad(jevois_mavlink_attitude.yaw));
        }
        break;

        case MAVLINK_MSG_ID_ALTITUDE: {
          mavlink_msg_altitude_decode(&msg, &jevois_mavlink_altitude);

          DEBUG_PRINT("[jevois mavlink] desired altitude is %f", jevois_mavlink_altitude.altitude_relative);
        }
        break;

        case MAVLINK_MSG_ID_DEBUG: {
          mavlink_msg_debug_decode(&msg, &jevois_mavlink_debug);

          DEBUG_PRINT("[jevois mavlink] debug value is %f", jevois_mavlink_debug.value);
          DEBUG_PRINT("[jevois mavlink] debug ind is %d", jevois_mavlink_debug.ind);
        }
        break;

        case MAVLINK_MSG_ID_MANUAL_SETPOINT: {
          mavlink_msg_manual_setpoint_decode(&msg, &jevois_mavlink_manual_setpoint);

          DEBUG_PRINT("[jevois mavlink] phi_cmd = %f\n", DegOfRad(jevois_mavlink_manual_setpoint.roll));
          DEBUG_PRINT("[jevois mavlink] theta_cmd = %f\n", DegOfRad(jevois_mavlink_manual_setpoint.pitch));
          DEBUG_PRINT("[jevois mavlink] psi_cmd = %f\n", DegOfRad(jevois_mavlink_manual_setpoint.yaw));
          DEBUG_PRINT("[jevois mavlink] alt_cmd = %f\n", jevois_mavlink_manual_setpoint.thrust);
        }
        break;


        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
          mavlink_msg_local_position_ned_decode(&msg, &jevois_mavlink_local_position);
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


/*
 * Copyright (C) 2012 Xavier Gibert
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

/** \file ap_downlink.h
 *  \brief Set of macros defining the periodic telemetry messages of AP process
 *
 * The PeriodicSendAp() macro is generated from the telemetry description
 * (named in conf.xml, usually in conf/telemetry directory). This macro
 * is a sequence of calls to PERIODIC_SEND_message() which have to be defined
 * in the present file.
 *
 */

#ifndef MAVLINK_DOWNLINK_H
#define MAVLINK_DOWNLINK_H

#include <inttypes.h>

#include "generated/airframe.h"

#include "subsystems/datalink/downlink.h"

#include "downlink_msg.h"
#include "generated/periodic_telemetry.h"

#if defined DOWNLINK
#define Mavlink(x) x
#else
#define Mavlink(x) {}
#endif

/* _type: 1 for FIXEDWING, 2 for QUADROTOR, 4 for HELICOPTER */
#define PERIODIC_SEND_HEARTBEAT(_trans,_dev) { \
  uint32_t custom_mode = 0;\
  uint8_t _type = 1;\
  uint8_t _autopilot = 9;\
  uint8_t base_mode = 220;\
  uint8_t system_status = 0;\
  uint8_t mavlink_version = 3;\
  DOWNLINK_SEND_HEARTBEAT(_trans, _dev, &custom_mode, &_type, &_autopilot, &base_mode, &system_status, &mavlink_version);\
}


#define PERIODIC_SEND_SYS_STATUS(_trans,_dev) { \
  uint32_t _bitmask = (uint32_t) 0b11100100001111111; \
  uint32_t load_cpu = 5000; \
  uint16_t voltage_bat = vsupply*100; \
  uint16_t current_bat = (int16_t) (electrical.current/10); \
  uint16_t drop_packets = 0; \
  uint16_t err = 0; \
  int8_t bat_remaining = -1; \
  Mavlink ({ \
      DOWNLINK_SEND_SYS_STATUS(_trans, _dev, &_bitmask, &_bitmask, &_bitmask, &load_cpu, &voltage_bat, &current_bat, &drop_packets, &drop_packets, &err, &err, &err, &err, &bat_remaining); \
      }); \
}

#define PERIODIC_SEND_SYSTEM_TIME(_trans, _dev) { \
  uint64_t time_unix_usec = (uint64_t) (sys_time.nb_sec*1000000); \
  uint32_t time_boot_ms = (uint32_t) (sys_time.nb_sec*1000); \
  Mavlink ({ \
      DOWNLINK_SEND_SYSTEM_TIME(_trans, _dev, &time_unix_usec, &time_boot_ms); \
      }); \
}

/* CHANGE_OPERATOR_CONTROL_ACK */
/*
#define PERIODIC_SEND_AUTH_KEY(_trans, _dev) { \
uint8_t key_length = 32; \
char[32] key = {'0','1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16','17','18','19','20','21','22','23','24','25','26','27','28','29','*','+'};\
Mavlink ({ \
DOWNLINK_SEND_AUTH_KEY(_trans, _dev, key_length, key); \
}); \
}
*/
/* PARAM_VALUE */

/* GPS_RAW_INT */

/* GPS_STATUS */

/* SCALED_IMU */

/* RAW_IMU */

/* RAW_PRESSURE */

/* SCALED_PRESSURE */

#define PERIODIC_SEND_MAV_ATTITUDE(_trans, _dev) { \
  uint32_t time_boot_ms = (uint32_t) (sys_time.nb_sec*1000); \
  float roll = estimator_phi; \
  float pitch = estimator_theta; \
  float yaw = estimator_psi; \
  float rollspeed = estimator_p; \
  float pitchspeed = estimator_q; \
  float yawspeed = estimator_r; \
  Mavlink({ \
      DOWNLINK_SEND_MAV_ATTITUDE(_trans, _dev, &time_boot_ms, &roll, &pitch, &yaw, &rollspeed, &pitchspeed, &yawspeed); \
      }); \
}

/* ATTITUDE_QUATERNION */

#define PERIODIC_SEND_LOCAL_POSITION_NED(_trans, _dev) { \
  uint32_t time_boot_ms = (uint32_t) (sys_time.nb_sec*1000); \
  float x = (float) (gps.utm_pos.north/100) ; \
  float y = (float) (gps.utm_pos.east/100) ; \
  float z = (float) (gps.utm_pos.alt/1000) ; \
  float vx = (float) (gps.ned_vel.x/100) ; \
  float vy = (float) (gps.ned_vel.y/100) ; \
  float vz = (float) (gps.ned_vel.z/100); \
  Mavlink({ \
      DOWNLINK_SEND_LOCAL_POSITION_NED(_trans, _dev, &time_boot_ms, &x, &y, &z, &vx, &vy, &vz); \
      }); \
}

#define PERIODIC_SEND_GLOBAL_POSITION_INT(_trans, _dev) { \
  uint32_t time_boot_ms = (uint32_t) (sys_time.nb_sec*1000); \
  int32_t lat = (int32_t) gps.lla_pos.lat; \
  int32_t lon = (int32_t) gps.lla_pos.lon; \
  int32_t alt = (int32_t) gps.lla_pos.lat; \
  int32_t relative_alt = (int32_t) (estimator_z - 0) ; \
  int16_t vx = (int16_t) gps.ned_vel.x ; \
  int16_t vy = (int16_t) gps.ned_vel.y ; \
  int16_t vz = (int16_t) gps.ned_vel.z ; \
  uint16_t hdg = (uint16_t) (gps.course * 100) ; \
  Mavlink({ \
      DOWNLINK_SEND_GLOBAL_POSITION_INT(_trans, _dev, &time_boot_ms, &lat, &lon, &alt, &relative_alt, &vx, &vy, &vz, &hdg); \
      }); \
}

/* RC_CHANNELS_SCALED */

/* RC_CHANNELS_RAW */

/* SERVO_OUTPUT_RAW */

/* MISSION_ITEM */

/* MISSION_CURRENT */

/* MISSION_COUNT */

/* MISSION_ITEM_REACHED */

/* MISSION_ACK */

/* GPS_GLOBAL_ORIGIN */

#define PERIODIC_SEND_LOCAL_POSITION_SETPOINT(_trans,_dev) { \
  float x = desired_x ;\
  float y = desired_y ;\
  float z = nav_altitude ;\
  float yaw = nav_course ;\
  uint8_t coordinate_frame =  1;/*MAV_FRAME_GLOBAL*/\
  Mavlink({ \
      DOWNLINK_SEND_LOCAL_POSITION_SETPOINT(_trans, _dev, &x, &y, &z, &yaw, &coordinate_frame); \
      }); \
}

/* GLOBAL_POSITION_SETPOINT_INT */

/* SAFETY_ALLOWED_AREA */

/* ROLL_PITCH_YAW_THRUST_SETPOINT */

/* ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT */

/* NAV_CONTROLLER_OUTPUT */

/* DATA_STREAM */

#define PERIODIC_SEND_VFR_HUD(_trans, _dev) { \
  float airspeed = estimator_airspeed; \
  float groundspeed = (float) (gps.gspeed / 100); \
  float alt_msl = estimator_z; \
  float climb = (float) (gps.ned_vel.z/100); \
  int16_t heading = (int16_t) ((gps.course/1e7)*57.32); \
  uint16_t throttle = (uint16_t) (v_ctl_throttle_slewed/96); \
  Mavlink({ \
      DOWNLINK_SEND_VFR_HUD(_trans, _dev, &airspeed, &groundspeed, &alt_msl, &climb, &heading, &throttle); \
      }); \
}

/* COMMAND_ACK */

/* HIL_CONTROLS */

/* OPTICAL_FLOW */

/* GLOBAL_VISION_POSITION_ESTIMATE */

/* VISION_POSITION_ESTIMATE */

/* VISION_SPEED_ESTIMATE */

/* VICON_POSITION_ESTIMATE */

/* MEMORY_VECT */

/* DEBUG_VECT */

/* NAMED_VALUE_FLOAT */

/* NAMED_VALUE_INT */

/* STATUSTEXT */

/* DEBUG */


#endif /* MAVLINK_DOWNLINK_H */

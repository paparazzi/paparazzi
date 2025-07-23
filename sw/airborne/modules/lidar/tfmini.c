/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
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

/** @file modules/lidar/tfmini.c
 *  @brief driver for the TFMini lidar
 *
 */
#include "tfmini.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

// Test
#include "firmwares/rotorcraft/autopilot_static.h"

#ifdef USE_NPS
#include "nps_autopilot.h"
#include "slam/lidar_correction.h"
struct NPS_Lidar nps_lidar;
#endif

#define LIDAR_MIN_RANGE 0.1
#define LIDAR_MAX_RANGE 12.0

#define LIDAR_OFFSET 0.20   // Horizontal distance from the IMU to the LiDAR (in meters)
#define LIDAR_HEIGHT 0.24   // Height of the LiDAR above the ground (in meters)

#define MOTOR_SPEED 5
static uint32_t last_time = 0;

struct TFMini tfmini = {
  .parse_status = TFMINI_INITIALIZE
};


bool enable_servo = false;
float motor_speed = MOTOR_SPEED;
struct TFMiniServo tf_servo;

static void tfmini_parse(uint8_t byte);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/**
 * Downlink message lidar
 */
static void tfmini_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t) tfmini.parse_status;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &tfmini.distance,
                      &tf_servo.ang,
                      &tfmini.mode,
                      &status);
}

#ifdef USE_NPS

static void tfmini_send_nps_lidar(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_NPS_LIDAR(trans, dev, AC_ID,
                      &nps_lidar.distance,
                      &tf_servo.ang,
                      &nps_lidar.pos,
                      &nps_lidar.t,
                      &nps_lidar.s,
                      &nps_lidar.denom);
}

#endif

#endif

/**
 * Initialization function
 */
void tfmini_init(void)
{
  tfmini.device = &((TFMINI_PORT).device);

  tfmini.update_agl = USE_TFMINI_AGL;
  tfmini.compensate_rotation = TFMINI_COMPENSATE_ROTATION;

  tfmini.strength = 0;
  tfmini.distance = 0;
  tfmini.parse_status = TFMINI_PARSE_HEAD;

  tf_servo.pos = 1500*0;
  tf_servo.dir = 0;

  #if PERIODIC_TELEMETRY
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, tfmini_send_lidar);
   #ifdef USE_NPS
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NPS_LIDAR, tfmini_send_nps_lidar);
   #endif
  #endif
}


// Simulated Lidar Measurement
#ifdef USE_NPS
void sim_overwrite_lidar(){
  uint32_t now_ts = get_sys_time_usec();

  if (!ins_int.ltp_initialized) return;

  // Convert GPS position to NED coordinates
  struct FloatVect2 pos = {0.0f, 0.0f};
  struct NedCoor_i ned = {0.0f, 0.0f, 0.0f};
  ned_of_lla_point_i(&ned, stateGetNedOrigin_i(), &gps.lla_pos);
  pos.x = (float) (ned.y/100.0f);
  pos.y = (float) (ned.x/100.0f);
  float theta = M_PI/2-(stateGetNedToBodyEulers_f()->psi)-tf_servo.ang*M_PI/180;

  float min_distance = FLT_MAX;

  // Traverse the wall array and store the minimum distance (!= 0)
  for (uint8_t w = 0; w < wall_system.wall_count; w++) {
    struct Wall *wall = &wall_system.walls[w];
    for (uint8_t p = 0; p < wall->count - 1; p++) {
      struct FloatVect2 p1 = wall->points_ltp[p];
      struct FloatVect2 p2 = wall->points_ltp[p+1];
      float distance = distance_to_wall(theta, &pos, &p1, &p2);
      if (distance < min_distance && distance > 0) {
        min_distance = distance;
      }
    }
  }

  // Store that data in the variable tfmini.distance
  nps_lidar.pos = (struct FloatVect2) pos;
  nps_lidar.distance = min_distance;
  if (min_distance == FLT_MAX || min_distance > LIDAR_MAX_RANGE){
    min_distance = 0;
  }
  
  tfmini.distance = min_distance;

  // send message (if requested)
  if (tfmini.update_agl) {
    AbiSendMsgLIDAR_SERVO(AGL_LIDAR_TFMINI_ID, now_ts, tfmini.distance, tf_servo.ang);
  }
}
#endif // USE_NPS


/**
 * Lidar event function
 * Receive bytes from the UART port and parse them
 */
void tfmini_event(void)
{

  #ifdef USE_NPS
  if (nps_bypass_lidar) {
    sim_overwrite_lidar();
  }
  #else
    while (tfmini.parse_status != TFMINI_INITIALIZE && tfmini.device->char_available(tfmini.device->periph)) {
      tfmini_parse(tfmini.device->get_byte(tfmini.device->periph));
    }
  #endif
}

/**
 * Parse the lidar bytes 1 by 1
 */
static void tfmini_parse(uint8_t byte)
{
  switch (tfmini.parse_status) {
    case TFMINI_INITIALIZE:
      break;
    case TFMINI_PARSE_HEAD:
      if (byte == 0x59) {
        tfmini.parse_crc = byte;
        tfmini.parse_status++;
      }
      break;
    case TFMINI_PARSE_HEAD2:
      if (byte == 0x59) {
        tfmini.parse_crc += byte;
        tfmini.parse_status++;
      } else {
        tfmini.parse_status = TFMINI_PARSE_HEAD;
      }
      break;

    case TFMINI_PARSE_DIST_L:
      tfmini.raw_dist = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_DIST_H:
      tfmini.raw_dist |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_STRENGTH_L:
      tfmini.raw_strength = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_STRENGTH_H:
      tfmini.raw_strength |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_MODE:
      tfmini.raw_mode = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_BYTE7:
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_CHECKSUM:
      // When the CRC matches
      if (tfmini.parse_crc == byte) {
        uint32_t now_ts = get_sys_time_usec();
        tfmini.distance = tfmini.raw_dist / 100.f;
        tfmini.strength = tfmini.raw_strength;
        tfmini.mode = tfmini.raw_mode;

        // When the distance is valid
        if (tfmini.distance != 0xFFFF) {
          // compensate AGL measurement for body rotation
          if (tfmini.compensate_rotation) {
            float theta = stateGetNedToBodyEulers_f()->theta;
            float ground_distance;
            if(fabs(theta) < 0.01){
              ground_distance = 100;  // Si es 0 esta recto
            }
            else{
              ground_distance = LIDAR_HEIGHT/sinf(-theta) - LIDAR_OFFSET;
            }
            
            if ((tfmini.distance >= ground_distance) && (ground_distance > 0)) {
              tfmini.distance = 0;
            }

            // HERE IT DID THE CALCULATION THINKING OF A DRONE, IT DOESN'T WORK FOR US
            // float phi = stateGetNedToBodyEulers_f()->phi;
            // float theta = stateGetNedToBodyEulers_f()->theta;
            // float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
            // tfmini.distance = tfmini.distance * gain;
          }

          // send message (if requested)
          if (tfmini.update_agl) {
            AbiSendMsgLIDAR_SERVO(AGL_LIDAR_TFMINI_ID, now_ts, tfmini.distance, tf_servo.ang);
          }
        }
      }

      // Start reading again
      tfmini.parse_status = TFMINI_PARSE_HEAD;
      break;

    default:
      // Error, return to start
      tfmini.parse_status = TFMINI_PARSE_HEAD;
      break;
  }
}


// ####################################
// ############ LIDAR MOTOR ###########
// ####################################

#ifdef COMMAND_SERVO
void tfmini_servo(){
  if (get_sys_time_msec() > last_time + motor_speed) {
    last_time = get_sys_time_msec();
    if(enable_servo){
      tf_servo.pos += (tf_servo.dir == 0) ? 100 : -100;
      if (tf_servo.pos >= MAX_PPRZ*0.8 || tf_servo.pos <= -MAX_PPRZ*0.8) {
          tf_servo.dir ^= 1;
      }
    }
    else{
      tf_servo.pos = 0;
    }
    commands[COMMAND_SERVO] = tf_servo.pos;
    tf_servo.ang = PWM2ANGLE(tf_servo.pos);
  }
}
#else
void tfmini_servo(void) {
  // No servo functionality; do nothing
}
#endif



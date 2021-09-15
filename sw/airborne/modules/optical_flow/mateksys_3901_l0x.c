/*
 * Copyright (C) 2020 Paparazzi Team
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

/** @file modules/optical_flow/mateksys_3901_l0x.h
 *  @brief Driver for the mateksys_3901_l0x sensor via MSP protocol output
 *
 */

#include <stdlib.h>
#include "mateksys_3901_l0x.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


// Define configuration parameters
#ifndef MATEKSYS_3901_L0X_MOTION_THRES
#define MATEKSYS_3901_L0X_MOTION_THRES 100
#endif

#ifndef MATEKSYS_3901_L0X_DISTANCE_THRES
#define MATEKSYS_3901_L0X_DISTANCE_THRES 200
#endif

#ifndef MATEKSYS_3901_L0X_MAX_FLOW
#define MATEKSYS_3901_L0X_MAX_FLOW 300
#endif

#ifndef MATEKSYS_3901_L0X_MAX_DISTANCE
#define MATEKSYS_3901_L0X_MAX_DISTANCE 3000
#endif

#ifndef USE_MATEKSYS_3901_L0X_AGL
#define USE_MATEKSYS_3901_L0X_AGL 1
#endif

#ifndef USE_MATEKSYS_3901_L0X_OPTICAL_FLOW
#define USE_MATEKSYS_3901_L0X_OPTICAL_FLOW 1
#endif

#ifndef MATEKSYS_3901_L0X_COMPENSATE_ROTATION
#define MATEKSYS_3901_L0X_COMPENSATE_ROTATION 1
#endif

#ifndef MATEKSYS_3901_L0X_FLOW_X_SCALER
#define MATEKSYS_3901_L0X_FLOW_X_SCALER 1
#endif

#ifndef MATEKSYS_3901_L0X_FLOW_Y_SCALER
#define MATEKSYS_3901_L0X_FLOW_Y_SCALER 1
#endif

struct Mateksys3901l0X mateksys3901l0x = {
  .parse_status = MATEKSYS_3901_L0X_INITIALIZE
};

static void mateksys3901l0x_parse(uint8_t byte);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/**
 * Downlink message flow and lidar (included velocity estimation, not yet tested)
 */
static void mateksys3901l0x_send_optical_flow(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_OPTICAL_FLOW(trans, dev, AC_ID,
                              &mateksys3901l0x.time_usec,
                              &mateksys3901l0x.sensor_id,
                              &mateksys3901l0x.motionX,
                              &mateksys3901l0x.motionY,
                              &mateksys3901l0x.velocityX,
                              &mateksys3901l0x.velocityY,
                              &mateksys3901l0x.motion_quality,
                              &mateksys3901l0x.distancemm,
                              &mateksys3901l0x.distance_compensated,
                              &mateksys3901l0x.distancemm_quality);
}

#endif

/**
 * Initialization function
 */
void mateksys3901l0x_init(void)
{
  mateksys3901l0x.device = &((MATEKSYS_3901_L0X_PORT).device);
  mateksys3901l0x.parse_crc = 0;
  mateksys3901l0x.motion_quality = 0;    
  mateksys3901l0x.motionX_temp = 0;                                         
  mateksys3901l0x.motionX = 0;   
  mateksys3901l0x.motionY_temp = 0;                                                 
  mateksys3901l0x.motionY = 0;                                                
  mateksys3901l0x.distancemm_quality = 0;
  mateksys3901l0x.distancemm_temp = 0;
  mateksys3901l0x.distancemm = 0;
  mateksys3901l0x.distance_compensated = 0;
  mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
  mateksys3901l0x.scaler_x = MATEKSYS_3901_L0X_FLOW_X_SCALER;
  mateksys3901l0x.scaler_y = MATEKSYS_3901_L0X_FLOW_Y_SCALER;

#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW, mateksys3901l0x_send_optical_flow);
#endif
}

/**
 * Scale the Flow X 
 */
void mateksys_3901_l0x_scale_X(float scalex)
{
  mateksys3901l0x.scaler_x = scalex;
}

/**
 * Scale the Flow Y
 */
void mateksys_3901_l0x_scale_Y(float scaley)
{
  mateksys3901l0x.scaler_y = scaley;
}

/**
 * Receive bytes from the UART port and parse them
 */
void mateksys3901l0x_event(void)
{
  while (mateksys3901l0x.parse_status != MATEKSYS_3901_L0X_INITIALIZE && mateksys3901l0x.device->char_available(mateksys3901l0x.device->periph)) {
    mateksys3901l0x_parse(mateksys3901l0x.device->get_byte(mateksys3901l0x.device->periph));
  }
}

/**
 * Parse the sensor MSP output bytes 1 by 1
 */
static void mateksys3901l0x_parse(uint8_t byte)
{

  switch (mateksys3901l0x.parse_status) {
    case MATEKSYS_3901_L0X_INITIALIZE:
      break;

    case MATEKSYS_3901_L0X_PARSE_HEAD: // MSP general header $
      if (byte == 0x24) {
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_HEAD2: // MSPv2 identifier X
      if (byte == 0x58) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DIRECTION: // direction <
      if (byte == 0x3C) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_LENGTH: // set to 0
      if (byte == 0x00) {
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B1: // 0x01 = rangefinder; 0x02 = opticalflow
      if (byte == 0x01 || byte == 0x02) { 
        mateksys3901l0x.sensor_id = byte;
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B2: // sensor id pointer
      if (byte == 0x1F) { 
        mateksys3901l0x.parse_status++;
        mateksys3901l0x.parse_crc += byte;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_SIZE: // two fixed sizes are expected if message is
      if (byte == 0x05 || byte == 0x09) { 
        mateksys3901l0x.parse_status++;
        mateksys3901l0x.parse_crc += byte;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_POINTER:  // should be zero, used to redirect to motion parsing or lidar parsing
      if (mateksys3901l0x.sensor_id == 0x01) { 
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY;
      } else if (mateksys3901l0x.sensor_id == 0x02) {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    // rangefinder data parsing
    case MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY:
      mateksys3901l0x.distancemm_quality = byte;
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_DISTANCE_B1:
      if (mateksys3901l0x.distancemm_quality <= MATEKSYS_3901_L0X_DISTANCE_THRES) {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      } else {
        mateksys3901l0x.distancemm_temp = byte;
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DISTANCE_B2:
      mateksys3901l0x.distancemm_temp |= (byte << 8);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

		case MATEKSYS_3901_L0X_PARSE_DISTANCE_B3:
      mateksys3901l0x.distancemm_temp |= (byte << 16);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_DISTANCE_B4:
      mateksys3901l0x.distancemm_temp |= (byte << 24);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_CHECKSUM;
      break;

    // optical flow data parsing
    case MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY:
      mateksys3901l0x.motion_quality = byte;
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

	  case MATEKSYS_3901_L0X_PARSE_MOTIONY_B1:
      if (mateksys3901l0x.motion_quality <= MATEKSYS_3901_L0X_MOTION_THRES) {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      } else {
        mateksys3901l0x.motionY_temp = byte;
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONY_B2:
      mateksys3901l0x.motionY_temp |= (byte << 8);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

		case MATEKSYS_3901_L0X_PARSE_MOTIONY_B3:
      mateksys3901l0x.motionY_temp |= (byte << 16);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONY_B4:
      mateksys3901l0x.motionY_temp |= (byte << 24);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONX_B1:
      mateksys3901l0x.motionX_temp = byte;
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONX_B2:
      mateksys3901l0x.motionX_temp |= (byte << 8);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

		case MATEKSYS_3901_L0X_PARSE_MOTIONX_B3:
      mateksys3901l0x.motionX_temp |= (byte << 16);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;
		
		case MATEKSYS_3901_L0X_PARSE_MOTIONX_B4:
      mateksys3901l0x.motionX_temp |= (byte << 24);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_CHECKSUM:

      // When the distance and motion info are valid (max values based on sensor specifications)...
      if (mateksys3901l0x.distancemm_temp > 0 && mateksys3901l0x.distancemm_temp <= MATEKSYS_3901_L0X_MAX_DISTANCE && abs(mateksys3901l0x.motionX_temp) <= MATEKSYS_3901_L0X_MAX_FLOW && abs(mateksys3901l0x.motionY_temp) <= MATEKSYS_3901_L0X_MAX_FLOW) {
        
        // pass temporary message and apply calibration parameters
        mateksys3901l0x.motionX = mateksys3901l0x.motionX_temp * mateksys3901l0x.scaler_x;
        mateksys3901l0x.motionY = mateksys3901l0x.motionY_temp * mateksys3901l0x.scaler_y;
        mateksys3901l0x.distancemm = mateksys3901l0x.distancemm_temp;
        
        // get from ground distance to altitude by compensating for body rotation 
        if (MATEKSYS_3901_L0X_COMPENSATE_ROTATION) {

          float phi = stateGetNedToBodyEulers_f()->phi;
          float theta = stateGetNedToBodyEulers_f()->theta;
          mateksys3901l0x.distance_compensated = ((mateksys3901l0x.distancemm * cos(phi)) * cos(theta)) * 0.001;

        }

        // estimate velocity and send it to telemetry (flow not compensated for gyro measurements)
        mateksys3901l0x.velocityX = mateksys3901l0x.distance_compensated * sin(RadOfDeg(mateksys3901l0x.motionY));  // velocity in m/sec
        mateksys3901l0x.velocityY = mateksys3901l0x.distance_compensated * sin(RadOfDeg(mateksys3901l0x.motionX));  // velocity in m/sec

        // get ticks
        mateksys3901l0x.time_usec = get_sys_time_usec();

        // send AGL (if requested)
        if (USE_MATEKSYS_3901_L0X_AGL) {
          AbiSendMsgAGL(AGL_LIDAR_MATEKSYS_3901_L0X_ID, 
                        mateksys3901l0x.time_usec, 
                        mateksys3901l0x.distance_compensated);
        }

        // send optical flow (if requested)
        if (USE_MATEKSYS_3901_L0X_OPTICAL_FLOW) {
          AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_MATEKSYS_3901_L0X_ID, 
                                mateksys3901l0x.time_usec, 
                                mateksys3901l0x.motionX,          // motion in deg/sec
                                mateksys3901l0x.motionY,          // motion in deg/sec
                                0,
                                0,
                                mateksys3901l0x.motion_quality,
                                0.0);
        }
      }
      // Start reading again
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      break;

    default:
      // Error, return to start
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      break;

 }   
}

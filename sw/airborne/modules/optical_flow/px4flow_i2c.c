/*
 * Copyright (C) 2013 Gautier Hattenberger
 * 2016 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** @file modules/optical_flow/px4flow.c
 *  @brief driver for the optical flow sensor PX4FLOW over i2c
 *
 *  Sensor from the PIXHAWK project
 */

#include "modules/optical_flow/px4flow_i2c.h"
#include "subsystems/abi.h"
#include "filters/median_filter.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

/*
 * acceptable quality of optical flow (0-min,1-max)
 * higher threshold means fewer measurements will be available
 * but they will be more precise. Generally it is probably a better
 * idea to have lower threshold, and propagate the noise (1-quality)
 * to the INS
 */
#ifndef PX4FLOW_QUALITY_THRESHOLD
#define PX4FLOW_QUALITY_THRESHOLD 0.1
#endif

struct px4flow_data px4flow;

struct MedianFilterInt sonar_filter;

#define PX4FLOW_I2C_FRAME 0x0
#define PX4FLOW_I2C_INTEGRAL_FRAME 0x16
#define PX4FLOW_I2C_FRAME_LENGTH 22
#define PX4FLOW_I2C_INTEGRAL_FRAME_LENGTH 25
#define PX4FLOW_I2C_ID 0x4D

/**
 * Propagate optical flow information
 */
static inline void px4flow_i2c_frame_cb(void)
{
  static float quality = 0;
  static float noise = 0;
  uint32_t now_ts = get_sys_time_usec();
  quality = ((float)px4flow.i2c_frame.qual) / 255.0;
  noise = px4flow.stddev + (1 - quality) * px4flow.stddev * 10;
  noise = noise * noise; // square the noise to get variance of the measurement

  static float timestamp = 0;
  static uint32_t time_usec = 0;

  static float flow_comp_m_x = 0.0;
  static float flow_comp_m_y = 0.0;

  if (quality > PX4FLOW_QUALITY_THRESHOLD) {
    timestamp = get_sys_time_float();
    time_usec = (uint32_t)(timestamp * 1e6);

    flow_comp_m_x = ((float)px4flow.i2c_frame.flow_comp_m_x) / 1000.0;
    flow_comp_m_y = ((float)px4flow.i2c_frame.flow_comp_m_y) / 1000.0;

    // flip the axis (if the PX4FLOW is mounted as shown in
    // https://pixhawk.org/modules/px4flow
    AbiSendMsgVELOCITY_ESTIMATE(VEL_PX4FLOW_ID,
                                time_usec,
                                flow_comp_m_y,
                                flow_comp_m_x,
                                0.0f,
                                noise,
                                noise,
                                -1.f);
  }

  // distance is always positive - use median filter to remove outliers
  static int32_t ground_distance = 0;
  static float ground_distance_float = 0.0;

  // update filter
  ground_distance = update_median_filter_i(&sonar_filter, (int32_t)px4flow.i2c_frame.ground_distance);
  ground_distance_float = ((float)ground_distance) / 1000.0;

  // compensate AGL measurement for body rotation
  if (px4flow.compensate_rotation) {
      float phi = stateGetNedToBodyEulers_f()->phi;
      float theta = stateGetNedToBodyEulers_f()->theta;
      float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));
      ground_distance_float = ground_distance_float / gain;
  }

  if (px4flow.update_agl) {
    AbiSendMsgAGL(AGL_SONAR_PX4FLOW_ID, now_ts, ground_distance_float);
  }
}



/**
 * Propagate itegral frame
 */
static inline void px4flow_i2c_int_frame_cb(void)
{

}


/**
 * Initialization function
 */
void px4flow_i2c_init(void)
{
  px4flow.trans.status = I2CTransDone;
  px4flow.addr = PX4FLOW_I2C_ADDR;
#if REQUEST_INT_FRAME
  px4flow.status = PX4FLOW_INT_FRAME_REQ;
#else
  px4flow.status = PX4FLOW_FRAME_REQ;
#endif
  px4flow.update_agl = USE_PX4FLOW_AGL;
  px4flow.compensate_rotation = PX4FLOW_COMPENSATE_ROTATION;
  px4flow.stddev = PX4FLOW_NOISE_STDDEV;

  init_median_filter_i(&sonar_filter, PX4FLOW_MEDIAN_LENGTH);
}

/**
 * Poll px4flow for data
 * 152 i2c frames are created per second, so the PX4FLOW can be polled
 * at up to 150Hz (faster rate won't bring any finer resolution)
 */
void px4flow_i2c_periodic(void)
{
  switch (px4flow.status) {
    case PX4FLOW_FRAME_REQ:
      // ask for i2c frame
      px4flow.trans.buf[0] = PX4FLOW_I2C_FRAME;
      if (i2c_transceive(&PX4FLOW_I2C_DEV, &px4flow.trans, px4flow.addr, 1, PX4FLOW_I2C_FRAME_LENGTH)) {
        // transaction OK, increment status
        px4flow.status = PX4FLOW_FRAME_REC;
      }
      break;
    case PX4FLOW_FRAME_REC:
      // check if the transaction was successful
      if (px4flow.trans.status == I2CTransSuccess) {
        // retrieve data
        uint8_t idx = 0;
        px4flow.i2c_frame.frame_count = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(uint16_t);
        px4flow.i2c_frame.pixel_flow_x_sum = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.pixel_flow_y_sum = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.flow_comp_m_x = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.flow_comp_m_y = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.qual = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.gyro_x_rate = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.gyro_y_rate = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.gyro_z_rate = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_frame.gyro_range = px4flow.trans.buf[idx];
        idx += sizeof(uint8_t);
        px4flow.i2c_frame.sonar_timestamp = px4flow.trans.buf[idx];
        idx += sizeof(uint8_t);
        px4flow.i2c_frame.ground_distance = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);

        // propagate measurements
        px4flow_i2c_frame_cb();
      }
      // increment status
      // ask for regular frame again
      px4flow.status = PX4FLOW_FRAME_REQ;
      break;
    case PX4FLOW_INT_FRAME_REQ:
      // Send the command to begin a measurement.
      px4flow.trans.buf[0] = PX4FLOW_I2C_INTEGRAL_FRAME;
      if (i2c_transmit(&PX4FLOW_I2C_DEV, &px4flow.trans, px4flow.addr, 1)) {
        // transaction OK, increment status
        px4flow.status = PX4FLOW_INT_FRAME_REC;
      }
      break;
    case PX4FLOW_INT_FRAME_REC:
      // ask for integral frame
      px4flow.trans.buf[0] = PX4FLOW_I2C_INTEGRAL_FRAME;
      if (i2c_transceive(&PX4FLOW_I2C_DEV, &px4flow.trans, px4flow.addr, 1, PX4FLOW_I2C_INTEGRAL_FRAME_LENGTH)) {
        // transaction OK, increment status
        px4flow.status = PX4FLOW_INT_FRAME_REC_OK;
      }
      break;
    case PX4FLOW_INT_FRAME_REC_OK:
      // check if the transaction was successful
      if (px4flow.trans.status == I2CTransSuccess) {
        // retrieve data
        uint8_t idx = 0;
        px4flow.i2c_int_frame.frame_count_since_last_readout = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(uint16_t);
        px4flow.i2c_int_frame.pixel_flow_x_integral = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_int_frame.pixel_flow_y_integral = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(uint16_t);
        px4flow.i2c_int_frame.gyro_x_rate_integral = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_int_frame.gyro_y_rate_integral = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_int_frame.gyro_z_rate_integral = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_int_frame.integration_timespan = (px4flow.trans.buf[idx + 3] << 24 | px4flow.trans.buf[idx + 2] << 16
            | px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(uint32_t);
        px4flow.i2c_int_frame.sonar_timestamp = (px4flow.trans.buf[idx + 3] << 24 | px4flow.trans.buf[idx + 2] << 16
                                                | px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(uint32_t);
        px4flow.i2c_int_frame.ground_distance = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_int_frame.gyro_temperature = (px4flow.trans.buf[idx + 1] << 8 | px4flow.trans.buf[idx]);
        idx += sizeof(int16_t);
        px4flow.i2c_int_frame.qual = px4flow.trans.buf[idx];

        // propagate measurements
        px4flow_i2c_int_frame_cb();
      }
      // increment status
      px4flow.status = PX4FLOW_INT_FRAME_REQ;
      break;
    default:
      break;
  }
}



/**
 * Downlink message for debug
 * Copy volatile variables from the px4_i2c_frame for safety
 */
void px4flow_i2c_downlink(void)
{
  // Convert i2c_frame into PX4FLOW message
  uint8_t id = PX4FLOW_I2C_ID;

  float timestamp = get_sys_time_float();
  static float distance_quality = 0;

#if REQUEST_INT_FRAME
  int32_t flow_x = px4flow.i2c_int_frame.pixel_flow_x_integral;
  int32_t flow_y = px4flow.i2c_int_frame.pixel_flow_y_integral;

  float flow_comp_m_x = 0.0;
  float flow_comp_m_y = 0.0;

  uint8_t quality = px4flow.i2c_int_frame.qual;
  float ground_distance = ((float)px4flow.i2c_int_frame.ground_distance) / 1000.0;
#else
  int32_t flow_x = px4flow.i2c_frame.pixel_flow_x_sum;
  int32_t flow_y = px4flow.i2c_frame.pixel_flow_y_sum;

  float flow_comp_m_x = ((float)px4flow.i2c_frame.flow_comp_m_x) / 1000.0;
  float flow_comp_m_y = ((float)px4flow.i2c_frame.flow_comp_m_y) / 1000.0;

  uint8_t quality = px4flow.i2c_frame.qual;
  float ground_distance = ((float)px4flow.i2c_frame.ground_distance) / 1000.0;
#endif

  DOWNLINK_SEND_OPTICAL_FLOW(DefaultChannel, DefaultDevice,
                            &timestamp,
                            &id,
                            &flow_x,
                            &flow_y,
                            &flow_comp_m_x,
                            &flow_comp_m_y,
                            &quality,
                            &ground_distance,
                            &distance_quality);
}

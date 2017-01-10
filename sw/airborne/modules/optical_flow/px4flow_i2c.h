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

/** @file modules/optical_flow/px4flow.h
 *  @brief driver for the optical flow sensor PX4FLOW
 *
 *  Sensor from the PIXHAWK project
 */

#ifndef PX4FLOW_I2C_H
#define PX4FLOW_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"

enum Px4FlowStatus {
  PX4FLOW_FRAME_REQ,
  PX4FLOW_FRAME_REC,
  PX4FLOW_INT_FRAME_REQ,
  PX4FLOW_INT_FRAME_REC,
  PX4FLOW_INT_FRAME_REC_OK
};

struct px4flow_i2c_frame
{
    uint16_t frame_count;// counts created I2C frames [#frames]
    int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
    int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
    int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
    int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
    int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
    int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
    int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
    int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
    uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
    uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
};

struct px4flow_i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t qual;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
};

struct px4flow_data
{
  struct px4flow_i2c_frame i2c_frame;
  struct px4flow_i2c_integral_frame i2c_int_frame;
  struct i2c_transaction trans;
  uint8_t addr;
  float stddev;
  enum Px4FlowStatus status;
  bool update_agl;
  bool compensate_rotation;
};

extern struct px4flow_data px4flow;

extern void px4flow_i2c_init(void);
extern void px4flow_i2c_periodic(void);
extern void px4flow_i2c_downlink(void);


#endif /* PX4FLOW_I2C_H */


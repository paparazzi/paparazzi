/*
 * Copyright (C) 2016 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** @file modules/lidar/lidar_lite.c
 *  @brief driver for the Lidar-Lite i2c lidar version 1 (silver label)
 *
 *  Note that the version 1 (silver label) seems to generate unexpected events
 *  on the i2c bus, such as misplaced start or stop or word reset (see I2C_ERRORS message).
 *  It seems to have no effect on other i2c devices (especially the IMU), but
 *  use with caution.
 *
 *  The newer versions function correctly.
 *
 */
#include "modules/lidar/lidar_lite.h"
#include "subsystems/abi.h"
#include "filters/median_filter.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

struct LidarLite lidar_lite;
struct MedianFilterInt lidar_lite_filter;

#define LIDAR_LITE_REG_ADDR 0x00
#define LIDAR_LITE_REG_VAL 0x04
#define LIDAR_LITE_READ_ADDR 0x8F


/**
 * Initialization function
 */
void lidar_lite_init(void)
{
  lidar_lite.trans.status = I2CTransDone;
  lidar_lite.addr = LIDAR_LITE_I2C_ADDR;
  lidar_lite.status = LIDAR_LITE_INIT_RANGING;
  lidar_lite.update_agl = USE_LIDAR_LITE_AGL;
  lidar_lite.compensate_rotation = LIDAR_LITE_COMPENSATE_ROTATION;

  lidar_lite.distance = 0;
  lidar_lite.distance_raw = 0;

  init_median_filter_i(&lidar_lite_filter, LIDAR_LITE_MEDIAN_LENGTH);
}

/**
 * Lidar event function
 * Basically just check the progress of the transation
 * to prevent overruns during high speed operation
 * (ie. polling the radar at >100Hz)
 */
void lidar_lite_event(void)
{
  switch (lidar_lite.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      lidar_lite.trans.status = I2CTransDone;
      break;
    case I2CTransFailed:
      // set to done
      lidar_lite.trans.status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
}

/**
 * Poll lidar for data
 * for altitude hold 50Hz-100Hz should be enough,
 * in theory can go faster (see the datasheet for Lidar Lite v3
 */
void lidar_lite_periodic(void)
{
  switch (lidar_lite.status) {
    case LIDAR_LITE_INIT_RANGING:
      if (lidar_lite.trans.status == I2CTransDone) {
        // ask for i2c frame
        lidar_lite.trans.buf[0] = LIDAR_LITE_REG_ADDR; // sets register pointer to  (0x00)
        lidar_lite.trans.buf[1] = LIDAR_LITE_REG_VAL; // take acquisition & correlation processing with DC correction
        if (i2c_transmit(&LIDAR_LITE_I2C_DEV, &lidar_lite.trans, lidar_lite.addr, 2)){
          // transaction OK, increment status
          lidar_lite.status = LIDAR_LITE_REQ_READ;
        }
      }
      break;
    case LIDAR_LITE_REQ_READ:
      if (lidar_lite.trans.status == I2CTransDone) {
        lidar_lite.trans.buf[0] = LIDAR_LITE_READ_ADDR; // sets register pointer to results register
        if (i2c_transmit(&LIDAR_LITE_I2C_DEV, &lidar_lite.trans, lidar_lite.addr, 1)){
          // transaction OK, increment status
          lidar_lite.status = LIDAR_LITE_READ_DISTANCE;
        }
      }
      break;
    case LIDAR_LITE_READ_DISTANCE:
      if (lidar_lite.trans.status == I2CTransDone) {
        // clear buffer
        lidar_lite.trans.buf[0] = 0;
        lidar_lite.trans.buf[1] = 0;
        if (i2c_receive(&LIDAR_LITE_I2C_DEV, &lidar_lite.trans, lidar_lite.addr, 2)){
          // transaction OK, increment status
          lidar_lite.status = LIDAR_LITE_PARSE;
        }
      }
      break;
    case LIDAR_LITE_PARSE: {
      // filter data
      uint32_t now_ts = get_sys_time_usec();
      lidar_lite.distance_raw = update_median_filter_i(
                                  &lidar_lite_filter,
                                  (uint32_t)((lidar_lite.trans.buf[0] << 8) | lidar_lite.trans.buf[1]));
      lidar_lite.distance = ((float)lidar_lite.distance_raw)/100.0;

      // compensate AGL measurement for body rotation
      if (lidar_lite.compensate_rotation) {
          float phi = stateGetNedToBodyEulers_f()->phi;
          float theta = stateGetNedToBodyEulers_f()->theta;
          float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));
          lidar_lite.distance = lidar_lite.distance / gain;
      }

      // send message (if requested)
      if (lidar_lite.update_agl) {
        AbiSendMsgAGL(AGL_LIDAR_LITE_ID, now_ts, lidar_lite.distance);
      }

      // increment status
      lidar_lite.status = LIDAR_LITE_INIT_RANGING;
      break;
    }
    default:
      break;
  }
}

/**
 * Downlink message for debug
 */
void lidar_lite_downlink(void)
{
  uint8_t trans = lidar_lite.trans.status;
  uint8_t status = lidar_lite.status;
  DOWNLINK_SEND_LIDAR(DefaultChannel, DefaultDevice,
      &lidar_lite.distance,
      &status,
      &trans);
}

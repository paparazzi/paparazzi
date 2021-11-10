/*
 * Copyright (C) 2017 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** @file modules/lidar/lidar_sf11.h
 *  @brief driver for the Parallax SF11-A/B/C Laser Rangefinder connected over i2c bus.
 */
#include "modules/lidar/lidar_sf11.h"
#include "modules/core/abi.h"
#include "filters/median_filter.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

struct LidarSF11 lidar_sf11;
struct MedianFilterInt lidar_sf11_filter;


/**
 * Initialization function
 */
void lidar_sf11_init(void)
{
  lidar_sf11.trans.status = I2CTransDone;
  lidar_sf11.addr = LIDAR_SF11_I2C_ADDR;
  lidar_sf11.status = LIDAR_SF11_REQ_READ;
  lidar_sf11.update_agl = USE_LIDAR_SF11_AGL;
  lidar_sf11.compensate_rotation = LIDAR_SF11_COMPENSATE_ROTATION;

  lidar_sf11.distance = 0;
  lidar_sf11.distance_raw = 0;

  init_median_filter_i(&lidar_sf11_filter, MEDIAN_DEFAULT_SIZE);
}

/**
 * Lidar event function
 * Check if the transaction succeded before reading the result
 */
void lidar_sf11_event(void)
{
  switch (lidar_sf11.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      lidar_sf11.trans.status = I2CTransDone;
      // increment status
      lidar_sf11.status = LIDAR_SF11_READ_OK;
      break;
    case I2CTransFailed:
      // set to done
      lidar_sf11.trans.status = I2CTransDone;
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
 */
void lidar_sf11_periodic(void)
{
  switch (lidar_sf11.status) {
    case LIDAR_SF11_REQ_READ:
      // read two bytes
      lidar_sf11.trans.buf[0] = 0; // set tx to zero
      i2c_transceive(&LIDAR_SF11_I2C_DEV, &lidar_sf11.trans, lidar_sf11.addr, 1, 2);
      break;
    case LIDAR_SF11_READ_OK: {
      // process results
      // filter data
      uint32_t now_ts = get_sys_time_usec();
      lidar_sf11.distance_raw = update_median_filter_i(
                                  &lidar_sf11_filter,
                                  (uint32_t)((lidar_sf11.trans.buf[0] << 8) | lidar_sf11.trans.buf[1]));
      lidar_sf11.distance = ((float)lidar_sf11.distance_raw)/100.0;

      // compensate AGL measurement for body rotation
      if (lidar_sf11.compensate_rotation) {
          float phi = stateGetNedToBodyEulers_f()->phi;
          float theta = stateGetNedToBodyEulers_f()->theta;
          float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));
          lidar_sf11.distance = lidar_sf11.distance / gain;
      }

      // send message (if requested)
      if (lidar_sf11.update_agl) {
        AbiSendMsgAGL(AGL_LIDAR_SF11_ID, now_ts, lidar_sf11.distance);
      }

      // reset status
      lidar_sf11.status = LIDAR_SF11_REQ_READ;
      break;
    }
    default:
      break;
  }
}

/**
 * Downlink message for debug
 */
void lidar_sf11_downlink(void)
{
  uint8_t trans = lidar_sf11.trans.status;
  uint8_t status = lidar_sf11.status;
  DOWNLINK_SEND_LIDAR(DefaultChannel, DefaultDevice,
      &lidar_sf11.distance,
      &status,
      &trans);
}

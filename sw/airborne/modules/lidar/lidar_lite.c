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
 *  Note that the lidar seems to generate unexpected events on the i2c bus,
 *  such as misplaced start or stop or word reset (see I2C_ERRORS message).
 *  It seems to have no effect on other i2c devices (especially the IMU), but
 *  use with caution.
 *
 */
#include "modules/lidar/lidar_lite.h"
#include "subsystems/abi.h"
#include "filters/median_filter.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


struct lidar_lite lidar;

struct MedianFilterInt lidar_filter;

#define LIDAR_LITE_REG_ADDR 0x00
#define LIDAR_LITE_REG_VAL 0x04
#define LIDAR_LITE_READ_ADDR 0x8F


/**
 * Initialization function
 */
void lidar_lite_init(void)
{
  lidar.trans.status = I2CTransDone;
  lidar.addr = LIDAR_LITE_I2C_ADDR;
  lidar.status = LIDAR_INIT_RANGING;
  lidar.update_agl = USE_LIDAR_LITE_AGL;

  init_median_filter(&lidar_filter);
}

/**
 * Poll lidar for data
 * run at max 50Hz
 */
void lidar_lite_periodic(void)
{
  switch (lidar.status) {
    case LIDAR_INIT_RANGING:
      // ask for i2c frame
      lidar.trans.buf[0] = LIDAR_LITE_REG_ADDR; // sets register pointer to  (0x00)
      lidar.trans.buf[1] = LIDAR_LITE_REG_VAL; // take acquisition & correlation processing with DC correction
      if (i2c_transmit(&LIDAR_LITE_I2C_DEV, &lidar.trans, lidar.addr, 2)){
        // transaction OK, increment status
        lidar.status = LIDAR_REQ_READ;
      }
      break;
    case LIDAR_REQ_READ:
      lidar.trans.buf[0] = LIDAR_LITE_READ_ADDR; // sets register pointer to results register
      lidar.trans.buf[1] = 0;
      if (i2c_transceive(&LIDAR_LITE_I2C_DEV, &lidar.trans, lidar.addr, 1,2)){
        // transaction OK, increment status
        lidar.status = LIDAR_READ_DISTANCE;
      }
      break;
    case LIDAR_READ_DISTANCE:
      // filter data
      lidar.distance_raw = update_median_filter(&lidar_filter, (uint32_t)((lidar.trans.buf[0] << 8) | lidar.trans.buf[1]));
      lidar.distance = ((float)lidar.distance_raw)/100.0;

      // send message (if requested)
      if (lidar.update_agl) {
        AbiSendMsgAGL(AGL_LIDAR_LITE_ID, lidar.distance);
      }

      // increment status
      lidar.status = LIDAR_INIT_RANGING;
      break;
    default:
      break;
  }
}

/**
 * Downlink message for debug
 */
void lidar_lite_downlink(void)
{
  uint8_t trans = lidar.trans.status;
  DOWNLINK_SEND_LIDAR(DefaultChannel, DefaultDevice,
      &lidar.distance,
      &lidar.status,
      &trans);
}

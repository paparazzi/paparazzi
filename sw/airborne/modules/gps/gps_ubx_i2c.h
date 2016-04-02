/*
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz,Gautier Hattenberger,
 *               2016 Michael Sierra <sierramichael.a@gmail.com>
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

/**
 * @file modules/gps/gps_ubx_i2c.h
 * pprz link device for Ublox over I2C
 *
 * This module adds i2c functionality for the ublox using existing
 * driver
 */

#ifndef GPS_UBX_I2C_H
#define GPS_UBX_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "pprzlink/pprzlink_device.h"

#define GPS_I2C_BUF_SIZE 255

/** read states
 */
typedef enum GpsI2CReadState
{
  gps_i2c_read_standby,     ///< dont read anything
  gps_i2c_read_sizeof,      ///< read size of ubx buffer
  gps_i2c_read_data         ///< read data from ubx buffer
} GpsI2CReadState;

/** write states
 */
typedef enum GpsI2CWriteState
{
  gps_i2c_write_standby,        ///< wait for gps_ubx to read buffer or ucenter to transmit
  gps_i2c_write_request_size,   ///< request size of ubx buffer
  gps_i2c_write_cfg             ///< send a config msg and get reply
} GpsI2CWriteState;

/** ubx_i2c state
 */
struct GpsUbxI2C
{
  GpsI2CReadState read_state;
  GpsI2CWriteState write_state;

  uint8_t rx_buf[GPS_I2C_BUF_SIZE];     ///< receive buffer
  uint8_t tx_buf[GPS_I2C_BUF_SIZE];     ///< transmit buffer

  uint16_t rx_buf_avail;                ///< how many bytes are waiting to be read
  uint16_t rx_buf_idx;                  ///< rx buf index
  uint16_t tx_buf_idx;                  ///< tx buf index

  bool tx_rdy;                        ///< are we ready to transmit

  struct i2c_transaction trans;         ///< i2c transaction

  int baudrate;                         ///< baudrate, unused

  struct link_device device;            ///< ppz link device
};

extern struct GpsUbxI2C gps_i2c;
/** init function
 */
extern void gps_ubx_i2c_init(void);
/** handle message sending
 */
extern void gps_ubx_i2c_periodic(void);
/** handle message reception
 */
extern void gps_ubx_i2c_read_event(void);
/** is driver ready to send a message
 */
extern bool gps_i2c_tx_is_ready(void);
/** config is done, begin reading messages
 */
extern void gps_i2c_begin(void);

/** i2c event
 */
static inline void GpsUbxi2cEvent(void)
{
  if (gps_i2c.trans.status == I2CTransSuccess) {
    gps_ubx_i2c_read_event();
  } else if (gps_i2c.trans.status == I2CTransFailed) {
    // if transaction failed, mark as done so can be retried
    gps_i2c.trans.status = I2CTransDone;
  }
}

#endif // GPS_UBX_I2C_H

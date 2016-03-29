/*
 * Copyright (C) 2016 Michael Sierra <sierramichael.a@gmail.com>
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

#ifndef GPS_UBX_I2C_H
#define GPS_UBX_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "pprzlink/pprzlink_device.h"

#define GPS_I2C_BUF_SIZE 255

typedef enum GpsI2cReadState
{
  gps_i2c_read_standby,
  gps_i2c_read_sizeof,
  gps_i2c_read_data,
  gps_i2c_read_ack
} GpsI2cReadState;
 
typedef enum GpsI2cWriteState
{
  gps_i2c_write_standby,
  gps_i2c_write_request_size,
  gps_i2c_write_cfg,
  gps_i2c_write_get_ack
} GpsI2cWriteState;

struct GpsUbxI2C
{
  GpsI2cReadState read_state;
  GpsI2cWriteState write_state;

  uint8_t rx_buf[GPS_I2C_BUF_SIZE];
  uint8_t tx_buf[GPS_I2C_BUF_SIZE];

  uint16_t rx_buf_avail;
  uint16_t rx_buf_idx;
  uint16_t tx_buf_idx;

  bool_t tx_rdy;

  uint16_t bytes_to_read;

  struct i2c_transaction trans;

  int baudrate; //not actually used duhh

  struct link_device device;
};

extern struct GpsUbxI2C gps_i2c;

void gps_i2c_send(void);
extern void gps_ubx_i2c_init(void);
extern void gps_ubx_i2c_periodic(void);
extern void gps_ubx_i2c_read_event(void);
extern bool_t gps_i2c_tx_is_ready(void);
extern void gps_i2c_begin(void);

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
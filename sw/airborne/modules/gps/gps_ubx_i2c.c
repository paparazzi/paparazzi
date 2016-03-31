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
 * @file modules/gps/gps_ubx_i2c.c
 * pprz link device for Ublox over I2C
 *
 * This module adds i2c functionality for the ublox using existing
 * driver
 */

#include "mcu_periph/i2c.h"
#include "modules/gps/gps_ubx_i2c.h"
#include "subsystems/datalink/downlink.h"
#include <math.h>

// ublox i2c address
#define GPS_I2C_SLAVE_ADDR (0x42 << 1)

#ifndef GPS_UBX_I2C_DEV
#define GPS_UBX_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(GPS_UBX_I2C_DEV)

#define GPS_I2C_ADDR_NB_AVAIL_BYTES 0xFD    ///< number of bytes available register
#define GPS_I2C_ADDR_DATA 0xFF            ///< data stream register

// Global variables
struct GpsUbxI2C gps_i2c;

// Local variables
bool   gps_ubx_i2c_ucenter_done;      ///< ucenter finished configuring flag
uint16_t gps_ubx_i2c_bytes_to_read;     ///< ublox bytes to read

/** null function
 * @param p unused
 */
void null_function(struct GpsUbxI2C *p);

/** Check available space in transmit buffer
 * @param p unused
 * @param fd unused
 * @param len number of bytes to check
 */
int gps_i2c_check_free_space(struct GpsUbxI2C *p, long *fd, uint16_t len);

/** Put byte into transmit buffer.
 * @param p unused
 * @param fd unused
 * @param data byte to put in buffer
 */
void gps_i2c_put_byte(struct GpsUbxI2C *p, long fd, uint8_t data);

/** Put bytes into transmit buffer.
 * @param p unused
 * @param fd unused
 * @param data byte to put in buffer
 */
void gps_i2c_put_buffer(struct GpsUbxI2C *p, long fd, uint8_t *data, uint16_t len);

/** send buffer when ready
 * @param p unused
 */
void gps_i2c_msg_ready(struct GpsUbxI2C *p, long fd);

/** check if a new character is available
 * @param p unused
 */
uint8_t gps_i2c_char_available(struct GpsUbxI2C *p);

/** get a new char
 * @param p unused
 */
uint8_t gps_i2c_getch(struct GpsUbxI2C *p);

void gps_ubx_i2c_init(void)
{
  gps_ubx_i2c_ucenter_done = FALSE;
  gps_i2c.read_state = gps_i2c_read_standby;
  gps_i2c.write_state = gps_i2c_write_standby;

  gps_i2c.trans.status = I2CTransDone;
  gps_i2c.rx_buf_avail = 0;
  gps_i2c.rx_buf_idx = 0;
  gps_i2c.tx_buf_idx = 0;
  gps_i2c.tx_rdy = TRUE;

  gps_i2c.device.periph = (void *)&gps_i2c;
  gps_i2c.device.check_free_space = (check_free_space_t)gps_i2c_check_free_space; ///< check if transmit buffer is not full
  gps_i2c.device.put_byte = (put_byte_t)gps_i2c_put_byte;                         ///< put one byte
  gps_i2c.device.put_buffer = (put_buffer_t)gps_i2c_put_buffer;                   ///< put several bytes
  gps_i2c.device.send_message = (send_message_t)gps_i2c_msg_ready;                ///< send completed buffer
  gps_i2c.device.char_available = (char_available_t)gps_i2c_char_available;       ///< check if a new character is available
  gps_i2c.device.get_byte = (get_byte_t)gps_i2c_getch;                            ///< get a new char
  gps_i2c.device.set_baudrate = (set_baudrate_t)null_function;                    ///< set device baudrate
}

void null_function(struct GpsUbxI2C *p __attribute__((unused))) {}

int gps_i2c_check_free_space(struct GpsUbxI2C *p __attribute__((unused)), long *fd __attribute__((unused)), uint16_t len)
{
  return (GPS_I2C_BUF_SIZE - gps_i2c.tx_buf_idx) >= len;
}

void gps_i2c_put_buffer(struct GpsUbxI2C *p, long fd, uint8_t *data, uint16_t len)
{
  int i = 0;
  for (i = 0; i < len; i++) {
    gps_i2c_put_byte(p, fd, data[i]);
  }
}

void gps_i2c_put_byte(struct GpsUbxI2C *p __attribute__((unused)), long fd __attribute__((unused)), uint8_t data)
{
  gps_i2c.tx_buf[gps_i2c.tx_buf_idx++] = data;
}

void gps_i2c_msg_ready(struct GpsUbxI2C *p __attribute__((unused)), long fd __attribute__((unused)))
{
  gps_i2c.write_state = gps_i2c_write_cfg;
  gps_i2c.tx_rdy = FALSE;
}

uint8_t gps_i2c_char_available(struct GpsUbxI2C *p __attribute__((unused)))
{
  return (((int)gps_i2c.rx_buf_avail - (int)gps_i2c.rx_buf_idx) > 0);
}

bool gps_i2c_tx_is_ready(void)
{
  return gps_i2c.tx_rdy;
}

void gps_i2c_begin(void)
{
  gps_ubx_i2c_ucenter_done = TRUE;
}

uint8_t gps_i2c_getch(struct GpsUbxI2C *p __attribute__((unused)))
{
  return gps_i2c.rx_buf[gps_i2c.rx_buf_idx++];
}

void gps_ubx_i2c_periodic(void)
{
  if (gps_i2c.trans.status == I2CTransDone)
  {
    switch(gps_i2c.write_state)
    {
      case gps_i2c_write_standby:
      if (!gps_i2c_char_available(&gps_i2c))
      {
        if (gps_ubx_i2c_bytes_to_read > GPS_I2C_BUF_SIZE || gps_ubx_i2c_ucenter_done)
        {
          gps_i2c.write_state = gps_i2c_write_request_size;
        } else {
          gps_i2c.tx_rdy = TRUE;
        }
      }
      break;

      case gps_i2c_write_request_size:
        gps_i2c.trans.buf[0] = GPS_I2C_ADDR_NB_AVAIL_BYTES;
        i2c_transceive(&GPS_UBX_I2C_DEV, &gps_i2c.trans, GPS_I2C_SLAVE_ADDR, 1, 2);
        gps_i2c.write_state = gps_i2c_write_standby;
        gps_i2c.read_state = gps_i2c_read_sizeof;
      break;

      case gps_i2c_write_cfg:
        memcpy(&gps_i2c.trans.buf, gps_i2c.tx_buf, gps_i2c.tx_buf_idx);
        i2c_transmit(&GPS_UBX_I2C_DEV, &gps_i2c.trans, GPS_I2C_SLAVE_ADDR, gps_i2c.tx_buf_idx);
        gps_i2c.tx_buf_idx = 0;
        gps_i2c.read_state = gps_i2c_read_standby;
        gps_i2c.write_state = gps_i2c_write_request_size;
      break;

      default:
      break;
    }
  }
}

void gps_ubx_i2c_read_event(void)
{
  switch(gps_i2c.read_state)
  {
    case gps_i2c_read_standby:
      break;
    break;

    // how many bytes are available
    case gps_i2c_read_sizeof:
      gps_ubx_i2c_bytes_to_read = ((uint16_t)(gps_i2c.trans.buf[0]) << 7) | (uint16_t)(gps_i2c.trans.buf[1]);
      gps_i2c.trans.status = I2CTransDone;
      if (gps_ubx_i2c_bytes_to_read == 0xFFFF || gps_ubx_i2c_bytes_to_read == 0x0000)
      {
        gps_i2c.write_state = gps_i2c_write_request_size;
        return;
      } else if (gps_ubx_i2c_bytes_to_read > GPS_I2C_BUF_SIZE)
      {
        gps_i2c.trans.buf[0] = GPS_I2C_ADDR_DATA;
        i2c_transceive(&GPS_UBX_I2C_DEV, &gps_i2c.trans, GPS_I2C_SLAVE_ADDR, 1, GPS_I2C_BUF_SIZE);
        gps_i2c.read_state = gps_i2c_read_data;
      } else
      {
        gps_i2c.trans.buf[0] = GPS_I2C_ADDR_DATA;
        i2c_transceive(&GPS_UBX_I2C_DEV, &gps_i2c.trans, GPS_I2C_SLAVE_ADDR, 1, gps_ubx_i2c_bytes_to_read);
        gps_i2c.read_state = gps_i2c_read_data;
        return;
      }
    break;
    case gps_i2c_read_data:
      if (gps_ubx_i2c_bytes_to_read > GPS_I2C_BUF_SIZE)
      {
        memcpy(&gps_i2c.rx_buf, &gps_i2c.trans.buf, GPS_I2C_BUF_SIZE);
        gps_i2c.rx_buf_idx = 0;
        gps_i2c.rx_buf_avail = GPS_I2C_BUF_SIZE;
      } else
      {
        memcpy(&gps_i2c.rx_buf, &gps_i2c.trans.buf, gps_ubx_i2c_bytes_to_read);
        gps_i2c.rx_buf_idx = 0;
        gps_i2c.rx_buf_avail = gps_ubx_i2c_bytes_to_read;
      }

      gps_i2c.write_state = gps_i2c_write_standby;
      gps_i2c.read_state = gps_i2c_read_standby;
    break;

    default:
    break;
  }

  // Transaction has been read
  gps_i2c.trans.status = I2CTransDone;
}

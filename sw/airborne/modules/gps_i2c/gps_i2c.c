/*
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz,Gautier Hattenberger
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

#include "gps_i2c.h"
#include "mcu_periph/i2c.h"
#include "subsystems/gps.h"

struct i2c_transaction i2c_gps_trans;


uint8_t gps_i2c_rx_buf[GPS_I2C_BUF_SIZE];
uint8_t gps_i2c_rx_insert_idx, gps_i2c_rx_extract_idx;
uint8_t gps_i2c_tx_buf[GPS_I2C_BUF_SIZE];
uint8_t gps_i2c_tx_insert_idx, gps_i2c_tx_extract_idx;
bool_t gps_i2c_done, gps_i2c_data_ready_to_transmit;

/* u-blox5 protocole, page 4 */
#define GPS_I2C_ADDR_NB_AVAIL_BYTES 0xFD
#define GPS_I2C_ADDR_DATA 0xFF

#define GPS_I2C_STATUS_IDLE                   0
#define GPS_I2C_STATUS_ASKING_DATA            1
#define GPS_I2C_STATUS_ASKING_NB_AVAIL_BYTES  2
#define GPS_I2C_STATUS_READING_NB_AVAIL_BYTES 3
#define GPS_I2C_STATUS_READING_DATA           4

#define gps_i2c_AddCharToRxBuf(_x) { \
    gps_i2c_rx_buf[gps_i2c_rx_insert_idx] = _x; \
    gps_i2c_rx_insert_idx++; /* size=256, No check for buf overflow */ \
  }

static uint8_t gps_i2c_status;
//static uint16_t gps_i2c_nb_avail_bytes; /* size buffer =~ 12k */
//static uint8_t data_buf_len;

void gps_i2c_init(void)
{
  gps_i2c_status = GPS_I2C_STATUS_IDLE;
  gps_i2c_done = TRUE;
  gps_i2c_data_ready_to_transmit = FALSE;
  gps_i2c_rx_insert_idx = 0;
  gps_i2c_rx_extract_idx = 0;
  gps_i2c_tx_insert_idx = 0;
#ifdef GPS_CONFIGURE
  /* The call in main_ap.c is made before the modules init (too early) */
  gps_configure_uart();
#endif
}

void gps_i2c_periodic(void)
{
  /*
    if (gps_i2c_done && gps_i2c_status == GPS_I2C_STATUS_IDLE) {
      i2c0_buf[0] = GPS_I2C_ADDR_NB_AVAIL_BYTES;
      i2c0_transmit_no_stop(GPS_I2C_SLAVE_ADDR, 1, &gps_i2c_done);
      gps_i2c_done = FALSE;
      gps_i2c_status = GPS_I2C_STATUS_ASKING_NB_AVAIL_BYTES;
    }
  */

}

void gps_i2c_event(void)
{
  /*
   *  switch (gps_i2c_status) {
    case GPS_I2C_STATUS_IDLE:
      if (gps_i2c_data_ready_to_transmit) {
        // Copy data from our buffer to the i2c buffer
        uint8_t data_size = Min(gps_i2c_tx_insert_idx-gps_i2c_tx_extract_idx, I2C0_BUF_LEN);
        uint8_t i;
        for(i = 0; i < data_size; i++, gps_i2c_tx_extract_idx++)
          i2c0_buf[i] = gps_i2c_tx_buf[gps_i2c_tx_extract_idx];

        // Start i2c transmit
        i2c0_transmit(GPS_I2C_SLAVE_ADDR, data_size, &gps_i2c_done);
        gps_i2c_done = FALSE;

        // Reset flag if finished
        if (gps_i2c_tx_extract_idx >= gps_i2c_tx_insert_idx) {
          gps_i2c_data_ready_to_transmit = FALSE;
          gps_i2c_tx_insert_idx = 0;
        }
      }
      break;

    case GPS_I2C_STATUS_ASKING_NB_AVAIL_BYTES:
      i2c0_receive(GPS_I2C_SLAVE_ADDR, 2, &gps_i2c_done);
      gps_i2c_done = FALSE;
      gps_i2c_status = GPS_I2C_STATUS_READING_NB_AVAIL_BYTES;
      break;

    case GPS_I2C_STATUS_READING_NB_AVAIL_BYTES:
      gps_i2c_nb_avail_bytes = (i2c0_buf[0]<<8) | i2c0_buf[1];

      if (gps_i2c_nb_avail_bytes)
        goto continue_reading;
      else
        gps_i2c_status = GPS_I2C_STATUS_IDLE;
      break;

    continue_reading:

    case GPS_I2C_STATUS_ASKING_DATA:
      data_buf_len = Min(gps_i2c_nb_avail_bytes, I2C0_BUF_LEN);
      gps_i2c_nb_avail_bytes -= data_buf_len;

      i2c0_receive(GPS_I2C_SLAVE_ADDR, data_buf_len, &gps_i2c_done);
      gps_i2c_done = FALSE;
      gps_i2c_status = GPS_I2C_STATUS_READING_DATA;
      break;

    case GPS_I2C_STATUS_READING_DATA: {
      uint8_t i;
      for(i = 0; i < data_buf_len; i++) {
        gps_i2c_AddCharToRxBuf(i2c0_buf[i]);
      }

      if (gps_i2c_nb_avail_bytes)
       goto continue_reading;
      else
        gps_i2c_status = GPS_I2C_STATUS_IDLE;
      break;
    }

    default:
      return;
    }
  */

}

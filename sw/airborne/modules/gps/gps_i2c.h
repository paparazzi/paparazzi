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

#ifndef GPS_I2C
#define GPS_I2C

#include "std.h"

/// Default address for u-blox (and others?)
#define GPS_I2C_SLAVE_ADDR (0x42 << 1)

#define GPS_I2C_BUF_SIZE 256
extern uint8_t gps_i2c_rx_buf[GPS_I2C_BUF_SIZE];
extern uint8_t gps_i2c_rx_insert_idx, gps_i2c_rx_extract_idx;
extern uint8_t gps_i2c_tx_buf[GPS_I2C_BUF_SIZE];
extern uint8_t gps_i2c_tx_insert_idx, gps_i2c_tx_extract_idx;

extern bool_t gps_i2c_done, gps_i2c_data_ready_to_transmit;

void gps_i2c_init(void);
void gps_i2c_event(void);
void gps_i2c_periodic(void);

#define gps_i2cEvent() { if (gps_i2c_done) gps_i2c_event(); }
#define gps_i2cChAvailable() (gps_i2c_rx_insert_idx != gps_i2c_rx_extract_idx)
#define gps_i2cGetch() (gps_i2c_rx_buf[gps_i2c_rx_extract_idx++])
#define gps_i2cTransmit(_char) {             \
    if (! gps_i2c_data_ready_to_transmit)  /* Else transmitting, overrun*/     \
      gps_i2c_tx_buf[gps_i2c_tx_insert_idx++] = _char; \
  }
#define gps_i2cSendMessage() {           \
    gps_i2c_data_ready_to_transmit = TRUE; \
    gps_i2c_tx_extract_idx = 0;            \
  }
// #define gps_i2cTxRunning (gps_i2c_data_ready_to_transmit)
// #define gps_i2cInitParam(_baud, _uart_prm1, _uart_prm2) {}

#endif // GPS_I2C

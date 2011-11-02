/*
 * Copyright (C) 2011 Gautier Hattenberger
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

/* driver for MCP3550/1/3
 */

#include "peripherals/mcp355x.h"
#include "mcu_periph/spi.h"

bool_t mcp355x_data_available;
int32_t mcp355x_data;
uint8_t mcp355x_spi_buf[4];

void mcp355x_init(void) {
  mcp355x_data_available = FALSE;
  mcp355x_data = 0;

  SpiClrCPOL();
  SpiClrCPHA();
}

void mcp355x_read(void) {
  spi_buffer_length = 4;
  spi_buffer_input = mcp355x_spi_buf;
  SpiSelectSlave0();
  SpiStart();
}

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

void mcp355x_event(void) {
  static uint32_t filtered = 0;
  if (spi_message_received) {
    spi_message_received = FALSE;
    if ((mcp355x_spi_buf[0]>>4) == 0) {
      //mcp355x_data = (int32_t)(((uint32_t)mcp355x_spi_buf[0]<<16) | ((uint32_t)mcp355x_spi_buf[1]<<8) | (mcp355x_spi_buf[2]));
      mcp355x_data = (int32_t)(
          ((uint32_t)mcp355x_spi_buf[0]<<17) |
          ((uint32_t)mcp355x_spi_buf[1]<<9) |
          ((uint32_t)mcp355x_spi_buf[2]<<1) |
          (mcp355x_spi_buf[3]>>7));
      filtered = (5*filtered + mcp355x_data) / (6);
      DOWNLINK_SEND_DEBUG(DefaultChannel,4,mcp355x_spi_buf);
      DOWNLINK_SEND_BARO_RAW(DefaultChannel,&mcp355x_data,&filtered);
    }
  }
}


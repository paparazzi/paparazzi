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
 */

/**
 * @file peripherals/mcp355x.c
 * @brief Driver for MCP3550/1/3
 */

#include "peripherals/mcp355x.h"
#include "mcu_periph/spi.h"

bool_t mcp355x_data_available;
int32_t mcp355x_data;
uint8_t mcp355x_val[4];

struct spi_transaction mcp355x_spi_trans;

void mcp355x_init(void)
{
  mcp355x_data_available = FALSE;
  mcp355x_data = 0;

  mcp355x_spi_trans.input_length = 4;
  mcp355x_spi_trans.input_buf = mcp355x_val;
  mcp355x_spi_trans.output_length = 0;
  mcp355x_spi_trans.output_buf = NULL;
  mcp355x_spi_trans.slave_idx = SPI_SLAVE0;
  mcp355x_spi_trans.select = SPISelect;

  mcp355x_spi_trans.cpol = SPICpolIdleLow;
  mcp355x_spi_trans.cpha = SPICphaEdge1;
  mcp355x_spi_trans.dss = SPIDss8bit;
  mcp355x_spi_trans.bitorder = SPIMSBFirst;
  mcp355x_spi_trans.cdiv = SPIDiv64;
}

void mcp355x_read(void)
{
  spi_submit(&spi1, &mcp355x_spi_trans);
}

void mcp355x_event(void)
{
  if (mcp355x_spi_trans.status == SPITransSuccess) {
    if ((mcp355x_spi_trans.input_buf[0] >> 4) == 0) {
      mcp355x_data = (int32_t)(
                       ((uint32_t)mcp355x_spi_trans.input_buf[0] << 17) |
                       ((uint32_t)mcp355x_spi_trans.input_buf[1] << 9) |
                       ((uint32_t)mcp355x_spi_trans.input_buf[2] << 1) |
                       (mcp355x_spi_trans.input_buf[3] >> 7));
      mcp355x_data_available = TRUE;
    }
    mcp355x_spi_trans.status = SPITransDone;
  }
  if (mcp355x_spi_trans.status == SPITransFailed) {
    mcp355x_spi_trans.status = SPITransDone;
  }
}


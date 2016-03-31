/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lis302dl_spi.c
 *
 * Driver for LIS302DL 3-axis accelerometer from ST using SPI.
 */

#include "peripherals/lis302dl_spi.h"

void lis302dl_spi_init(struct Lis302dl_Spi *lis, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  lis->spi_p = spi_p;

  /* configure spi transaction */
  lis->spi_trans.cpol = SPICpolIdleHigh;
  lis->spi_trans.cpha = SPICphaEdge2;
  lis->spi_trans.dss = SPIDss8bit;
  lis->spi_trans.bitorder = SPIMSBFirst;
  lis->spi_trans.cdiv = SPIDiv64;

  lis->spi_trans.select = SPISelectUnselect;
  lis->spi_trans.slave_idx = slave_idx;
  lis->spi_trans.output_length = 2;
  lis->spi_trans.input_length = 8;
  // callback currently unused
  lis->spi_trans.before_cb = NULL;
  lis->spi_trans.after_cb = NULL;
  lis->spi_trans.input_buf = &(lis->rx_buf[0]);
  lis->spi_trans.output_buf = &(lis->tx_buf[0]);

  /* set inital status: Success or Done */
  lis->spi_trans.status = SPITransDone;

  /* set default LIS302DL config options */
  lis302dl_set_default_config(&(lis->config));

  lis->initialized = false;
  lis->data_available = false;
  lis->init_status = LIS_CONF_UNINIT;
}


static void lis302dl_spi_write_to_reg(struct Lis302dl_Spi *lis, uint8_t _reg, uint8_t _val)
{
  lis->spi_trans.output_length = 2;
  lis->spi_trans.input_length = 0;
  lis->tx_buf[0] = _reg;
  lis->tx_buf[1] = _val;
  spi_submit(lis->spi_p, &(lis->spi_trans));
}

// Configuration function called once before normal use
static void lis302dl_spi_send_config(struct Lis302dl_Spi *lis)
{
  uint8_t reg_val = 0;

  switch (lis->init_status) {
    case LIS_CONF_WHO_AM_I:
      /* query device id */
      lis->spi_trans.output_length = 1;
      lis->spi_trans.input_length = 2;
      /* set read bit then reg address */
      lis->tx_buf[0] = (1 << 7 | LIS302DL_REG_WHO_AM_I);
      if (spi_submit(lis->spi_p, &(lis->spi_trans))) {
        lis->init_status++;
      }
      break;
    case LIS_CONF_REG2:
      /* set SPI mode, Filtered Data Selection */
      reg_val = (lis->config.spi_3_wire << 7) | (lis->config.filt_data << 4);
      lis302dl_spi_write_to_reg(lis, LIS302DL_REG_CTRL_REG2, reg_val);
      lis->init_status++;
      break;
    case LIS_CONF_REG3:
      /* Interrupt active high/low */
      lis302dl_spi_write_to_reg(lis, LIS302DL_REG_CTRL_REG3, (lis->config.int_invert << 7));
      lis->init_status++;
      break;
    case LIS_CONF_ENABLE:
      /* set data rate, range, enable measurement, is in standby after power up */
      reg_val = (lis->config.rate << 7) |
                (1 << 6) | // Power Down Control to active mode
                (lis->config.range << 5) |
                0x5; // enable z,y,x axes
      lis302dl_spi_write_to_reg(lis, LIS302DL_REG_CTRL_REG1, reg_val);
      lis->init_status++;
      break;
    case LIS_CONF_DONE:
      lis->initialized = true;
      lis->spi_trans.status = SPITransDone;
      break;
    default:
      break;
  }
}

void lis302dl_spi_start_configure(struct Lis302dl_Spi *lis)
{
  if (lis->init_status == LIS_CONF_UNINIT) {
    lis->init_status++;
    if (lis->spi_trans.status == SPITransSuccess || lis->spi_trans.status == SPITransDone) {
      lis302dl_spi_send_config(lis);
    }
  }
}

void lis302dl_spi_read(struct Lis302dl_Spi *lis)
{
  if (lis->initialized && lis->spi_trans.status == SPITransDone) {
    lis->spi_trans.output_length = 1;
    lis->spi_trans.input_length = 8;
    /* set read bit and multiple byte bit, then address */
    lis->tx_buf[0] = (1 << 7 | 1 << 6 | LIS302DL_REG_STATUS);
    spi_submit(lis->spi_p, &(lis->spi_trans));
  }
}

void lis302dl_spi_event(struct Lis302dl_Spi *lis)
{
  if (lis->initialized) {
    if (lis->spi_trans.status == SPITransFailed) {
      lis->spi_trans.status = SPITransDone;
    } else if (lis->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      if (bit_is_set(lis->rx_buf[1], 3)) {
        // new xyz data available
        lis->data.vect.x = lis->rx_buf[3];
        lis->data.vect.y = lis->rx_buf[5];
        lis->data.vect.z = lis->rx_buf[7];
        lis->data_available = true;
      }
      lis->spi_trans.status = SPITransDone;
    }
  } else if (lis->init_status != LIS_CONF_UNINIT) { // Configuring but not yet initialized
    switch (lis->spi_trans.status) {
      case SPITransFailed:
        lis->init_status--; // Retry config (TODO max retry)
      case SPITransSuccess:
        if (lis->init_status == LIS_CONF_WHO_AM_I_OK) {
          if (lis->rx_buf[1] == LIS302DL_WHO_AM_I) {
            lis->init_status++;
          } else {
            lis->init_status = LIS_CONF_WHO_AM_I;
          }
        }
      case SPITransDone:
        lis->spi_trans.status = SPITransDone;
        lis302dl_spi_send_config(lis);
        break;
      default:
        break;
    }
  }
}

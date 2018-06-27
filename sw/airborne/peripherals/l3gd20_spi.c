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
 * @file peripherals/l3gd20_spi.c
 *
 * Driver for L3GD20 3-axis gyroscope from ST using SPI.
 */

#include "peripherals/l3gd20_spi.h"

void l3gd20_spi_init(struct L3gd20_Spi *l3g, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  l3g->spi_p = spi_p;

  /* configure spi transaction */
  l3g->spi_trans.cpol = SPICpolIdleHigh;
  l3g->spi_trans.cpha = SPICphaEdge2;
  l3g->spi_trans.dss = SPIDss8bit;
  l3g->spi_trans.bitorder = SPIMSBFirst;
  l3g->spi_trans.cdiv = SPIDiv64;

  l3g->spi_trans.select = SPISelectUnselect;
  l3g->spi_trans.slave_idx = slave_idx;
  l3g->spi_trans.output_length = 2;
  l3g->spi_trans.input_length = 8;
  // callback currently unused
  l3g->spi_trans.before_cb = NULL;
  l3g->spi_trans.after_cb = NULL;
  l3g->spi_trans.input_buf = &(l3g->rx_buf[0]);
  l3g->spi_trans.output_buf = &(l3g->tx_buf[0]);

  /* set inital status: Success or Done */
  l3g->spi_trans.status = SPITransDone;

  /* set default L3GD20 config options */
  l3gd20_set_default_config(&(l3g->config));

  l3g->initialized = false;
  l3g->data_available = false;
  l3g->init_status = L3G_CONF_UNINIT;
}


static void l3gd20_spi_write_to_reg(struct L3gd20_Spi *l3g, uint8_t _reg, uint8_t _val)
{
  l3g->spi_trans.output_length = 2;
  l3g->spi_trans.input_length = 0;
  l3g->tx_buf[0] = _reg;
  l3g->tx_buf[1] = _val;
  spi_submit(l3g->spi_p, &(l3g->spi_trans));
}

// Configuration function called once before normal use
static void l3gd20_spi_send_config(struct L3gd20_Spi *l3g)
{
  uint8_t reg_val = 0;

  switch (l3g->init_status) {
    case L3G_CONF_WHO_AM_I:
      /* query device id */
      l3g->spi_trans.output_length = 1;
      l3g->spi_trans.input_length = 2;
      /* set read bit then reg address */
      l3g->tx_buf[0] = (1 << 7 | L3GD20_REG_WHO_AM_I);
      if (spi_submit(l3g->spi_p, &(l3g->spi_trans))) {
        l3g->init_status++;
      }
      break;
    case L3G_CONF_REG4:
      /* set SPI mode, Filtered Data Selection */
      reg_val = (l3g->config.spi_3_wire << 0) | (l3g->config.full_scale << 4);
      l3gd20_spi_write_to_reg(l3g, L3GD20_REG_CTRL_REG4, reg_val);
      l3g->init_status++;
      break;
    case L3G_CONF_ENABLE:
      /* set data rate, range, enable measurement, is in standby after power up */
      reg_val = (l3g->config.drbw << 4) |
                L3GD20_PD | // Power Down Control to active mode
                L3GD20_Xen | L3GD20_Yen | L3GD20_Zen; // enable z,y,x axes
      l3gd20_spi_write_to_reg(l3g, L3GD20_REG_CTRL_REG1, reg_val);
      l3g->init_status++;
      break;
    case L3G_CONF_DONE:
      l3g->initialized = true;
      l3g->spi_trans.status = SPITransDone;
      break;
    default:
      break;
  }
}

void l3gd20_spi_start_configure(struct L3gd20_Spi *l3g)
{
  if (l3g->init_status == L3G_CONF_UNINIT) {
    l3g->init_status++;
    if (l3g->spi_trans.status == SPITransSuccess || l3g->spi_trans.status == SPITransDone) {
      l3gd20_spi_send_config(l3g);
    }
  }
}

void l3gd20_spi_read(struct L3gd20_Spi *l3g)
{
  if (l3g->initialized && l3g->spi_trans.status == SPITransDone) {
    l3g->spi_trans.output_length = 1;
    l3g->spi_trans.input_length = 8;
    /* set read bit and multiple byte bit, then address */
    l3g->tx_buf[0] = (1 << 7 | 1 << 6 | L3GD20_REG_STATUS_REG);
    spi_submit(l3g->spi_p, &(l3g->spi_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void l3gd20_spi_event(struct L3gd20_Spi *l3g)
{
  if (l3g->initialized) {
    if (l3g->spi_trans.status == SPITransFailed) {
      l3g->spi_trans.status = SPITransDone;
    } else if (l3g->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      if (bit_is_set(l3g->rx_buf[1], 3)) {
        // new xyz data available
        l3g->data_rates.rates.p = Int16FromBuf(l3g->rx_buf, 2);
        l3g->data_rates.rates.q = Int16FromBuf(l3g->rx_buf, 4);
        l3g->data_rates.rates.r = Int16FromBuf(l3g->rx_buf, 6);
        l3g->data_available = true;
      }
      l3g->spi_trans.status = SPITransDone;
    }
  } else if (l3g->init_status != L3G_CONF_UNINIT) { // Configuring but not yet initialized
    switch (l3g->spi_trans.status) {
      case SPITransFailed:
        l3g->init_status--; // Retry config (TODO max retry)
        /* FALLTHROUGH */
      case SPITransSuccess:
        if (l3g->init_status == L3G_CONF_WHO_AM_I_OK) {
          if (l3g->rx_buf[1] == L3GD20_WHO_AM_I) {
            l3g->init_status++;
          } else {
            l3g->init_status = L3G_CONF_WHO_AM_I;
          }
        }
        /* FALLTHROUGH */
      case SPITransDone:
        l3g->spi_trans.status = SPITransDone;
        l3gd20_spi_send_config(l3g);
        break;
      default:
        break;
    }
  }
}

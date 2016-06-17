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
 * @file peripherals/adxl345_spi.c
 *
 * Driver for the accelerometer ADXL345 from Analog Devices using SPI.
 *
 * @todo Use SPICallback to copy data from rx_buf when transaction is successful,
 * instead of checking transaction status for SPITransSuccess in event loop.
 * Problem is that in the callback we don't have a reference to the Adxl345_Spi struct.
 */

#include "peripherals/adxl345_spi.h"


void adxl345_spi_init(struct Adxl345_Spi *adxl, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  adxl->spi_p = spi_p;

  /* configure spi transaction */
  adxl->spi_trans.cpol = SPICpolIdleHigh;
  adxl->spi_trans.cpha = SPICphaEdge2;
  adxl->spi_trans.dss = SPIDss8bit;
  adxl->spi_trans.bitorder = SPIMSBFirst;
  adxl->spi_trans.cdiv = SPIDiv64;

  adxl->spi_trans.select = SPISelectUnselect;
  adxl->spi_trans.slave_idx = slave_idx;
  adxl->spi_trans.output_length = 7;
  adxl->spi_trans.input_length = 7;
  // callback currently unused
  adxl->spi_trans.before_cb = NULL;
  adxl->spi_trans.after_cb = NULL;
  adxl->spi_trans.input_buf = &(adxl->rx_buf[0]);
  adxl->spi_trans.output_buf = &(adxl->tx_buf[0]);

  /* set inital status: Success or Done */
  adxl->spi_trans.status = SPITransDone;

  /* set default ADXL345 config options */
  adxl345_set_default_config(&(adxl->config));

  adxl->initialized = false;
  adxl->data_available = false;
  adxl->init_status = ADXL_CONF_UNINIT;
}


static void adxl345_spi_write_to_reg(struct Adxl345_Spi *adxl, uint8_t _reg, uint8_t _val)
{
  adxl->spi_trans.output_length = 2;
  adxl->spi_trans.input_length = 0;
  adxl->tx_buf[0] = _reg;
  adxl->tx_buf[1] = _val;
  spi_submit(adxl->spi_p, &(adxl->spi_trans));
}

// Configuration function called once before normal use
static void adxl345_spi_send_config(struct Adxl345_Spi *adxl)
{
  switch (adxl->init_status) {
    case ADXL_CONF_RATE:
      adxl345_spi_write_to_reg(adxl, ADXL345_REG_BW_RATE, adxl->config.rate);
      adxl->init_status++;
      break;
    case ADXL_CONF_INT:
      adxl345_spi_write_to_reg(adxl, ADXL345_REG_INT_ENABLE, (adxl->config.drdy_int_enable << 7));
      adxl->init_status++;
      break;
    case ADXL_CONF_FORMAT:
      adxl345_spi_write_to_reg(adxl, ADXL345_REG_DATA_FORMAT, adxl345_data_format(&adxl->config));
      adxl->init_status++;
      break;
    case ADXL_CONF_ENABLE:
      /* enable measurement, is in standby after power up */
      adxl345_spi_write_to_reg(adxl, ADXL345_REG_POWER_CTL, (0x1 << 3));
      adxl->init_status++;
      break;
    case ADXL_CONF_DONE:
      adxl->initialized = true;
      adxl->spi_trans.status = SPITransDone;
      break;
    default:
      break;
  }
}

void adxl345_spi_start_configure(struct Adxl345_Spi *adxl)
{
  if (adxl->init_status == ADXL_CONF_UNINIT) {
    adxl->init_status++;
    if (adxl->spi_trans.status == SPITransSuccess || adxl->spi_trans.status == SPITransDone) {
      adxl345_spi_send_config(adxl);
    }
  }
}

void adxl345_spi_read(struct Adxl345_Spi *adxl)
{
  if (adxl->initialized && adxl->spi_trans.status == SPITransDone) {
    adxl->spi_trans.output_length = 1;
    adxl->spi_trans.input_length = 7;
    /* set read bit and multiple byte bit, then address */
    adxl->tx_buf[0] = (1 << 7 | 1 << 6 | ADXL345_REG_DATA_X0);
    spi_submit(adxl->spi_p, &(adxl->spi_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void adxl345_spi_event(struct Adxl345_Spi *adxl)
{
  if (adxl->initialized) {
    if (adxl->spi_trans.status == SPITransFailed) {
      adxl->spi_trans.status = SPITransDone;
    } else if (adxl->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      adxl->data.vect.x = Int16FromBuf(adxl->rx_buf, 1);
      adxl->data.vect.y = Int16FromBuf(adxl->rx_buf, 3);
      adxl->data.vect.z = Int16FromBuf(adxl->rx_buf, 5);
      adxl->data_available = true;
      adxl->spi_trans.status = SPITransDone;
    }
  } else if (adxl->init_status != ADXL_CONF_UNINIT) { // Configuring but not yet initialized
    switch (adxl->spi_trans.status) {
      case SPITransFailed:
        adxl->init_status--; // Retry config (TODO max retry)
      case SPITransSuccess:
      case SPITransDone:
        adxl->spi_trans.status = SPITransDone;
        adxl345_spi_send_config(adxl);
        break;
      default:
        break;
    }
  }
}

/*
 * Copyright (C) 2014 Gautier Hattenberger, Alexandre Bustico
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file peripherals/ads1220.c
 *
 * Driver for the ADS1220
 * 24-bits ADC from TI
 * SPI communication
 *
 */

#include "peripherals/ads1220.h"

// Commands
#define ADS1220_WREG(_reg, _nb) ((1<<6)|(_reg<<2)|(_nb-1))
#define ADS1220_RREG(_reg, _nb) ((1<<5)|(_reg<<2)|(_nb-1))
#define ADS1220_RESET 0x06
#define ADS1220_START_SYNC 0x08
#define ADS1220_POWERDOWN 0x02
#define ADS1220_RDATA 0x10

// Conf registers
#define ADS1220_CONF0 0x0
#define ADS1220_CONF1 0x1
#define ADS1220_CONF2 0x2
#define ADS1220_CONF3 0x3


// Init function
void ads1220_init(struct Ads1220 *ads, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  ads->spi_p = spi_p;

  /* configure spi transaction */
  ads->spi_trans.cpol = SPICpolIdleLow;
  ads->spi_trans.cpha = SPICphaEdge2;
  ads->spi_trans.dss = SPIDss8bit;
  ads->spi_trans.bitorder = SPIMSBFirst;
  ads->spi_trans.cdiv = SPIDiv128; // f_PCLK / div

  ads->spi_trans.select = SPISelectUnselect;
  ads->spi_trans.slave_idx = slave_idx;
  ads->spi_trans.output_length = 0;
  ads->spi_trans.input_length = 0;
  ads->spi_trans.before_cb = NULL;
  ads->spi_trans.after_cb = NULL;
  ads->spi_trans.input_buf = &(ads->rx_buf[0]);
  ads->spi_trans.output_buf = &(ads->tx_buf[0]);

  /* set inital status: Success or Done */
  ads->spi_trans.status = SPITransDone;

  ads->data = 0;
  ads->data_available = FALSE;
  ads->config.status = ADS1220_UNINIT;
}


// Configuration function called once before normal use
static void ads1220_send_config(struct Ads1220 *ads)
{
  ads->spi_trans.output_length = 5;
  ads->spi_trans.input_length = 0;
  ads->tx_buf[0] = ADS1220_WREG(ADS1220_CONF0, 4);
  ads->tx_buf[1] = (
                     (ads->config.pga_bypass << 0) |
                     (ads->config.gain << 1) |
                     (ads->config.mux << 4));
  ads->tx_buf[2] = (
                     (ads->config.conv << 2) |
                     (ads->config.rate << 5));
  ads->tx_buf[3] = (
                     (ads->config.idac << 0) |
                     (ads->config.vref << 6));
  ads->tx_buf[4] = (
                     (ads->config.i2mux << 2) |
                     (ads->config.i1mux << 5));
  spi_submit(ads->spi_p, &(ads->spi_trans));
}

// Configuration function called before normal use
void ads1220_configure(struct Ads1220 *ads)
{
  if (ads->config.status == ADS1220_UNINIT) {
    if (ads->spi_trans.status == SPITransSuccess || ads->spi_trans.status == SPITransDone) {
      ads->spi_trans.output_length = 1;
      ads->spi_trans.input_length = 0;
      ads->tx_buf[0] = ADS1220_RESET;
      spi_submit(ads->spi_p, &(ads->spi_trans));
      ads->config.status = ADS1220_SEND_RESET;
    }
  } else if (ads->config.status == ADS1220_INITIALIZING) { // Configuring but not yet initialized
    if (ads->spi_trans.status == SPITransSuccess || ads->spi_trans.status == SPITransDone) {
      ads1220_send_config(ads); // do config
    }
  }
}

// Read next data
void ads1220_read(struct Ads1220 *ads)
{
  if (ads->config.status == ADS1220_INITIALIZED && ads->spi_trans.status == SPITransDone) {
    ads->spi_trans.output_length = 0;
    ads->spi_trans.input_length = 3;
    spi_submit(ads->spi_p, &(ads->spi_trans));
  }
}

// Check end of transaction
void ads1220_event(struct Ads1220 *ads)
{
  if (ads->config.status == ADS1220_INITIALIZED) {
    if (ads->spi_trans.status == SPITransFailed) {
      ads->spi_trans.status = SPITransDone;
    } else if (ads->spi_trans.status == SPITransSuccess) {
      // Successfull reading of 24bits adc
      ads->data = (uint32_t)(((uint32_t)(ads->rx_buf[0]) << 16) | ((uint32_t)(ads->rx_buf[1]) << 8) | (ads->rx_buf[2]));
      ads->data_available = TRUE;
      ads->spi_trans.status = SPITransDone;
    }
  } else if (ads->config.status == ADS1220_SEND_RESET) { // Reset ads1220 before configuring
    if (ads->spi_trans.status == SPITransFailed) {
      ads->spi_trans.status = SPITransDone;
      ads->config.status = ADS1220_UNINIT; // config failed
    } else if (ads->spi_trans.status == SPITransSuccess) {
      ads->spi_trans.status = SPITransDone;
      ads->config.status = ADS1220_INITIALIZING;
      // do config at next call of ads1220_configure() (or ads1220_periodic())
    }
  } else if (ads->config.status == ADS1220_INITIALIZING) { // Configuring but not yet initialized
    if (ads->spi_trans.status == SPITransFailed) {
      ads->spi_trans.status = SPITransDone;
      ads->config.status = ADS1220_UNINIT; // config failed
    } else if (ads->spi_trans.status == SPITransSuccess) {
      ads->spi_trans.status = SPITransDone;
      ads->config.status = ADS1220_INITIALIZED; // config done
    }
  }
}


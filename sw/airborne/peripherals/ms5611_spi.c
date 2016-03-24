/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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
 * @file peripherals/ms5611_spi.c
 * Measurement Specialties (Intersema) MS5611-01BA and MS5607-02BA03 pressure/temperature sensor interface for SPI.
 *
 */


#include "peripherals/ms5611_spi.h"


void ms5611_spi_init(struct Ms5611_Spi *ms, struct spi_periph *spi_p, uint8_t slave_idx,
                     bool is_ms5607)
{
  /* set spi_peripheral */
  ms->spi_p = spi_p;

  /* configure spi transaction */
  ms->spi_trans.cpol = SPICpolIdleHigh;
  ms->spi_trans.cpha = SPICphaEdge2;
  ms->spi_trans.dss = SPIDss8bit;
  ms->spi_trans.bitorder = SPIMSBFirst;
  ms->spi_trans.cdiv = SPIDiv64;

  ms->spi_trans.select = SPISelectUnselect;
  ms->spi_trans.slave_idx = slave_idx;
  ms->spi_trans.output_length = 1;
  ms->spi_trans.input_length = 4;
  ms->spi_trans.before_cb = NULL;
  ms->spi_trans.after_cb = NULL;
  ms->spi_trans.input_buf = ms->rx_buf;
  ms->spi_trans.output_buf = ms->tx_buf;

  /* set initial status: Success or Done */
  ms->spi_trans.status = SPITransDone;

  ms->data_available = false;
  ms->initialized = false;
  ms->status = MS5611_STATUS_UNINIT;
  ms->prom_cnt = 0;
  ms->is_ms5607 = is_ms5607;
}

void ms5611_spi_start_configure(struct Ms5611_Spi *ms)
{
  if (ms->status == MS5611_STATUS_UNINIT) {
    ms->initialized = false;
    ms->prom_cnt = 0;
    ms->tx_buf[0] = MS5611_SOFT_RESET;
    spi_submit(ms->spi_p, &(ms->spi_trans));
    ms->status = MS5611_STATUS_RESET;
  }
}

void ms5611_spi_start_conversion(struct Ms5611_Spi *ms)
{
  if (ms->status == MS5611_STATUS_IDLE &&
      ms->spi_trans.status == SPITransDone) {
    /* start D1 conversion */
    ms->tx_buf[0] = MS5611_START_CONV_D1;
    spi_submit(ms->spi_p, &(ms->spi_trans));
    ms->status = MS5611_STATUS_CONV_D1;
  }
}

/**
 * Periodic function to ensure proper delay after triggering reset or conversion.
 * Should run at 100Hz max.
 * Typical conversion time is 8.22ms at max resolution.
 */
void ms5611_spi_periodic_check(struct Ms5611_Spi *ms)
{
  switch (ms->status) {
    case MS5611_STATUS_RESET:
      ms->status = MS5611_STATUS_RESET_OK;
      break;
    case MS5611_STATUS_RESET_OK:
      if (ms->spi_trans.status == SPITransDone) {
        /* start getting prom data */
        ms->tx_buf[0] = MS5611_PROM_READ | (ms->prom_cnt << 1);
        spi_submit(ms->spi_p, &(ms->spi_trans));
        ms->status = MS5611_STATUS_PROM;
      }
      break;
    case MS5611_STATUS_CONV_D1:
      ms->status = MS5611_STATUS_CONV_D1_OK;
      break;
    case MS5611_STATUS_CONV_D1_OK:
      if (ms->spi_trans.status == SPITransDone) {
        /* read D1 adc */
        ms->tx_buf[0] = MS5611_ADC_READ;
        spi_submit(ms->spi_p, &(ms->spi_trans));
        ms->status = MS5611_STATUS_ADC_D1;
      }
      break;
    case MS5611_STATUS_CONV_D2:
      ms->status = MS5611_STATUS_CONV_D2_OK;
      break;
    case MS5611_STATUS_CONV_D2_OK:
      if (ms->spi_trans.status == SPITransDone) {
        /* read D2 adc */
        ms->tx_buf[0] = MS5611_ADC_READ;
        spi_submit(ms->spi_p, &(ms->spi_trans));
        ms->status = MS5611_STATUS_ADC_D2;
      }
      break;
    default:
      break;
  }
}

void ms5611_spi_event(struct Ms5611_Spi *ms)
{
  if (ms->initialized) {
    if (ms->spi_trans.status == SPITransFailed) {
      ms->status = MS5611_STATUS_IDLE;
      ms->spi_trans.status = SPITransDone;
    } else if (ms->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      switch (ms->status) {

        case MS5611_STATUS_ADC_D1:
          /* read D1 (pressure) */
          ms->data.d1 = (ms->rx_buf[1] << 16) |
                        (ms->rx_buf[2] << 8) |
                        ms->rx_buf[3];
          if (ms->data.d1 == 0) {
            /* if value is zero, it was read to soon and is invalid, back to idle */
            ms->status = MS5611_STATUS_IDLE;
          } else {
            /* start D2 conversion */
            ms->tx_buf[0] = MS5611_START_CONV_D2;
            spi_submit(ms->spi_p, &(ms->spi_trans));
            ms->status = MS5611_STATUS_CONV_D2;
          }
          break;

        case MS5611_STATUS_ADC_D2:
          /* read D2 (temperature) */
          ms->data.d2 = (ms->rx_buf[1] << 16) |
                        (ms->rx_buf[2] << 8) |
                        ms->rx_buf[3];
          if (ms->data.d2 == 0) {
            /* if value is zero, it was read to soon and is invalid, back to idle */
            ms->status = MS5611_STATUS_IDLE;
          } else {
            /* calculate temp and pressure from measurements and set available if valid */
            if (ms->is_ms5607) {
              ms->data_available = ms5607_calc(&(ms->data));
            }
            else {
              ms->data_available = ms5611_calc(&(ms->data));
            }
            ms->status = MS5611_STATUS_IDLE;
          }
          break;

        default:
          break;
      }
      ms->spi_trans.status = SPITransDone;
    }
  } else if (ms->status != MS5611_STATUS_UNINIT) { // Configuring but not yet initialized
    switch (ms->spi_trans.status) {

      case SPITransFailed:
        /* try again */
        ms->status = MS5611_STATUS_UNINIT;
        ms->spi_trans.status = SPITransDone;
        break;

      case SPITransSuccess:
        if (ms->status == MS5611_STATUS_PROM) {
          /* read prom data */
          ms->data.c[ms->prom_cnt++] = (ms->rx_buf[1] << 8) |
                                       ms->rx_buf[2];
          if (ms->prom_cnt < PROM_NB) {
            /* get next prom data */
            ms->tx_buf[0] = MS5611_PROM_READ | (ms->prom_cnt << 1);
            spi_submit(ms->spi_p, &(ms->spi_trans));
          } else {
            /* done reading prom, check prom crc */
            if (ms5611_prom_crc_ok(ms->data.c)) {
              ms->initialized = true;
              ms->status = MS5611_STATUS_IDLE;
            } else {
              /* checksum error, try again */
              ms->status = MS5611_STATUS_UNINIT;
            }
          }
        }
        ms->spi_trans.status = SPITransDone;
        break;

      default:
        break;
    }
  }
}

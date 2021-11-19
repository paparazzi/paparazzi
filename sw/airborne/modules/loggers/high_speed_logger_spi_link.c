/*
 * Copyright (C) 2005-2013 The Paparazzi Team
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

#include "high_speed_logger_spi_link.h"

#include "modules/imu/imu.h"
#include "mcu_periph/spi.h"


struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct spi_transaction high_speed_logger_spi_link_transaction;

static volatile bool high_speed_logger_spi_link_ready = true;

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans);

void high_speed_logger_spi_link_init(void)
{
  high_speed_logger_spi_link_data.id = 0;

  high_speed_logger_spi_link_transaction.select        = SPISelectUnselect;
  high_speed_logger_spi_link_transaction.cpol          = SPICpolIdleHigh;
  high_speed_logger_spi_link_transaction.cpha          = SPICphaEdge2;
  high_speed_logger_spi_link_transaction.dss           = SPIDss8bit;
  high_speed_logger_spi_link_transaction.bitorder      = SPIMSBFirst;
  high_speed_logger_spi_link_transaction.cdiv          = SPIDiv64;
  high_speed_logger_spi_link_transaction.slave_idx     = HIGH_SPEED_LOGGER_SPI_LINK_SLAVE_NUMBER;
  high_speed_logger_spi_link_transaction.output_length = sizeof(high_speed_logger_spi_link_data);
  high_speed_logger_spi_link_transaction.output_buf    = (uint8_t *) &high_speed_logger_spi_link_data;
  high_speed_logger_spi_link_transaction.input_length  = 0;
  high_speed_logger_spi_link_transaction.input_buf     = NULL;
  high_speed_logger_spi_link_transaction.after_cb      = high_speed_logger_spi_link_trans_cb;
}


void high_speed_logger_spi_link_periodic(void)
{
  // Static counter to identify missing samples
  static int32_t counter = 0;

  // count all periodic steps
  counter ++;

  // only send a new message if the previous was completely sent
  if (high_speed_logger_spi_link_ready) {
    // copy the counter into the SPI datablock
    high_speed_logger_spi_link_data.id = counter;

    high_speed_logger_spi_link_ready = false;
    high_speed_logger_spi_link_data.gyro_p     = imu.gyro_unscaled.p;
    high_speed_logger_spi_link_data.gyro_q     = imu.gyro_unscaled.q;
    high_speed_logger_spi_link_data.gyro_r     = imu.gyro_unscaled.r;
    high_speed_logger_spi_link_data.acc_x      = imu.accel_unscaled.x;
    high_speed_logger_spi_link_data.acc_y      = imu.accel_unscaled.y;
    high_speed_logger_spi_link_data.acc_z      = imu.accel_unscaled.z;
    high_speed_logger_spi_link_data.mag_x      = imu.mag_unscaled.x;
    high_speed_logger_spi_link_data.mag_y      = imu.mag_unscaled.y;
    high_speed_logger_spi_link_data.mag_z      = imu.mag_unscaled.z;

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  }
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = true;
}



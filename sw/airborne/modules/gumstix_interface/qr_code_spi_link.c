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

#include "qr_code_spi_link.h"

#include "subsystems/imu.h"
#include "mcu_periph/spi.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

//struct qr_code_spi_link_data qr_code_spi_link_data;
struct spi_transaction qr_code_spi_link_transaction;

static volatile bool_t qr_code_spi_data_available = FALSE;

uint8_t testDataOut[3] = {1, 2, 3};
uint8_t testDataIn[3] = {9, 9, 9};

static void qr_code_spi_link_trans_cb(struct spi_transaction *trans);

void qr_code_spi_link_init(void)
{
  qr_code_spi_link_transaction.cpol          = SPICpolIdleHigh;
  qr_code_spi_link_transaction.cpha          = SPICphaEdge2;
  qr_code_spi_link_transaction.dss           = SPIDss8bit;
  qr_code_spi_link_transaction.bitorder      = SPIMSBFirst;
  qr_code_spi_link_transaction.output_length = 3;
  qr_code_spi_link_transaction.output_buf    = testDataOut;
  qr_code_spi_link_transaction.input_length  = 3;
  qr_code_spi_link_transaction.input_buf     = testDataIn;
  qr_code_spi_link_transaction.after_cb      = qr_code_spi_link_trans_cb;
  //spi_slave_set_config(&spi1, &qr_code_spi_link_transaction);
  spi_slave_register(&spi1, &qr_code_spi_link_transaction);
}


void qr_code_spi_link_periodic(void)
{
  if (qr_code_spi_data_available) {
    qr_code_spi_data_available = FALSE;
    uint16_t x, y;
    memcpy(&x, qr_code_spi_link_transaction.input_buf, 2);
    memcpy(&y, qr_code_spi_link_transaction.input_buf + 2, 2);
    DOWNLINK_SEND_VISUALTARGET(DefaultChannel, DefaultDevice, &x, &y);
    spi_slave_register(&spi1, &qr_code_spi_link_transaction);
  }
}

static void qr_code_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  qr_code_spi_data_available = TRUE;
}



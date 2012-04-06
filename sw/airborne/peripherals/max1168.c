/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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

#include "peripherals/max1168.h"
#include "mcu_periph/spi.h"

volatile uint8_t max1168_status;
uint16_t max1168_values[MAX1168_NB_CHAN];

struct spi_transaction max1168_req_trans;
struct spi_transaction max1168_read_trans;

extern void max1168_init( void ) {

  max1168_arch_init();

  uint8_t i;
  for (i=0; i<MAX1168_NB_CHAN; i++)
    max1168_values[i] = 0;

  // init spi transaction parameters
  max1168_req_trans.slave_idx = MAX1168_SLAVE_IDX;
  max1168_req_trans.cpol = SPICpolIdleLow;
  max1168_req_trans.cpha = SPICphaEdge1;
  max1168_req_trans.dss = SPIDss16bit;
  max1168_req_trans.select = SPISelect;
  max1168_req_trans.length = 1;
  max1168_req_trans.output_buf[0] = MAX1168_CONF_CR << 8;

  max1168_read_trans.slave_idx = MAX1168_SLAVE_IDX;
  max1168_read_trans.cpol = SPICpolIdleLow;
  max1168_read_trans.cpha = SPICphaEdge1;
  max1168_read_trans.dss = SPIDss16bit;
  max1168_read_trans.select = SPIUnselect;
  // read 8 frames
  // FIXME should be function of control register options
  max1168_read_trans.length = 8;

  max1168_status = MAX1168_IDLE;
}

#include "led.h"
void max1168_read( void ) {
  ASSERT((max1168_status == MAX1168_IDLE), DEBUG_MAX_1168, MAX1168_ERR_READ_OVERUN);

  /* set SPI transaction */
  /* SPI is locked between the two transactions */
  /* SPI is unlocked when EOC is received */

  spi_submit(&(MAX1168_SPI_DEV),&max1168_req_trans);
  spi_lock(&(MAX1168_SPI_DEV),MAX1168_SLAVE_IDX);
  spi_submit(&(MAX1168_SPI_DEV),&max1168_read_trans);

  max1168_status = MAX1168_SENDING_REQ;

}

void max1168_event( void ) {
  //LED_OFF(1);
  //LED_OFF(2);
  //LED_OFF(3);
  //LED_ON(4);
  // handle request transaction
  if (max1168_req_trans.status == SPITransSuccess) {
    //LED_ON(3);
    if (max1168_status == MAX1168_GOT_EOC) {
      //LED_ON(1);
      // eoc occurs, unlock SPI
      spi_resume(&(MAX1168_SPI_DEV),MAX1168_SLAVE_IDX);

      max1168_status = MAX1168_READING_RES;
      max1168_req_trans.status = SPITransDone;
    }
    else { /* TODO ? */ }
  }
  else if (max1168_req_trans.status == SPITransFailed) {
      //LED_ON(2);
    // TODO
    max1168_status = MAX1168_IDLE;
    spi_slave_unselect(MAX1168_SLAVE_IDX);
    max1168_req_trans.status = SPITransDone;
  }

  // handle reading transaction
  if (max1168_read_trans.status == SPITransSuccess) {
    if (max1168_status == MAX1168_READING_RES) {
      // store values
      max1168_values[0] = max1168_read_trans.input_buf[0];
      max1168_values[1] = max1168_read_trans.input_buf[1];
      max1168_values[2] = max1168_read_trans.input_buf[2];
      max1168_values[3] = max1168_read_trans.input_buf[3];
      max1168_values[4] = max1168_read_trans.input_buf[4];
      max1168_values[5] = max1168_read_trans.input_buf[5];
      max1168_values[6] = max1168_read_trans.input_buf[6];
      max1168_values[7] = max1168_read_trans.input_buf[7];
      max1168_status = MAX1168_DATA_AVAILABLE;
      max1168_read_trans.status = SPITransDone;
    }
    else { /* TODO ? */ }
  }
  else if (max1168_read_trans.status == SPITransFailed) {
    // TODO
    max1168_status = MAX1168_IDLE;
    spi_slave_unselect(MAX1168_SLAVE_IDX);
    max1168_read_trans.status = SPITransDone;
  }
  //LED_OFF(4);
}

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
 */

/**
 * @file peripherals/ms2100.c
 * Driver for the ms2100 magnetic sensor from PNI
 */

#include "peripherals/ms2100.h"
#include "mcu_periph/spi.h"

#include <stdlib.h>  // for abs

volatile uint8_t ms2100_status;
volatile int16_t ms2100_values[MS2100_NB_AXIS];
volatile uint8_t ms2100_cur_axe;

struct spi_transaction ms2100_trans;

uint8_t ms2100_control_byte;
uint8_t ms2100_val[2];


void ms2100_init( void ) {

  ms2100_arch_init();

  uint8_t i;
  for (i=0; i<MS2100_NB_AXIS; i++)
    ms2100_values[i] = 0;
  ms2100_cur_axe = 0;

  // init spi transaction parameters
  ms2100_trans.slave_idx = MS2100_SLAVE_IDX;
  ms2100_trans.select = SPISelectUnselect;
  ms2100_trans.cpol = SPICpolIdleLow;
  ms2100_trans.cpha = SPICphaEdge1;
  ms2100_trans.dss = SPIDss8bit;
  ms2100_trans.status = SPITransDone;

  ms2100_status = MS2100_IDLE;
}

void ms2100_read( void ) {

  /* set SPI transaction */
  ms2100_control_byte = (ms2100_cur_axe+1) << 0 | MS2100_DIVISOR << 4;
  ms2100_trans.output_buf = &ms2100_control_byte;
  ms2100_trans.output_length = 1;
  ms2100_trans.input_buf = 0;
  ms2100_trans.input_length = 0;
  ms2100_trans.before_cb = ms2100_reset_cb; // implemented in ms2100_arch.c

  spi_submit(&(MS2100_SPI_DEV),&ms2100_trans);

  ms2100_status = MS2100_SENDING_REQ;
}

void ms2100_event( void ) {
  if (ms2100_trans.status == SPITransSuccess) {
    if (ms2100_status == MS2100_GOT_EOC) {
      // eoc occurs, submit reading req
      // read 2 bytes
      ms2100_trans.output_buf = 0;
      ms2100_trans.output_length = 0;
      ms2100_trans.input_buf = ms2100_val;
      ms2100_trans.input_length = 2;
      ms2100_trans.before_cb = 0; // no reset when reading values
      spi_submit(&(MS2100_SPI_DEV),&ms2100_trans);

      ms2100_status = MS2100_READING_RES;
      ms2100_trans.status = SPITransDone;
    }
    else if (ms2100_status == MS2100_READING_RES) {
      // store value
      int16_t new_val;
      new_val = ms2100_trans.input_buf[0] << 8;
      new_val += ms2100_trans.input_buf[1];
      if (abs(new_val) < 2000)
        ms2100_values[ms2100_cur_axe] = new_val;
      ms2100_cur_axe++;
      if (ms2100_cur_axe > 2) {
        ms2100_cur_axe = 0;
        ms2100_status = MS2100_DATA_AVAILABLE;
      }
      else {
        ms2100_status = MS2100_IDLE;
      }
      ms2100_trans.status = SPITransDone;
    }
    else { /* TODO ? */ }
  }
  else if (ms2100_trans.status == SPITransFailed) {
    // TODO is it enough ?
    ms2100_status = MS2100_IDLE;
    ms2100_trans.status = SPITransDone;
  }
}


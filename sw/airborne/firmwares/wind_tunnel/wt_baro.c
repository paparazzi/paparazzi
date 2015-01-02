/*
 * Copyright (C) 2007  ENAC
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

/** \file
 *  \brief
 *
 */

#include "wt_baro.h"
#include "spi.h"

uint32_t wt_baro_pressure;
bool_t wt_baro_available;

static bool_t status_read_data;

#define CMD_INIT_1      0x24 // set chanel AIN1/AIN2 and next operation on filter high
#define CMD_INIT_2      0xCF // set unipolar mode, 24 bits, no boost, filter high
#define CMD_INIT_3      0x34 // set chanel AIN1/AIN2 and next operation on filter low
#define CMD_INIT_4      0x00 // set low filter
#define CMD_INIT_5      0x14 // set chanel AIN1/AIN2 and next operation on mode register
#define CMD_INIT_6      0x20 // set gain to 1, burnout current off, no filter sync, self calibration
#define CMD_MEASUREMENT 0x54 // set chanel AIN1/AIN2 and next operation on data register


uint8_t buf_input[3];
uint8_t buf_output[3];

#define Uint24(buf_input) (((uint32_t)buf_input[0]) << 16 |((uint16_t)buf_input[1]) << 8 | buf_input[2])


static void send1_on_spi(uint8_t d)
{
  buf_output[0] = d;
  spi_buffer_length = 1;

  spi_buffer_input = (uint8_t *)&buf_input;
  spi_buffer_output = (uint8_t *)&buf_output;
  SpiStart();
}


void wt_baro_init(void)
{

  wt_baro_pressure = 0;

  send1_on_spi(CMD_INIT_1);
  send1_on_spi(CMD_INIT_2);
  send1_on_spi(CMD_INIT_3);
  send1_on_spi(CMD_INIT_4);
  send1_on_spi(CMD_INIT_5);
  send1_on_spi(CMD_INIT_6);

  status_read_data = FALSE;
  wt_baro_available = FALSE;

}

void wt_baro_periodic(void)
{
  if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  if (status_read_data) {
    buf_output[0] = buf_output[1] = buf_output[2] = 0;
    spi_buffer_length = 3;
  } else {
    buf_output[0] = CMD_MEASUREMENT;
    spi_buffer_length = 1;
  }

  spi_buffer_input = (uint8_t *)&buf_input;
  spi_buffer_output = (uint8_t *)&buf_output;
  //if (status_read_data)
  //  SpiSetCPHA();
  //else
  //  SpiClrCPHA();
  SpiStart();
}

static uint32_t data;

/* Handle the SPI message, i.e. store the received values in variables */
void wt_baro_event(void)
{

  if (status_read_data) {
    data = Uint24(buf_input);
    /* Compute pressure */
    wt_baro_pressure = data;
    wt_baro_available = TRUE;
  } /* else nothing to read */

  status_read_data = !status_read_data;

  //if (!status_read_data) {
  // /* Ask next conversion now */
  //  baro_MS5534A_send();
  //}
}


/*
 * $Id$
 *  
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz
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


#include <stdbool.h>
#include "MPPT.h"
#include "main_fbw.h"
#include "i2c.h"


#define MPPT_SLAVE_ADDR 0x40
#define NB_I2C_DATA 8
#define MPPT_MODE_ADDR 0xf

/**
 0: VBat (mV)
 1: IBat (mA)
 2: PBat (mW)
 3: VSol (mV)
 4: ISol (mA)
 5: PSol (mW)
 6: IConv (mA)
 7: PConv (mW)
 
 9: IBat + IConv
*/



#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "uart.h"
#include "messages.h"
#include "downlink.h"

static volatile bool_t MPPT_i2c_done;

uint8_t MPPT_mode;
/** A value different from 0 is a request from this mode */

static uint8_t MPPT_status;
#define MPPT_STATUS_IDLE 0
#define MPPT_STATUS_WRITING 1
#define MPPT_STATUS_ASKING 2
#define MPPT_STATUS_READING 3


static uint8_t data_index = 0xff;
static int16_t MPPT_data[NB_DATA];

void MPPT_init( void ) {
  MPPT_mode = 0;
  MPPT_status = MPPT_STATUS_IDLE;
  MPPT_i2c_done = TRUE;
}



static void MPPT_ask( void ) {
  data_index++;
  if (data_index >= NB_I2C_DATA) {
    /* Setting the current value */
    fbw_current_milliamp = MPPT_data[MPPT_IBAT_INDEX];

    MPPT_data[MPPT_ITOTAL_INDEX] = MPPT_data[MPPT_IBAT_INDEX] + MPPT_data[MPPT_ICONV_INDEX];
    DOWNLINK_SEND_MPPT(DefaultChannel, NB_DATA, MPPT_data);
    data_index = 0;
  }
  
  i2c0_buf[0] = data_index;
  i2c0_transmit(MPPT_SLAVE_ADDR, 1, &MPPT_i2c_done);
  MPPT_i2c_done = FALSE;
  MPPT_status = MPPT_STATUS_ASKING;
}

void MPPT_periodic( void ) {
  //  MPPT_i2c_done = TRUE;

  if (MPPT_i2c_done) {
    switch (MPPT_status) {
    case MPPT_STATUS_IDLE:
      /* If free, change mode if needed */
      if (MPPT_mode) {
	i2c0_buf[0] = MPPT_MODE_ADDR;
	i2c0_buf[1] = 0;
	i2c0_buf[2] = MPPT_mode;
	i2c0_transmit(MPPT_SLAVE_ADDR, 3, &MPPT_i2c_done);
	MPPT_i2c_done = FALSE;
	MPPT_mode = 0;
	MPPT_status = MPPT_STATUS_WRITING;
      } else {
	MPPT_ask();
      }
      break;

    case MPPT_STATUS_WRITING:
      MPPT_status = MPPT_STATUS_IDLE;
      break;

    case MPPT_STATUS_ASKING:
      /* The slave should send 2 bytes */
      i2c0_receive(MPPT_SLAVE_ADDR, 2, &MPPT_i2c_done);
      MPPT_i2c_done = FALSE;
      MPPT_status = MPPT_STATUS_READING;
      break;
    
    case MPPT_STATUS_READING:
      /* We got 2 bytes */
      if (data_index < NB_I2C_DATA)
	MPPT_data[data_index] = (i2c0_buf[0]<<8) | i2c0_buf[1];
      MPPT_status = MPPT_STATUS_IDLE;
      break;
    }
  }
}



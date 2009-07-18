/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "actuators.h"

#include BOARD_CONFIG

uint8_t twi_blmc_nb_err;

uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];
volatile bool_t  buss_twi_blmc_status;
volatile bool_t  buss_twi_blmc_i2c_done;
volatile uint8_t buss_twi_blmc_idx;

const uint8_t buss_twi_blmc_addr[BUSS_TWI_BLMC_NB] = BUSS_BLMC_ADDR;

void actuators_init ( void ) {
  uint8_t i;
  for (i=0; i<BUSS_TWI_BLMC_NB;i++)
    buss_twi_blmc_motor_power[i] = 0;
  buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_IDLE;
  twi_blmc_nb_err = 0;
  buss_twi_blmc_i2c_done = TRUE;
}

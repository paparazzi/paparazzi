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

#include "i2c.h"
#include "led.h"

bool_t  actuators_asctec_twi_blmc_command;
uint8_t actuators_asctec_twi_blmc_command_type;

#define MB_TWI_CONTROLLER_ASCTECH_ADDR_FRONT 0
#define MB_TWI_CONTROLLER_ASCTECH_ADDR_BACK  1
#define MB_TWI_CONTROLLER_ASCTECH_ADDR_LEFT  2
#define MB_TWI_CONTROLLER_ASCTECH_ADDR_RIGHT 3
uint8_t actuators_asctec_twi_blmc_addr;
uint8_t actuators_asctec_twi_blmc_new_addr;

#define ASCTEC_TWI_BLMC_NB 4

int8_t asctec_twi_blmc_motor_power[ASCTEC_TWI_BLMC_NB];
uint8_t twi_blmc_nb_err;
volatile uint8_t mb_twi_i2c_done;


#define MB_TWI_CONTROLLER_MAX_CMD 200
#define MB_TWI_CONTROLLER_ADDR 0x02

void actuators_init(void) {
  twi_blmc_nb_err = 0;
  mb_twi_i2c_done = TRUE;
  actuators_asctec_twi_blmc_command = MB_TWI_CONTROLLER_COMMAND_NONE;
  actuators_asctec_twi_blmc_addr = MB_TWI_CONTROLLER_ASCTECH_ADDR_FRONT;
}

void asctec_twi_controller_send() {
  if (!mb_twi_i2c_done) {
    twi_blmc_nb_err++;
    
  }
  if (actuators_asctec_twi_blmc_command != MB_TWI_CONTROLLER_COMMAND_NONE) {

    switch (actuators_asctec_twi_blmc_command) {
      
    case MB_TWI_CONTROLLER_COMMAND_TEST :
      i2c0_buf[0] = 251;
      i2c0_buf[1] = actuators_asctec_twi_blmc_addr;
      i2c0_buf[2] = 0;
      i2c0_buf[3] = 231 + actuators_asctec_twi_blmc_addr;
      mb_twi_i2c_done = FALSE;
      i2c0_transmit(MB_TWI_CONTROLLER_ADDR, 4, &mb_twi_i2c_done);
      break;
      
    case MB_TWI_CONTROLLER_COMMAND_REVERSE :
      i2c0_buf[0] = 254;
      i2c0_buf[1] = actuators_asctec_twi_blmc_addr;
      i2c0_buf[2] = 0;
      i2c0_buf[3] = 234 + actuators_asctec_twi_blmc_addr;
      mb_twi_i2c_done = FALSE;
      i2c0_transmit(MB_TWI_CONTROLLER_ADDR, 4, &mb_twi_i2c_done);
      break;
      
    case MB_TWI_CONTROLLER_COMMAND_SET_ADDR :
      i2c0_buf[0] = 250;
      i2c0_buf[1] = actuators_asctec_twi_blmc_addr;
      i2c0_buf[2] = actuators_asctec_twi_blmc_new_addr;
      i2c0_buf[3] = 230 + actuators_asctec_twi_blmc_addr + 
	actuators_asctec_twi_blmc_new_addr;
      actuators_asctec_twi_blmc_addr = actuators_asctec_twi_blmc_new_addr;
      mb_twi_i2c_done = FALSE;
      i2c0_transmit(MB_TWI_CONTROLLER_ADDR, 4, &mb_twi_i2c_done);
      break;
      
    }
    actuators_asctec_twi_blmc_command = MB_TWI_CONTROLLER_COMMAND_NONE;
  }
  else {
    i2c0_buf[0] = 100 + asctec_twi_blmc_motor_power[SERVO_PITCH];
    i2c0_buf[1] = 100 + asctec_twi_blmc_motor_power[SERVO_ROLL];
    i2c0_buf[2] = 100 + asctec_twi_blmc_motor_power[SERVO_YAW];
    i2c0_buf[3] = asctec_twi_blmc_motor_power[SERVO_THRUST];
    mb_twi_i2c_done = FALSE;
    i2c0_transmit(MB_TWI_CONTROLLER_ADDR, 4, &mb_twi_i2c_done);
  }
}

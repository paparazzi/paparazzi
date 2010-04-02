/*
 * Copyright (C) 2010 ENAC
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

#include "infrared_i2c.h"
#include "i2c.h"

#define IR_VERT_I2C_ADDR (0x68 << 1)
#define OC_BIT (1 << 4)

#define IR_I2C_IDLE            0
#define IR_I2C_READ_TOP        1
#define IR_I2C_CONFIGURE_TOP   2

int16_t ir_i2c_top;

// Local variables
volatile bool_t ir_i2c_done;
bool_t ir_i2c_data_available;
static uint8_t ir_i2c_status;
uint8_t ir_i2c_conf_word;

#define NO_CONF_WORD 0xff
#define ValidConfWord(_x) (_x < 0x4)


void infrared_i2c_init( void ) {
  ir_i2c_done = TRUE;
  ir_i2c_data_available = FALSE;
  ir_i2c_status = IR_I2C_IDLE;
  ir_i2c_conf_word = IR_I2C_DEFAULT_CONF;
}

void infrared_i2c_update( void ) {
  if (ir_i2c_done && ir_i2c_status == IR_I2C_IDLE) {
    if (ValidConfWord(ir_i2c_conf_word)) {
      i2c0_buf[0] = ir_i2c_conf_word | OC_BIT;
      i2c0_transmit(IR_VERT_I2C_ADDR, 1, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_CONFIGURE_TOP;
      ir_i2c_conf_word = NO_CONF_WORD;
    } else {
      // Read next value TOP sensor
      i2c0_receive(IR_VERT_I2C_ADDR, 2, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_data_available = FALSE;
      ir_i2c_status = IR_I2C_READ_TOP;
    }
  }
}

void infrared_i2c_event( void ) {
  switch (ir_i2c_status) {
    case IR_I2C_IDLE :
      break;
    case IR_I2C_READ_TOP :
      ir_i2c_top = (i2c0_buf[0]<<8) | i2c0_buf[1];
      ir_i2c_data_available = TRUE;
      ir_i2c_status = IR_I2C_IDLE;
      break;
    case IR_I2C_CONFIGURE_TOP :
      ir_i2c_status = IR_I2C_IDLE;
      break;
  }
}

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

#define IR_HOR_I2C_ADDR (0x6C << 1)
#define IR_VER_I2C_ADDR (0x68 << 1)
#define IR_HOR_OC_BIT (0 << 4)
#define IR_VER_OC_BIT (1 << 4)
#define IR_START_CONV (1 << 7)

#define IR_HOR_I2C_SELECT_IR1 (0 << 5)
#define IR_HOR_I2C_SELECT_IR2 (1 << 5)

#define IR_I2C_IDLE             0
#define IR_I2C_READ_IR1         1
#define IR_I2C_IR2_SELECTED     2
#define IR_I2C_READ_TOP         3
#define IR_I2C_READ_IR2         4
#define IR_I2C_IR1_SELECTED     5
#define IR_I2C_CONFIGURE_HOR    6
#define IR_I2C_CONFIGURE_VER    7

// Global variables
int16_t ir_i2c_ir1;
int16_t ir_i2c_ir2;
int16_t ir_i2c_top;

volatile bool_t ir_i2c_done;
bool_t ir_i2c_data_available;
uint8_t ir_i2c_conf_word;
bool_t ir_i2c_conf_done;

// Local variables
static uint8_t ir_i2c_status;

#define NO_CONF_WORD 0xff
#define ValidConfWord(_x) (_x < 0x4)


void infrared_i2c_init( void ) {
  ir_i2c_done = TRUE;
  ir_i2c_data_available = FALSE;
  ir_i2c_status = IR_I2C_IDLE;
  ir_i2c_conf_word = IR_I2C_DEFAULT_CONF;
  ir_i2c_conf_done = FALSE;
}

void infrared_i2c_update( void ) {
  if (ir_i2c_done && ir_i2c_status == IR_I2C_IDLE) {
    if (ValidConfWord(ir_i2c_conf_word) && !ir_i2c_conf_done) {
      i2c0_buf[0] = 0;
      i2c0_buf[0] = ir_i2c_conf_word | IR_HOR_OC_BIT | IR_START_CONV;
      i2c0_transmit(IR_HOR_I2C_ADDR, 1, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_CONFIGURE_HOR;
    } else {
      // Read next values
      i2c0_receive(IR_HOR_I2C_ADDR, 3, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_data_available = FALSE;
      ir_i2c_status = IR_I2C_READ_IR1;
    }
  }
}

void infrared_i2c_event( void ) {
  switch (ir_i2c_status) {
    case IR_I2C_IDLE :
      break;
    case IR_I2C_READ_IR1 :
      if (bit_is_set(i2c0_buf[2],7)) {
        i2c0_receive(IR_HOR_I2C_ADDR, 3, &ir_i2c_done);
        ir_i2c_done = FALSE;
        break;
      }
      // Read IR1 value
      ir_i2c_ir1 = (i2c0_buf[0]<<8) | i2c0_buf[1];
      // Select IR2 channel
      i2c0_buf[0] = 0;
      i2c0_buf[0] = IR_HOR_I2C_SELECT_IR2 | IR_HOR_OC_BIT | ir_i2c_conf_word | IR_START_CONV;
      i2c0_transmit(IR_HOR_I2C_ADDR, 1, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_IR2_SELECTED;
      break;
    case IR_I2C_IR2_SELECTED :
      // IR2 selected, asking for TOP value
      i2c0_receive(IR_VER_I2C_ADDR, 2, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_READ_TOP;
      break;
    case IR_I2C_READ_TOP :
      // Read TOP value
      ir_i2c_top = (i2c0_buf[0]<<8) | i2c0_buf[1];
      // Asking for IR2 value
      i2c0_receive(IR_HOR_I2C_ADDR, 3, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_READ_IR2;
      break;
    case IR_I2C_READ_IR2 :
      // Read IR2 value
      if (bit_is_set(i2c0_buf[2],7)) {
        i2c0_receive(IR_HOR_I2C_ADDR, 3, &ir_i2c_done);
        ir_i2c_done = FALSE;
        break;
      }
      ir_i2c_ir2 = (i2c0_buf[0]<<8) | i2c0_buf[1];
      ir_i2c_data_available = TRUE;
      // Select IR1 channel
      i2c0_buf[0] = 0;
      i2c0_buf[0] = IR_HOR_I2C_SELECT_IR1 | IR_HOR_OC_BIT | ir_i2c_conf_word | IR_START_CONV;
      i2c0_transmit(IR_HOR_I2C_ADDR, 1, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_IR1_SELECTED;
      break;
    case IR_I2C_IR1_SELECTED :
      // End reading cycle
      ir_i2c_status = IR_I2C_IDLE;
      break;
    case IR_I2C_CONFIGURE_HOR :
      // HOR configured, now configuring TOP
      i2c0_buf[0] = 0;
      i2c0_buf[0] = ir_i2c_conf_word | IR_VER_OC_BIT;
      i2c0_transmit(IR_VER_I2C_ADDR, 1, &ir_i2c_done);
      ir_i2c_done = FALSE;
      ir_i2c_status = IR_I2C_CONFIGURE_VER;
      break;
    case IR_I2C_CONFIGURE_VER :
      // VER configured, end conf cycle
      ir_i2c_conf_done = TRUE;
      ir_i2c_status = IR_I2C_IDLE;
      break;
  }
}

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

#include "sensors/infrared_i2c.h"

// IR I2C definitions
#define IR_HOR_I2C_ADDR (0x6C << 1)
#define IR_VER_I2C_ADDR (0x68 << 1)
#define IR_SAMPLE_RATE_SELECT (0 << 2)
#define IR_HOR_OC_BIT (0 << 4)
#define IR_VER_OC_BIT (1 << 4)
#define IR_HOR_I2C_SELECT_IR1 (0 << 5)
#define IR_HOR_I2C_SELECT_IR2 (1 << 5)
#define IR_START_CONV (1 << 7)


#ifndef IR_I2C_IR1_NEUTRAL
#define IR_I2C_IR1_NEUTRAL 0
#endif

#ifndef IR_I2C_IR2_NEUTRAL
#define IR_I2C_IR2_NEUTRAL 0
#endif

#ifndef IR_I2C_TOP_NEUTRAL
#define IR_I2C_TOP_NEUTRAL 0
#endif

struct Infrared_raw ir_i2c;
bool ir_i2c_data_hor_available, ir_i2c_data_ver_available;
uint8_t ir_i2c_conf_word;
bool ir_i2c_conf_hor_done, ir_i2c_conf_ver_done;

// Local variables
#define IR_I2C_IDLE             0
#define IR_I2C_READ_IR1         1
#define IR_I2C_IR2_SELECTED     2
#define IR_I2C_READ_IR2         3
#define IR_I2C_IR1_SELECTED     4
#define IR_I2C_CONFIGURE_HOR    5

static uint8_t ir_i2c_hor_status;

#define NO_CONF_WORD 0xff
#define ValidConfWord(_x) (_x < 0x4)

// I2C structure
struct i2c_transaction irh_trans, irv_trans;

// Standard infrared implementation
void infrared_init(void)
{
  infrared_i2c_init();
}

void infrared_update(void)
{
  infrared_i2c_update();
}

void infrared_event(void)
{
  infrared_i2cEvent();
}

/** Initialisation
 */
void infrared_i2c_init(void)
{
  ir_i2c_data_hor_available = false;
  ir_i2c_data_ver_available = false;
  ir_i2c_hor_status = IR_I2C_IDLE;
  ir_i2c_conf_word = IR_I2C_DEFAULT_CONF;
  ir_i2c_conf_hor_done = false;
  ir_i2c_conf_ver_done = false;
  irh_trans.status = I2CTransDone;
  irv_trans.status = I2CTransDone;

  infrared_struct_init();
}

void infrared_i2c_update(void)
{
#if ! (defined SITL || defined HITL)
  // IR horizontal
  if (irh_trans.status == I2CTransDone && ir_i2c_hor_status == IR_I2C_IDLE) {
    if (ValidConfWord(ir_i2c_conf_word) && !ir_i2c_conf_hor_done) {
      irh_trans.buf[0] = ir_i2c_conf_word | IR_HOR_OC_BIT | IR_START_CONV ;
      i2c_transmit(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 1);
      ir_i2c_hor_status = IR_I2C_CONFIGURE_HOR;
    } else {
      // Read next values
      i2c_receive(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 3);
      ir_i2c_data_hor_available = false;
      ir_i2c_hor_status = IR_I2C_READ_IR1;
    }
  }
  // IR vertical
  if (irv_trans.status == I2CTransDone) {
    if (ValidConfWord(ir_i2c_conf_word) && !ir_i2c_conf_ver_done) {
      irv_trans.buf[0] = ir_i2c_conf_word | IR_VER_OC_BIT;
      i2c_transmit(&i2c0, &irv_trans, IR_VER_I2C_ADDR, 1);
    } else {
      // Read next values
      i2c_receive(&i2c0, &irv_trans, IR_VER_I2C_ADDR, 2);
      ir_i2c_data_ver_available = false;
    }
  }
#endif /* SITL || HITL */
}

#define FilterIR(_ir_prev, _ir_next) (((1<<ir_i2c_conf_word)*_ir_prev + _ir_next) / ((1<<ir_i2c_conf_word) + 1))

void infrared_i2c_hor_event(void)
{
#if ! (defined SITL || defined HITL)
  irh_trans.status = I2CTransDone;
  switch (ir_i2c_hor_status) {
    case IR_I2C_IDLE :
      break;
    case IR_I2C_READ_IR1 :
      if (bit_is_set(irh_trans.buf[2], 7)) {
        i2c_receive(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 3);
        break;
      }
      // Read IR1 value
      int16_t ir1 = (irh_trans.buf[0] << 8) | irh_trans.buf[1];
      ir1 = ir1 - (IR_I2C_IR1_NEUTRAL << ir_i2c_conf_word);
      ir_i2c.ir1 = FilterIR(ir_i2c.ir1, ir1);
      // Select IR2 channel
      irh_trans.buf[0] = IR_HOR_I2C_SELECT_IR2 | IR_HOR_OC_BIT | ir_i2c_conf_word | IR_START_CONV;
      i2c_transmit(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 1);
      ir_i2c_hor_status = IR_I2C_IR2_SELECTED;
      break;
    case IR_I2C_IR2_SELECTED :
      // IR2 selected, asking for IR2 value
      i2c_receive(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 3);
      ir_i2c_hor_status = IR_I2C_READ_IR2;
      break;
    case IR_I2C_READ_IR2 :
      // Read IR2 value
      if (bit_is_set(irh_trans.buf[2], 7)) {
        i2c_receive(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 3);
        break;
      }
      int16_t ir2 = (irh_trans.buf[0] << 8) | irh_trans.buf[1];
      ir2 = ir2 - (IR_I2C_IR2_NEUTRAL << ir_i2c_conf_word);
      ir_i2c.ir2 = FilterIR(ir_i2c.ir2, ir2);
      // Update estimator
      ir_i2c_data_hor_available = true;
#ifndef IR_I2C_READ_ONLY
      if (ir_i2c_data_ver_available) {
        ir_i2c_data_hor_available = false;
        ir_i2c_data_ver_available = false;
        UpdateIRValue(ir_i2c);
      }
#endif
      // Select IR1 channel
      irh_trans.buf[0] = IR_HOR_I2C_SELECT_IR1 | IR_HOR_OC_BIT | ir_i2c_conf_word | IR_START_CONV;
      i2c_transmit(&i2c0, &irh_trans, IR_HOR_I2C_ADDR, 1);
      ir_i2c_hor_status = IR_I2C_IR1_SELECTED;
      break;
    case IR_I2C_IR1_SELECTED :
      // End reading cycle
      ir_i2c_hor_status = IR_I2C_IDLE;
      break;
    case IR_I2C_CONFIGURE_HOR :
      // End conf cycle
      ir_i2c_conf_hor_done = true;
      ir_i2c_hor_status = IR_I2C_IDLE;
      break;
  }
#endif /* !SITL && !HITL */
}

void infrared_i2c_ver_event(void)
{
#if ! (defined SITL || defined HITL)
  irv_trans.status = I2CTransDone;
  // Read TOP value
  if (irv_trans.type == I2CTransRx) {
    int16_t ir3 = (irv_trans.buf[0] << 8) | irv_trans.buf[1];
    ir3 = ir3 - (IR_I2C_TOP_NEUTRAL << ir_i2c_conf_word);
    ir_i2c.ir3 = FilterIR(ir_i2c.ir3, ir3);
    ir_i2c_data_ver_available = true;
#ifndef IR_I2C_READ_ONLY
    if (ir_i2c_data_hor_available) {
      ir_i2c_data_hor_available = false;
      ir_i2c_data_ver_available = false;
      UpdateIRValue(ir_i2c);
    }
#endif
  }
  if (irv_trans.type == I2CTransTx) {
    ir_i2c_conf_ver_done = true;
  }
#endif /* !SITL && !HITL */
}


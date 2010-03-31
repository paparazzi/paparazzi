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

#include "uart.h"
#include "messages.h"
#include "downlink.h"

// Macros
#define IR_VERT_I2C_ADDR 0xEA
#define IR_VERT_I2C_REG 0x07

#ifndef IR_IR1_SIGN
#define IR_IR1_SIGN 1
#endif // IR_IR1_SIGN

#ifndef IR_IR2_SIGN
#define IR_IR2_SIGN 1
#endif // IR_IR2_SIGN

#ifndef IR_TOP_SIGN
#define IR_TOP_SIGN 1
#endif // IR_TOP_SIGN

/* Sensor installation */
#if defined IR_HORIZ_SENSOR_ALIGNED
/* IR1 on the lateral axis, IR2 on the longitudal axis */
#define IR_RollOfIrs(_ir1, _ir2) (_ir1)
#define IR_PitchOfIrs(_ir1, _ir2) (_ir2)
#elif IR_HORIZ_SENSOR_TILTED
/* IR1 rear-left -- front-right, IR2 rear-right -- front-left
   IR1_SIGN and IR2_SIGN give positive values when it's warm on the right side
*/
#define IR_RollOfIrs(_ir1, _ir2) (_ir1 + _ir2)
#define IR_PitchOfIrs(_ir1, _ir2) (-(_ir1) + _ir2)
#endif
/* Vertical sensor */
#ifndef IR_TopOfIr
#define IR_TopOfIr(_ir) ((IR_TOP_SIGN)*(_ir))
#endif

#define IR_I2C_IDLE       0
#define IR_I2C_READ_TOP   1

// Global variables
float ir_i2c_roll_neutral;
float ir_i2c_pitch_neutral;

int16_t ir_i2c_ir1;
int16_t ir_i2c_ir2;
int16_t ir_i2c_roll;
int16_t ir_i2c_pitch;
int16_t ir_i2c_top;

// Local variables
volatile bool_t ir_i2c_done;
static uint8_t ir_i2c_status;

void infrared_i2c_init( void ) {
  ir_i2c_roll_neutral  = RadOfDeg(IR_ROLL_NEUTRAL_DEFAULT);
  ir_i2c_pitch_neutral = RadOfDeg(IR_PITCH_NEUTRAL_DEFAULT);

  ir_i2c_status = IR_I2C_IDLE;

  // Transmit sensor conf
  // TODO
}

void infrared_i2c_update( void ) {
  if (ir_i2c_done && ir_i2c_status == IR_I2C_IDLE) {
    // Read next value TOP sensor
    i2c0_receive(IR_VERT_I2C_ADDR, 2, &ir_i2c_done);
    ir_i2c_status = IR_I2C_READ_TOP;
    ir_i2c_done = FALSE;
  }
}

void infrared_i2c_event( void ) {
  switch (ir_i2c_status) {
    case IR_I2C_IDLE :
      break;
    case IR_I2C_READ_TOP :
      ir_i2c_top = (i2c0_buf[0]<<8) | i2c0_buf[1];
      ir_i2c_status = IR_I2C_IDLE;
      break;
  }

}



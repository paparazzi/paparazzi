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
#include "estimator.h"

// IR I2C definitions
#define IR_HOR_I2C_ADDR (0x6C << 1)
#define IR_VER_I2C_ADDR (0x68 << 1)
#define IR_SAMPLE_RATE_SELECT (0 << 2)
#define IR_HOR_OC_BIT (0 << 4)
#define IR_VER_OC_BIT (1 << 4)
#define IR_HOR_I2C_SELECT_IR1 (0 << 5)
#define IR_HOR_I2C_SELECT_IR2 (1 << 5)
#define IR_START_CONV (1 << 7)


#ifndef IR_IR1_NEUTRAL
#define IR_IR1_NEUTRAL 0
#endif

#ifndef IR_IR2_NEUTRAL
#define IR_IR2_NEUTRAL 0
#endif

#ifndef IR_TOP_NEUTRAL
#define IR_TOP_NEUTRAL 0
#endif

// Standard infrared interface
int16_t ir_ir1;
int16_t ir_ir2;
int16_t ir_roll;
int16_t ir_pitch;
int16_t ir_top;

#ifndef IR_IR1_SIGN
#define IR_IR1_SIGN 1
#endif // IR_IR1_SIGN

#ifndef IR_IR2_SIGN
#define IR_IR2_SIGN 1
#endif // IR_IR2_SIGN

#ifndef IR_TOP_SIGN
#define IR_TOP_SIGN 1
#endif // IR_TOP_SIGN

float ir_roll_neutral;
float ir_pitch_neutral;

float ir_correction_left;
float ir_correction_right;
float ir_correction_down;
float ir_correction_up;

#ifndef IR_CORRECTION_LEFT
#define IR_CORRECTION_LEFT 1.
#endif

#ifndef IR_CORRECTION_RIGHT
#define IR_CORRECTION_RIGHT 1.
#endif

#ifndef IR_CORRECTION_UP
#define IR_CORRECTION_UP 1.
#endif

#ifndef IR_CORRECTION_DOWN
#define IR_CORRECTION_DOWN 1.
#endif

float ir_lateral_correction;
float ir_longitudinal_correction;
float ir_vertical_correction;

#ifndef IR_LATERAL_CORRECTION
#define IR_LATERAL_CORRECTION 1.
#endif

#ifndef IR_LONGITUDINAL_CORRECTION
#define IR_LONGITUDINAL_CORRECTION 1.
#endif

#ifndef IR_VERTICAL_CORRECTION
#define IR_VERTICAL_CORRECTION 1.
#endif

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
/* Vertical sensor, TOP_SIGN gives positice values when it's warm on the bottom */
#define IR_TopOfIr(_ir) ((IR_TOP_SIGN)*(_ir))



// Global variables
int16_t ir_i2c_ir1;
int16_t ir_i2c_ir2;
int16_t ir_i2c_top;

float ir_i2c_phi, ir_i2c_theta;

bool_t ir_i2c_data_available;
uint8_t ir_i2c_conf_word;
bool_t ir_i2c_conf_hor_done, ir_i2c_conf_ver_done;

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

//FIXME standard infrared should not ba ADC-dependent
void ir_init(void) {}

/** Initialisation
 */
void infrared_i2c_init( void ) {
  ir_i2c_data_available = FALSE;
  ir_i2c_hor_status = IR_I2C_IDLE;
  ir_i2c_conf_word = IR_I2C_DEFAULT_CONF;
  ir_i2c_conf_hor_done = FALSE;
  ir_i2c_conf_ver_done = FALSE;
  irh_trans.status = I2CTransDone;
  irv_trans.status = I2CTransDone;

  // Initialisation of standard infrared interface
  ir_roll_neutral  = RadOfDeg(IR_ROLL_NEUTRAL_DEFAULT);
  ir_pitch_neutral = RadOfDeg(IR_PITCH_NEUTRAL_DEFAULT);

  ir_correction_left = IR_CORRECTION_LEFT;
  ir_correction_right = IR_CORRECTION_RIGHT;
  ir_correction_up = IR_CORRECTION_UP;
  ir_correction_down = IR_CORRECTION_DOWN;

  ir_lateral_correction = IR_LATERAL_CORRECTION;
  ir_longitudinal_correction = IR_LONGITUDINAL_CORRECTION;
  ir_vertical_correction = IR_VERTICAL_CORRECTION;
}

#include "led.h"
void infrared_i2c_update( void ) {
#if ! (defined SITL || defined HITL)
  // IR horizontal
  if (irh_trans.status == I2CTransDone && ir_i2c_hor_status == IR_I2C_IDLE) {
    if (ValidConfWord(ir_i2c_conf_word) && !ir_i2c_conf_hor_done) {
      irh_trans.buf[0] = ir_i2c_conf_word | IR_HOR_OC_BIT | IR_START_CONV ;
      I2CTransmit(i2c0, irh_trans, IR_HOR_I2C_ADDR, 1);
      ir_i2c_hor_status = IR_I2C_CONFIGURE_HOR;
    } else {
      // Read next values
      I2CReceive(i2c0, irh_trans, IR_HOR_I2C_ADDR, 3);
      ir_i2c_data_available = FALSE;
      ir_i2c_hor_status = IR_I2C_READ_IR1;
    }
  }
  // IR vertical
  if (irv_trans.status == I2CTransDone) {
    if (ValidConfWord(ir_i2c_conf_word) && !ir_i2c_conf_ver_done) {
      irv_trans.buf[0] = ir_i2c_conf_word | IR_VER_OC_BIT;
      I2CTransmit(i2c0, irv_trans, IR_VER_I2C_ADDR, 1);
    } else {
      // Read next values
      I2CReceive(i2c0, irv_trans, IR_VER_I2C_ADDR, 2);
      ir_i2c_data_available = FALSE;
    }
  }
#else /* SITL || HITL */
/** ir_roll and ir_pitch set by simulator in sim_ir.c */
  estimator_update_state_infrared();
#endif
}

#define FilterIR(_ir_prev, _ir_next) (((1<<ir_i2c_conf_word)*_ir_prev + _ir_next) / ((1<<ir_i2c_conf_word) + 1))

void infrared_i2c_hor_event( void ) {
#if ! (defined SITL || defined HITL)
  irh_trans.status = I2CTransDone;
  switch (ir_i2c_hor_status) {
    case IR_I2C_IDLE :
      break;
    case IR_I2C_READ_IR1 :
      if (bit_is_set(irh_trans.buf[2],7)) {
        I2CReceive(i2c0, irh_trans, IR_HOR_I2C_ADDR, 3);
        break;
      }
      // Read IR1 value
      int16_t ir1 = (irh_trans.buf[0]<<8) | irh_trans.buf[1];
      ir_i2c_ir1 = FilterIR(ir_i2c_ir1, ir1);
      // Select IR2 channel
      irh_trans.buf[0] = IR_HOR_I2C_SELECT_IR2 | IR_HOR_OC_BIT | ir_i2c_conf_word | IR_START_CONV;
      I2CTransmit(i2c0, irh_trans, IR_HOR_I2C_ADDR, 1);
      ir_i2c_hor_status = IR_I2C_IR2_SELECTED;
      break;
    case IR_I2C_IR2_SELECTED :
      // IR2 selected, asking for IR2 value
      I2CReceive(i2c0, irh_trans, IR_HOR_I2C_ADDR, 3);
      ir_i2c_hor_status = IR_I2C_READ_IR2;
      break;
    case IR_I2C_READ_IR2 :
      // Read IR2 value
      if (bit_is_set(irh_trans.buf[2],7)) {
        I2CReceive(i2c0, irh_trans, IR_HOR_I2C_ADDR, 3);
        break;
      }
      int16_t ir2 = (irh_trans.buf[0]<<8) | irh_trans.buf[1];
      ir_i2c_ir2 = FilterIR(ir_i2c_ir2, ir2);
      // Update estimator
      ir_i2c_data_available = TRUE;
      ir_update();
      estimator_update_state_infrared();
      // Select IR1 channel
      irh_trans.buf[0] = IR_HOR_I2C_SELECT_IR1 | IR_HOR_OC_BIT | ir_i2c_conf_word | IR_START_CONV;
      I2CTransmit(i2c0, irh_trans, IR_HOR_I2C_ADDR, 1);
      ir_i2c_hor_status = IR_I2C_IR1_SELECTED;
      break;
    case IR_I2C_IR1_SELECTED :
      // End reading cycle
      ir_i2c_hor_status = IR_I2C_IDLE;
      break;
    case IR_I2C_CONFIGURE_HOR :
      // End conf cycle
      ir_i2c_conf_hor_done = TRUE;
      ir_i2c_hor_status = IR_I2C_IDLE;
      break;
  }
#endif /* !SITL && !HITL */
}

void infrared_i2c_ver_event( void ) {
#if ! (defined SITL || defined HITL)
  irv_trans.status = I2CTransDone;
  // Read TOP value
  if (irv_trans.type == I2CTransRx) {
    int16_t top = (irv_trans.buf[0]<<8) | irv_trans.buf[1];
    ir_i2c_top = FilterIR(ir_i2c_top, top);
    ir_i2c_data_available = TRUE;
    ir_update();
    estimator_update_state_infrared();
  }
  if (irv_trans.type == I2CTransTx) {
    ir_i2c_conf_ver_done = TRUE;
  }
#endif /* !SITL && !HITL */
}

#include "stdio.h"

void ir_update(void) {
  ir_ir1 = (IR_IR1_SIGN)*(ir_i2c_ir1 - (IR_IR1_NEUTRAL << ir_i2c_conf_word));
  ir_ir2 = (IR_IR2_SIGN)*(ir_i2c_ir2 - (IR_IR2_NEUTRAL << ir_i2c_conf_word));
  ir_roll = ir_lateral_correction * IR_RollOfIrs(ir_ir1, ir_ir2);
  ir_pitch = ir_longitudinal_correction * IR_PitchOfIrs(ir_ir1, ir_ir2);
  ir_top =  ir_vertical_correction * IR_TopOfIr(ir_i2c_top - (IR_TOP_NEUTRAL << ir_i2c_conf_word));
}

void estimator_update_state_infrared(void) {

  estimator_phi = atan2(ir_roll, ir_top) - ir_roll_neutral;
  estimator_theta = atan2(ir_pitch, ir_top) - ir_pitch_neutral;

  if (estimator_theta < -M_PI_2)
    estimator_theta += M_PI;
  else if (estimator_theta > M_PI_2)
    estimator_theta -= M_PI;

  if (estimator_phi >= 0)
    estimator_phi *= ir_correction_right;
  else
    estimator_phi *= ir_correction_left;

  if (estimator_theta >= 0)
    estimator_theta *= ir_correction_up;
  else
    estimator_theta *= ir_correction_down;

}

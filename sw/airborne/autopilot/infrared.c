/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

#include "adc.h"
#include "infrared.h"
#include "autopilot.h"
#include "estimator.h"

int16_t ir_roll;
int16_t ir_pitch;

int16_t ir_contrast     = IR_DEFAULT_CONTRAST;
int16_t ir_roll_neutral  = IR_ROLL_NEUTRAL_DEFAULT;
int16_t ir_pitch_neutral = IR_PITCH_NEUTRAL_DEFAULT;

#define RadOfIrFromConstrast(c) ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / c;

float ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / IR_DEFAULT_CONTRAST;

static struct adc_buf buf_ir1;
static struct adc_buf buf_ir2;

void ir_init(void) {
  RadOfIrFromConstrast(IR_DEFAULT_CONTRAST);
  adc_buf_channel(ADC_CHANNEL_IR1, &buf_ir1);
  adc_buf_channel(ADC_CHANNEL_IR2, &buf_ir2);
}

void ir_update(void) {
#ifndef SIMUL
  int16_t x1_mean = buf_ir1.sum/AV_NB_SAMPLE;
  int16_t x2_mean = buf_ir2.sum/AV_NB_SAMPLE;
  ir_roll = IR_RollOfIrs(x1_mean, x2_mean) - ir_roll_neutral;
  ir_pitch = IR_PitchOfIrs(x1_mean, x2_mean) - ir_pitch_neutral;
#else
  extern volatile int16_t simul_ir_roll, simul_ir_pitch;
  ir_roll = simul_ir_roll -  ir_roll_neutral; 
  ir_pitch = simul_ir_pitch - ir_pitch_neutral;  
#endif
}

/* 
   Contrast measurement
*/

void ir_gain_calib(void) {      // Plane nose down
  /* plane nose down -> negativ value */
  ir_contrast = - ir_pitch;
  RadOfIrFromConstrast(ir_contrast);
}

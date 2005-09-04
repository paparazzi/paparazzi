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
/** \file infrared.c
 *  \brief Regroup all functions link to \a ir
 */

#include <stdlib.h>

#include "adc.h"
#include "infrared.h"
#include "autopilot.h"
#include "estimator.h"

int16_t ir_roll;
int16_t ir_pitch;

/** Initialized to \a IR_DEFAULT_CONTRAST. Changed with calibration */
int16_t ir_contrast     = IR_DEFAULT_CONTRAST;
/** Initialized to \a IR_DEFAULT_CONTRAST.
 *  Changed with @@@@@ EST-CE QUE CA CHANGE @@@@@ */

/** \def RadOfIrFromConstrast(c)
 *  \brief Contrast measurement
 *  \note <b>Plane must be nose down !</b>
 */
#define RadOfIrFromConstrast(c) ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / c;

/** rad_of_ir variable factor: let convert \a ir value in radian.
 *  Initialized with airframe \a IR_RAD_OF_IR_CONTRAST and \a IR_DEFAULT_CONTRAST constants. \n
 *  Change when \a lls work.
 */
float ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / IR_DEFAULT_CONTRAST;

static struct adc_buf buf_ir1;
static struct adc_buf buf_ir2;

/** \fn void ir_init(void)
 *  \brief Initialisation of \a ir */
/** Initialize \a ir with the \a IR_DEFAULT_CONTRAST \n
 *  Initialize \a adc_buf_channel
 */
#ifndef ADC_CHANNEL_IR_NB_SAMPLES
#define ADC_CHANNEL_IR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#else
#warning " ADC_CHANNEL_IR_NB_SAMPLES";
#endif
void ir_init(void) {
  RadOfIrFromConstrast(IR_DEFAULT_CONTRAST);
  adc_buf_channel(ADC_CHANNEL_IR1, &buf_ir1, ADC_CHANNEL_IR_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IR2, &buf_ir2, ADC_CHANNEL_IR_NB_SAMPLES);
}

/** \fn void ir_update(void)
 *  \brief Update \a ir */
/** Different if the \a SIMUL flag is true or not. \n
 */
void ir_update(void) {
#ifndef SIMUL
  int16_t x1_mean = buf_ir1.sum/buf_ir1.av_nb_sample;
  int16_t x2_mean = buf_ir2.sum/buf_ir2.av_nb_sample;
  ir_roll = IR_RollOfIrs(x1_mean, x2_mean) - IR_ADC_ROLL_NEUTRAL;
  ir_pitch = IR_PitchOfIrs(x1_mean, x2_mean) - IR_ADC_PITCH_NEUTRAL;
#else
  extern volatile int16_t simul_ir_roll, simul_ir_pitch;
  ir_roll = simul_ir_roll - IR_ADC_ROLL_NEUTRAL; 
  ir_pitch = simul_ir_pitch - IR_ADC_PITCH_NEUTRAL;  
#endif
}

/** \fn void ir_gain_calib(void)
 *  \brief Contrast measurement
 *  \note <b>Plane must be nose down !</b>
 */
void ir_gain_calib(void) {
  /* plane nose down -> negativ value */
  ir_contrast = abs(ir_pitch);
  RadOfIrFromConstrast(ir_contrast);
}

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
#include "gps.h"
#include "autopilot.h"
#include "estimator.h"
#include "ap_downlink.h"
#include "sys_time.h"
#include "airframe.h"

#if defined IR_ESTIMATED_PHI_PI_4 || defined IR_ESTIMATED_PHI_MINUS_PI_4 || defined IR_ESTIMATED_THETA_PI_4
#error "IR_ESTIMATED_PHI_PI_4 correction has been deprecated. Please remove the definition from your airframe config file"
#endif


int16_t ir_ir1;
int16_t ir_ir2;
int16_t ir_roll;
int16_t ir_pitch;
int16_t ir_top;

float ir_roll_neutral;
float ir_pitch_neutral;

float ir_correction_left;
float ir_correction_right;
float ir_correction_down;
float ir_correction_up;

#if !defined IR_CORRECTION_LEFT
#define IR_CORRECTION_LEFT 1.
#endif

#if !defined IR_CORRECTION_RIGHT
#define IR_CORRECTION_RIGHT 1.
#endif

#if !defined IR_CORRECTION_UP
#define IR_CORRECTION_UP 1.
#endif

#if !defined IR_CORRECTION_DOWN
#define IR_CORRECTION_DOWN 1.
#endif

#ifndef SITL
static struct adc_buf buf_ir1;
static struct adc_buf buf_ir2;
#endif

#ifdef ADC_CHANNEL_IR_TOP
static struct adc_buf buf_ir_top;
#endif

#ifndef ADC_CHANNEL_IR_NB_SAMPLES
#define ADC_CHANNEL_IR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

float ir_lateral_correction;
float ir_longitudinal_correction;
float ir_vertical_correction;


#ifdef IR_360_LATERAL_CORRECTION
#error "IR_360_LATERAL_CORRECTION now IR_LATERAL_CORRECTION"
#endif

#ifdef IR_360_LONGITUDINAL_CORRECTION
#error "IR_360_LONGITUDINAL_CORRECTION now IR_LONGITUDINAL_CORRECTION"
#endif

#ifdef IR_360_VERTICAL_CORRECTION
#error "IR_360_VERTICAL_CORRECTION now IR_VERTICAL_CORRECTION"
#endif

#ifndef IR_LATERAL_CORRECTION
#define IR_LATERAL_CORRECTION 1.
#endif

#ifndef IR_LONGITUDINAL_CORRECTION
#define IR_LONGITUDINAL_CORRECTION 1.
#endif

#ifndef IR_VERTICAL_CORRECTION
#define IR_VERTICAL_CORRECTION 1.
#endif

#ifdef IR_360
#warning "IR_360 flag deprecated. Now default"
#endif


/** \brief Initialisation of \a ir */
/** Initialize \a adc_buf_channel
 */
void ir_init(void) {
#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_IR1, &buf_ir1, ADC_CHANNEL_IR_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IR2, &buf_ir2, ADC_CHANNEL_IR_NB_SAMPLES);
#endif

#ifdef ADC_CHANNEL_IR_TOP
  adc_buf_channel(ADC_CHANNEL_IR_TOP, &buf_ir_top, ADC_CHANNEL_IR_NB_SAMPLES);
#endif
 
  ir_roll_neutral  = RadOfDeg(IR_ROLL_NEUTRAL_DEFAULT);
  ir_pitch_neutral = RadOfDeg(IR_PITCH_NEUTRAL_DEFAULT);

#if defined IR_CORRECTION_LEFT && defined IR_CORRECTION_RIGHT
  ir_correction_left = IR_CORRECTION_LEFT;
  ir_correction_right = IR_CORRECTION_RIGHT;
#endif

#if defined IR_CORRECTION_UP && defined IR_CORRECTION_DOWN
  ir_correction_up = IR_CORRECTION_UP;
  ir_correction_down = IR_CORRECTION_DOWN;
#endif

  ir_lateral_correction = IR_LATERAL_CORRECTION;
  ir_longitudinal_correction = IR_LONGITUDINAL_CORRECTION;
  ir_vertical_correction = IR_VERTICAL_CORRECTION;

#ifndef ADC_CHANNEL_IR_TOP
  ir_top = IR_DEFAULT_CONTRAST;
#endif
}

#if defined IR_ADC_ROLL_NEUTRAL || defined IR_ADC_PITCH_NEUTRAL
#error "Neutrals on ROLL and PITCH deprecated. Please define IR_ADC_IR1_NEUTRAL
and IR_ADC_IR2_NEUTRAL"
#endif

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
#define IR_RollOfIrs(_ir1, _ir2) ((int8_t)(IR_IR1_SIGN)*(_ir1))
#define IR_PitchOfIrs(_ir1, _ir2) ((int8_t)(IR_IR2_SIGN)*(_ir2))
#elif IR_HORIZ_SENSOR_TILTED
/* IR1 rear-left -- front-right, IR2 rear-right -- front-left
   IR1_SIGN and IR2_SIGN give positive values when it's warm on the right side
*/
#define IR_RollOfIrs(_ir1, _ir2) ((int8_t)(IR_IR1_SIGN)*(_ir1) + (int8_t)(IR_IR2_SIGN)*(_ir2))
#define IR_PitchOfIrs(_ir1, _ir2) (-(int8_t)(IR_IR1_SIGN)*(_ir1) + (int8_t)(IR_IR2_SIGN)*(_ir2))
#endif

#ifdef ADC_CHANNEL_IR_TOP
#ifndef IR_TopOfIr
#define IR_TopOfIr(_ir) ((int8_t)(IR_TOP_SIGN)*(_ir))
#endif 
#endif


/** \brief Update \a ir_roll and ir_pitch from ADCs or from simulator
 * message in HITL and SITL modes
 */
void ir_update(void) {
#if ! (defined SITL || defined HITL)
  ir_ir1 = buf_ir1.sum/buf_ir1.av_nb_sample - IR_ADC_IR1_NEUTRAL;
  ir_ir2 = buf_ir2.sum/buf_ir2.av_nb_sample - IR_ADC_IR2_NEUTRAL;
  ir_roll = IR_RollOfIrs(ir_ir1, ir_ir2);
  ir_pitch = IR_PitchOfIrs(ir_ir1, ir_ir2);
#ifdef ADC_CHANNEL_IR_TOP
  ir_top =  IR_TopOfIr(buf_ir_top.sum/buf_ir_top.av_nb_sample - IR_ADC_TOP_NEUTRAL);
#endif // IR_TOP
#endif /* !SITL && !HITL */
/** #else ir_roll set by simulator in sim_ir.c */
}


static inline void ir_correction( void ) {
    /* infrared compensation */
#if defined IR_CORRECTION_LEFT && defined IR_CORRECTION_RIGHT
    if (estimator_phi >= 0) 
      estimator_phi *= ir_correction_right;
    else
      estimator_phi *= ir_correction_left;
#endif
    
#if defined IR_CORRECTION_UP && defined IR_CORRECTION_DOWN
    if (estimator_theta >= 0)
      estimator_theta *= ir_correction_up;
    else
      estimator_theta *= ir_correction_down;
#endif
}

void estimator_update_state_infrared( void ) {
  float tmp_ir_roll = ir_roll * ir_lateral_correction;
  float tmp_ir_pitch = ir_pitch * ir_longitudinal_correction;
  float tmp_ir_top = ir_top * ir_vertical_correction;

  estimator_phi  = atan2(tmp_ir_roll, tmp_ir_top) - ir_roll_neutral;
  
  estimator_theta  = atan2(tmp_ir_pitch, tmp_ir_top) - ir_pitch_neutral;

  if (estimator_theta < -M_PI_2)
    estimator_theta += M_PI;
  else if (estimator_theta > M_PI_2)
    estimator_theta -= M_PI;

  ir_correction();
}

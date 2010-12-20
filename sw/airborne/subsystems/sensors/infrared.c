/*
 * $Id$
 *
 * Copyright (C) 2003-2010  The Paparazzi Team
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

/** \file subsystems/sensors/infrared.c
 *  \brief Regroup all functions link to \a ir
 */




#include <stdlib.h>

#include "subsystems/sensors/infrared.h"
#include "mcu_periph/adc.h"

#include BOARD_CONFIG
#include "generated/airframe.h"


#if defined IR_ESTIMATED_PHI_PI_4 || defined IR_ESTIMATED_PHI_MINUS_PI_4 || defined IR_ESTIMATED_THETA_PI_4
#error "IR_ESTIMATED_PHI_PI_4 correction has been deprecated. Please remove the definition from your airframe config file"
#endif
#ifdef INFRARED
#error "The flag INFRARED has been deprecated. Please replace it with USE_INFRARED."
#endif
#if defined IR_ADC_ROLL_NEUTRAL || defined IR_ADC_PITCH_NEUTRAL
#error "Neutrals on ROLL and PITCH deprecated. Please define IR_ADC_IR1_NEUTRAL and IR_ADC_IR2_NEUTRAL"
#endif
#ifdef IR_360
#warning "IR_360 flag deprecated. Now default"
#endif
#ifdef IR_360_LATERAL_CORRECTION
#error "IR_360_LATERAL_CORRECTION now IR_LATERAL_CORRECTION"
#endif

#ifdef IR_360_LONGITUDINAL_CORRECTION
#error "IR_360_LONGITUDINAL_CORRECTION now IR_LONGITUDINAL_CORRECTION"
#endif

#ifdef IR_360_VERTICAL_CORRECTION
#error "IR_360_VERTICAL_CORRECTION now IR_VERTICAL_CORRECTION"
#endif


struct Infrared infrared;

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

#if ! (defined SITL || defined HITL)
static struct adc_buf buf_ir1;
static struct adc_buf buf_ir2;

#ifdef ADC_CHANNEL_IR_TOP
static struct adc_buf buf_ir_top;
#endif
#endif


#ifndef ADC_CHANNEL_IR_NB_SAMPLES
#define ADC_CHANNEL_IR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
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


/** \brief Initialisation of \a ir */
/** Initialize \a adc_buf_channel
 */
void infrared_init(void) {
#if ! (defined SITL || defined HITL)
  adc_buf_channel(ADC_CHANNEL_IR1, &buf_ir1, ADC_CHANNEL_IR_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IR2, &buf_ir2, ADC_CHANNEL_IR_NB_SAMPLES);
#ifdef ADC_CHANNEL_IR_TOP
  adc_buf_channel(ADC_CHANNEL_IR_TOP, &buf_ir_top, ADC_CHANNEL_IR_NB_SAMPLES);
#endif
#endif


  infrared.roll_neutral  = RadOfDeg(IR_ROLL_NEUTRAL_DEFAULT);
  infrared.pitch_neutral = RadOfDeg(IR_PITCH_NEUTRAL_DEFAULT);

  infrared.correction_left = IR_CORRECTION_LEFT;
  infrared.correction_right = IR_CORRECTION_RIGHT;
  infrared.correction_up = IR_CORRECTION_UP;
  infrared.correction_down = IR_CORRECTION_DOWN;

  infrared.lateral_correction = IR_LATERAL_CORRECTION;
  infrared.longitudinal_correction = IR_LONGITUDINAL_CORRECTION;
  infrared.vertical_correction = IR_VERTICAL_CORRECTION;

#if ! (defined ADC_CHANNEL_IR_TOP || defined HITL || defined SITL)
  infrared.top = IR_DEFAULT_CONTRAST;
#endif
}

#ifndef IR_IR1_SIGN
#define IR_IR1_SIGN 1
#endif /* IR_IR1_SIGN */

#ifndef IR_IR2_SIGN
#define IR_IR2_SIGN 1
#endif /* IR_IR2_SIGN */

#ifndef IR_TOP_SIGN
#define IR_TOP_SIGN 1
#endif /* IR_TOP_SIGN */

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

#ifdef ADC_CHANNEL_IR_TOP
#ifndef IR_TopOfIr
#define IR_TopOfIr(_ir) ((IR_TOP_SIGN)*(_ir))
#endif
#endif


/** \brief Update \a ir_roll and ir_pitch from ADCs or from simulator
 * message in HITL and SITL modes
 */
void infrared_update(void) {
#if ! (defined SITL || defined HITL)
  infrared.ir1 = (IR_IR1_SIGN)*((int32_t)(buf_ir1.sum/buf_ir1.av_nb_sample) - IR_ADC_IR1_NEUTRAL);
  infrared.ir2 = (IR_IR2_SIGN)*((int32_t)(buf_ir2.sum/buf_ir2.av_nb_sample) - IR_ADC_IR2_NEUTRAL);
  infrared.roll = infrared.lateral_correction * IR_RollOfIrs(infrared.ir1, infrared.ir2);
  infrared.pitch = infrared.longitudinal_correction * IR_PitchOfIrs(infrared.ir1, infrared.ir2);
#ifdef ADC_CHANNEL_IR_TOP
  infrared.ir3 = (int32_t)(buf_ir_top.sum/buf_ir_top.av_nb_sample) - IR_ADC_TOP_NEUTRAL;
  infrared.top =  infrared.vertical_correction * IR_TopOfIr(infrared.ir3);
#endif // IR_TOP
#endif /* !SITL && !HITL */
/** #else ir_roll set by simulator in sim_ir.c */
}


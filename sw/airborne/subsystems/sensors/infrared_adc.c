/*
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

/** \file subsystems/sensors/infrared_adc.c
 *  \brief Regroup all functions link to ADC \a ir
 */




#include <stdlib.h>

#include "subsystems/sensors/infrared_adc.h"
#include "mcu_periph/adc.h"

#include BOARD_CONFIG
#include "generated/airframe.h"

// TODO Specific sim implementation
#if ! (defined SITL || defined HITL)
static struct adc_buf buf_ir1;
static struct adc_buf buf_ir2;

#ifdef ADC_CHANNEL_IR_TOP
static struct adc_buf buf_ir3;
#endif
#endif

#ifndef ADC_CHANNEL_IR_NB_SAMPLES
#define ADC_CHANNEL_IR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

struct Infrared_raw ir_adc;

// Standard infrared implementation
void infrared_init(void)
{
  infrared_adc_init();
}

void infrared_update(void)
{
  infrared_adc_update();
}

/* No event with adc ir */
void infrared_event(void) {}

/** \brief Initialisation of \a ir */
/** Initialize \a adc_buf_channel
 */
void infrared_adc_init(void)
{
#if ! (defined SITL || defined HITL)
  adc_buf_channel(ADC_CHANNEL_IR1, &buf_ir1, ADC_CHANNEL_IR_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IR2, &buf_ir2, ADC_CHANNEL_IR_NB_SAMPLES);
#ifdef ADC_CHANNEL_IR_TOP
  adc_buf_channel(ADC_CHANNEL_IR_TOP, &buf_ir3, ADC_CHANNEL_IR_NB_SAMPLES);
#endif
#endif

  infrared_struct_init();

#if ! (defined ADC_CHANNEL_IR_TOP || defined HITL || defined SITL)
  ir_adc.ir3 = IR_DEFAULT_CONTRAST;
#endif
}


/** \brief Update \a ir_roll and ir_pitch from ADCs or from simulator
 * message in HITL and SITL modes
 */
void infrared_adc_update(void)
{
#if ! (defined SITL || defined HITL)
  ir_adc.ir1 = (int32_t)(buf_ir1.sum / buf_ir1.av_nb_sample) - IR_ADC_IR1_NEUTRAL;
  ir_adc.ir2 = (int32_t)(buf_ir2.sum / buf_ir2.av_nb_sample) - IR_ADC_IR2_NEUTRAL;
#ifdef ADC_CHANNEL_IR_TOP
  ir_adc.ir3 = (int32_t)(buf_ir3.sum / buf_ir3.av_nb_sample) - IR_ADC_TOP_NEUTRAL;
#endif // IR_TOP
  UpdateIRValue(ir_adc);
#endif /* !SITL && !HITL */
}


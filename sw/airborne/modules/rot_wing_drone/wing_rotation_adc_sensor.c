/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/rot_wing_drone/wing_rotation_adc_sensor.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module to read skew angle from adc sensor
 */

#include "modules/rot_wing_drone/wing_rotation_adc_sensor.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"

#include <stdlib.h>
#include "mcu_periph/adc.h"

/*** ADC channel connected to the wing rotation potentiometer */
#ifndef ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION
#define ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION ADC_5
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES 16
#endif

static struct adc_buf buf_wing_rot_pos;

// Initialization
void wing_rotation_adc_init(void)
{
  // ADC init
  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION, &buf_wing_rot_pos,
                  ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES);
}

void wing_rotation_adc_to_deg(void)
{
  float adc_wing_rotation = buf_wing_rot_pos.sum / buf_wing_rot_pos.av_nb_sample;
  float wing_angle_deg = 0.00247111 * adc_wing_rotation - 25.635294;


  // SEND ABI Message to ctr_eff_sched and other modules that want Actuator position feedback
  struct act_feedback_t feedback = {0};
  feedback.idx =  SERVO_ROTATION_MECH_IDX;
  feedback.position = 0.5 * M_PI - RadOfDeg(wing_angle_deg);
  feedback.set.position = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
}

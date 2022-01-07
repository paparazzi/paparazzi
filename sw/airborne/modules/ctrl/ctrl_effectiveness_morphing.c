/*
 * Copyright (C) 2021 Murat Bronz
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
 */

/** @file modules/ctrl/ctrl_effectiveness_morphing.c
 * Module that calculates the effectiveness matrix for a given (RoBust hexacopter here) geometry.
 * It has been written based on existing ctrl_effectiveness modules.
 */

#include "modules/ctrl/ctrl_effectiveness_morphing.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#error "You need to use WLS control allocation for this module"
#endif

#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

uint16_t adc_val1;
uint16_t adc_val2;

float gamma_1;
float gamma_2;

float R0;
float R1;


#ifndef ADC_CHANNEL_MORPHING1
#ifndef ADC_CHANNEL_MORPHING2
#error "at least one ADC_CHANNEL_MORPHING1/2 needs to be defined to use the ctrl_effectiveness_morphing module"
#endif
#endif

#ifndef ADC_CHANNEL_MORPHING_NB_SAMPLES
#define ADC_CHANNEL_MORPHING_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#ifdef ADC_CHANNEL_MORPHING1
static struct adc_buf buf_1;
#endif

#ifdef ADC_CHANNEL_MORPHING2
static struct adc_buf buf_2;
#endif

//Init the module
void ctrl_eff_morphing_init(void)
{
  adc_val1 = 0;
  adc_val2 = 0;
  R0 = 0.07; // inner radius where the root of motor arms connects
  R1 = 0.08; // Motor arm length 

#ifdef ADC_CHANNEL_MORPHING1
  adc_buf_channel(ADC_CHANNEL_MORPHING1, &buf_1, ADC_CHANNEL_MORPHING_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_MORPHING2
  adc_buf_channel(ADC_CHANNEL_MORPHING2, &buf_2, ADC_CHANNEL_MORPHING_NB_SAMPLES);
#endif
}

void ctrl_eff_morphing_periodic(void)
{
  // Keeping it like this thinking that we may have other methods
  // to switch between in the future.
  ctrl_eff_morphing_periodic_a();

}

static void calc_x_y(float gamma, float theta,  float arr[] ){
  // Calculate the 
  float sg=sinf(gamma);
  float cg=cosf(gamma);
  float st=sinf(theta);
  float ct=cosf(theta);
  float x_p = R1*sg;
  float y_p = R1*cg;
  arr[0] = R0*st + x_p*ct + y_p*st;
  arr[1] = R0*ct - x_p*st + y_p*ct;
}

void ctrl_eff_morphing_periodic_a(void)
{
  int8_t i;
  // Scaling factor which results with correct efficiency values 
  // for pitch and roll on RoBust frame.
  float scale = 0.25;
  float dist[2];

  // ADC values read from potentiometer of the servos
  float adc_val1 = buf_1.sum / buf_1.av_nb_sample;
  float adc_val2 = buf_2.sum / buf_2.av_nb_sample;

  // Converting the adc values to represent movement of the arms for +1/-1
  // Luckily the min/max rotation angle corresponds to +1/-1 [rad] ;
  // So gamma ~= arm rotations in [rad] ! 
  float gamma_1 = (adc_val1 - 1410)/ 510.;
  float gamma_2 = (adc_val2 - 1490)/ 510.;

  // Angular positions of the root of each arm in [deg]
  // (for hexa configuration for this specific distribution)
  static const float theta_arr[INDI_NUM_ACT] = { 60, 0, -60, -120, 180, 120 };

  // We want each arm to rotate in particular direction referenced by particular gamma angle
  float signed_gamma_arr[INDI_NUM_ACT] = {  -gamma_1, 
                                             gamma_1,
                                             gamma_2,
                                            -gamma_2, 
                                             gamma_2,
                                             gamma_1};
  // Apply the calculated efficiency values to the efficiency matrix
  // Only roll and pitch is changing
  for (i = 0; i < INDI_NUM_ACT; i++){
    calc_x_y(signed_gamma_arr[i], RadOfDeg(theta_arr[i]), dist);
    g1g2[0][i] = -dist[1]*scale ; // roll
    g1g2[1][i] =  dist[0]*scale ; // pitch
  }

  // Easy debug message :
  // float msg[] = {
  //   dist[0]*scale,-dist[1]*scale,
  //   g1g2[0][0], g1g2[1][0],
  // };
  // DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, msg);
}
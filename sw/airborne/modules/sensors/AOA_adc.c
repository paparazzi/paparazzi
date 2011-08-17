/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * Autor: Bruzzlee
 * Angle of Attack ADC Sensor
 * US DIGITAL MA3-A10-236-N
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

#include "modules/sensors/AOA_adc.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "estimator.h"
#include "std.h"
//Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

uint16_t adc_AOA_val;

//Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef SITL // Use ADC if not in simulation

#ifndef ADC_CHANNEL_AOA
#error "ADC_CHANNEL_AOA needs to be defined to use AOA_adc module"
#endif

#ifndef ADC_CHANNEL_AOA_NB_SAMPLES
#define ADC_CHANNEL_AOA_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

struct adc_buf buf_AOA;
float AOA_offset, AOA_filter;
float AOA, AOA_old;
#endif


void AOA_adc_init( void ) {
	AOA_offset = AOA_OFFSET;
	AOA_filter = AOA_FILTER;
	AOA_old = 0;
#ifndef SITL
	adc_buf_channel(ADC_CHANNEL_AOA, &buf_AOA, ADC_CHANNEL_AOA_NB_SAMPLES);
#endif
}

void AOA_adc_update( void ) {
#ifndef SITL
	adc_AOA_val = buf_AOA.sum / buf_AOA.av_nb_sample;

// 	PT1 filter and convert to rad
	AOA = AOA_filter * AOA_old + (1 - AOA_filter) * (adc_AOA_val*(2*M_PI)/1024-M_PI+AOA_offset);
	AOA_old = AOA;
#endif
	RunOnceEvery(30, DOWNLINK_SEND_AOA_adc(DefaultChannel, &adc_AOA_val, &AOA));

#ifdef USE_AOA
	EstimatorSetAOA(AOA);
#endif
}

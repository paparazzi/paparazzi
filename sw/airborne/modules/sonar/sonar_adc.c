/*
 * Copyright (C) 2010  Gautier Hattenberger, 2013 Tobias Münch
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

#include "generated/airframe.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

//#include "filters/median_filter.h"
#include "filters/low_pass_filter.h"

#include "mcu_periph/adc.h"
#include "subsystems/abi.h"
#ifdef SITL
#include "state.h"
#endif
#include "modules/sonar/sonar_adc.h"

/** Use low pass filter on pressure values
 */
#ifndef SONAR_USE_ADC_LOWPASS_FILTER
#define SONAR_USE_ADC_LOWPASS_FILTER TRUE
#endif

/** Time constant for second order Butterworth low pass filter
 * Default of 0.19 should give cut-off freq of 1/(2*pi*tau) ~= 1.25Hz
 */
#ifndef SONAR_ADC_LOWPASS_TAU
#define SONAR_ADC_LOWPASS_TAU 0.199
#endif

#ifdef SONAR_USE_ADC_LOWPASS_FILTER
static Butterworth2LowPass sonar_filt;
#endif

struct SonarAdc sonar_adc;

#ifndef SITL
static struct adc_buf sonar_adc_buf;
#endif

void sonar_adc_init(void)
{
  sonar_adc.meas = 0;
  sonar_adc.offset = SONAR_OFFSET;
#ifdef SONAR_USE_ADC_LOWPASS_FILTER
  init_butterworth_2_low_pass(&sonar_filt, SONAR_ADC_LOWPASS_TAU, SONAR_ADC_READ_PERIOD, 0);
#endif

#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_SONAR, &sonar_adc_buf, DEFAULT_AV_NB_SAMPLE);
#endif
}

/** Read ADC value to update sonar measurement
 */
void sonar_adc_read(void)
{
#ifndef SITL

  sonar_adc.meas = sonar_adc_buf.sum / sonar_adc_buf.av_nb_sample;
#ifdef SONAR_USE_ADC_LOWPASS_FILTER
  sonar_adc.distance = update_butterworth_2_low_pass(&sonar_filt, (float)(sonar_adc.meas - sonar_adc.offset) * SONAR_SCALE);
#else
  sonar_adc.distance = (float)(sonar_adc.meas - sonar_adc.offset) * SONAR_SCALE;
#endif  

  Bound(sonar_adc.distance, (float)SONAR_MIN_RANGE, (float)SONAR_MAX_RANGE);

#if SONAR_COMPENSATE_ROTATION
  float phi = stateGetNedToBodyEulers_f()->phi;
  float theta = stateGetNedToBodyEulers_f()->theta;
  float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));
  sonar_adc.distance =  sonar_adc.distance * gain;
#endif

#else // SITL
  sonar_adc.distance = stateGetPositionEnu_f()->z;
#endif // SITL

  // Send ABI message
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgAGL(AGL_SONAR_ADC_ID, now_ts, sonar_adc.distance);

#ifdef SENSOR_SYNC_SEND_SONAR
  // Send Telemetry report
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_adc.meas, &sonar_adc.distance);
#endif
}


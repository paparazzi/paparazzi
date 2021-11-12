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

#include "modules/sonar/sonar_adc.h"
#include "generated/airframe.h"
#include "mcu_periph/adc.h"
#include "modules/core/abi.h"
#ifdef SITL
#include "state.h"
#endif

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

/** Sonar offset.
 *  Offset value in ADC
 *  equals to the ADC value so that height is zero
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0
#endif

/** Sonar scale.
 *  Sensor sensitivity in m/adc (float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 0.0166
#endif

struct SonarAdc sonar_adc;

#ifndef SITL
static struct adc_buf sonar_adc_buf;
#endif

void sonar_adc_init(void)
{
  sonar_adc.meas = 0;
  sonar_adc.offset = SONAR_OFFSET;

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
  sonar_adc.distance = (float)(sonar_adc.meas - sonar_adc.offset) * SONAR_SCALE;
#else // SITL
  sonar_adc.distance = stateGetPositionEnu_f()->z;
  Bound(sonar_adc.distance, 0.1f, 7.0f);
#endif // SITL

  // Send ABI message
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgAGL(AGL_SONAR_ADC_ID, now_ts, sonar_adc.distance);

#ifdef SENSOR_SYNC_SEND_SONAR
  // Send Telemetry report
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_adc.meas, &sonar_adc.distance);
#endif
}


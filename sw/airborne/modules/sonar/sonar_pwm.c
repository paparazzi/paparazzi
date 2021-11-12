/*
* Copyright (C) 2020 OpenuAS
*
* Thanks to Jean-François Erdelyi & Gautier Hattenberger for ADC one
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

/**
* @file "modules/sensors/sonar_pwm.h"
* @author OpenUAS
* @brief Range sensor input via PWM
*
* Driver for a PWM based sonar range sensor
* Reads sensor using PWM input and outputs sonar distance to object
*
* Sensor example: Maxbotix EZ1
* https://www.maxbotix.com/033-using-pulse-width-pin-2.htm
*
*
*/

#include "modules/sonar/sonar_pwm.h"
#include "mcu_periph/pwm_input.h"
#include "modules/core/abi.h"

#include "filters/median_filter.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif
#ifdef SENSOR_SYNC_SEND_SONAR
#include "modules/datalink/downlink.h"
#endif
#include "generated/airframe.h"
#ifdef SITL
#include "state.h"
#endif

#ifdef SONAR_USE_PWM_FILTER
struct MedianFilterFloat sonar_filt;
#endif

/* Config parameters for sensor */

/// The input channel of the PWM based sensor
#ifndef SONAR_PWM_CHANNEL
#error "No SONAR_PWM_CHANNEL defined to use sonar_pwm module, add a line like define=SONAR_PWM_CHANNEL value=PWM_INPUT1"
#endif

/// Ranger offset value for what considered the distance should be zero
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0
#endif

/// Default to a 12 bit PWM sensor
#ifndef SONAR_PWM_PERIOD
#define SONAR_PWM_PERIOD 4096
#endif

/// Ranger scale or sensitivity as you wish
#ifndef SONAR_SCALE
#define SONAR_SCALE SONAR_PWM_PERIOD
#endif

/// The Median Filter strength
#ifndef SONAR_MEDIAN_SIZE
#define SONAR_MEDIAN_SIZE 7 //Good for a noisy sonar
#endif

/// The minimum range for the device to be able to measure
#ifndef SONAR_MIN_RANGE
#define SONAR_MIN_RANGE 0.15f //Default is a common value for regular sonars. Cannot measure closer than that
#endif

/// The maximum range for the device to be able to measure
#ifndef SONAR_MAX_RANGE
#define SONAR_MAX_RANGE 7.0f //Reasonable maximum value for regular sonars.
#endif

/// Some sensor may need an initial PWM offset (823 usec in the case of an EZ1 sensor)
#ifndef SONAR_PWM_OFFSET
#define SONAR_PWM_OFFSET 820 //822
#endif

// Enable telemetry report
#ifndef SENSOR_SYNC_SEND_SONAR
#define SENSOR_SYNC_SEND_SONAR TRUE
#endif

struct SonarPwm sonar_pwm;

/* Set the default values at initialization */
void sonar_pwm_init(void)
{
  sonar_pwm.offset = SONAR_OFFSET;
  sonar_pwm.scale = SONAR_SCALE;
  sonar_pwm.raw = 0.0f;
  #ifdef SONAR_USE_PWM_FILTER
    init_median_filter_f(&sonar_filt, SONAR_MEDIAN_SIZE);
  #endif
}

/* Read the sensor current measured value */
void sonar_pwm_read(void) {

#ifndef SITL
  // raw duty cycle in usec
  uint16_t sonar_duty_raw = get_pwm_input_duty_in_usec(SONAR_PWM_CHANNEL);

  // remove PWM offset where needed
  sonar_pwm.raw = sonar_duty_raw - SONAR_PWM_OFFSET;

#ifdef SONAR_USE_PWM_FILTER
  sonar_pwm.distance = update_median_filter_f(&sonar_filt, ((float)sonar_pwm.raw * sonar_pwm.scale) - sonar_pwm.offset);
#else
  sonar_pwm.distance = (((float)sonar_pwm.raw * sonar_pwm.scale) - sonar_pwm.offset);
#endif

  Bound(sonar_pwm.distance, (float)SONAR_MIN_RANGE, (float)SONAR_MAX_RANGE);

#if SONAR_COMPENSATE_ROTATION
  float phi = stateGetNedToBodyEulers_f()->phi;
  float theta = stateGetNedToBodyEulers_f()->theta;
  float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));
  sonar_adc.distance =  sonar_adc.distance * gain;
#endif

#else // SITL
  sonar_adc.distance = stateGetPositionEnu_f()->z;
#endif // SITL

#if USE_SONAR
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgAGL(AGL_SONAR_PWM_ID, now_ts, sonar_pwm.distance);
#endif

#ifdef SENSOR_SYNC_SEND_SONAR
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_pwm.raw, &sonar_pwm.distance);
#endif
}

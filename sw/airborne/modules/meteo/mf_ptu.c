/*
 * Copyright (C) 2014 Gautier Hattenberger
 *
 * This file is part of paparazzi

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

#include "modules/meteo/mf_ptu.h"

#include "mcu_periph/gpio.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/pwm_input.h"
#include "generated/airframe.h"

/** Default scale and offset
 *  send raw values if nothing defined in airframe file
 */
#ifndef PTU_PRESSURE_OFFSET
#define PTU_PRESSURE_OFFSET 0
#endif
#ifndef PTU_PRESSURE_SCALE
#define PTU_PRESSURE_SCALE 1.0
#endif
#ifndef PTU_TEMPERATURE_OFFSET
#define PTU_TEMPERATURE_OFFSET 0
#endif
#ifndef PTU_TEMPERATURE_SCALE
#define PTU_TEMPERATURE_SCALE 1.0
#endif
#ifndef PTU_HUMIDTY_OFFSET
#define PTU_HUMIDTY_OFFSET 0
#endif
#ifndef PTU_HUMIDTY_SCALE
#define PTU_HUMIDTY_SCALE 1.0
#endif

/** ADC buffers */
#ifndef ADC_CHANNEL_PTU_NB_SAMPLES
#define ADC_CHANNEL_PTU_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif
struct adc_buf pressure_buf;
struct adc_buf temp_buf;

/** Raw values */
uint16_t pressure_adc;
uint16_t temp_adc;
uint32_t humid_period;

#if LOG_PTU
#include "modules/loggers/sdlog_chibios.h"
bool log_ptu_started;
#endif

#if SEND_PTU
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "modules/gps/gps.h"
#endif

void mf_ptu_init(void)
{
  adc_buf_channel(ADC_CHANNEL_PRESSURE, &pressure_buf, ADC_CHANNEL_PTU_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_TEMPERATURE, &temp_buf, ADC_CHANNEL_PTU_NB_SAMPLES);

#ifdef PTU_POWER_GPIO
  gpio_setup_output(PTU_POWER_GPIO);
  gpio_set(PTU_POWER_GPIO);
#endif

  pressure_adc = 0;
  temp_adc = 0;
  humid_period = 0;

#if LOG_PTU
  log_ptu_started = false;
#endif
}

void mf_ptu_periodic(void)
{
  // Read ADC
  pressure_adc = pressure_buf.sum / pressure_buf.av_nb_sample;
  temp_adc = temp_buf.sum / temp_buf.av_nb_sample;
  // Read PWM
  humid_period = get_pwm_input_period_in_usec(PWM_INPUT_CHANNEL_HUMIDITY);

  // Log data
#if LOG_PTU
  if (pprzLogFile != -1) {
    if (!log_ptu_started) {
      sdLogWriteLog(pprzLogFile,
                    "P(adc) T(adc) H(usec) GPS_fix TOW(ms) Week Lat(1e7rad) Lon(1e7rad) HMSL(mm) gpseed(cm/s) course(1e7rad) climb(cm/s)\n");
      log_ptu_started = true;
    } else {
      sdLogWriteLog(pprzLogFile, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
                    pressure_adc, temp_adc, humid_period,
                    gps.fix, gps.tow, gps.week,
                    gps.lla_pos.lat, gps.lla_pos.lon, gps.hmsl,
                    gps.gspeed, gps.course, -gps.ned_vel.z);
    }
  }
#endif

  // Send data
#if SEND_PTU
#define PTU_DATA_SIZE 3
  float ptu_data[PTU_DATA_SIZE];
  ptu_data[0] = (float)(PTU_PRESSURE_SCALE * ((int16_t)pressure_adc - PTU_PRESSURE_OFFSET));
  ptu_data[1] = (float)(PTU_TEMPERATURE_SCALE * ((int16_t)temp_adc - PTU_TEMPERATURE_OFFSET));
  ptu_data[2] = (float)(PTU_HUMIDTY_SCALE * ((int16_t)humid_period - PTU_HUMIDTY_OFFSET));
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, PTU_DATA_SIZE, ptu_data);
#endif
}



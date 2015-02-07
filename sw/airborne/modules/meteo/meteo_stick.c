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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "modules/meteo/meteo_stick.h"

#include "subsystems/abi.h"
#include "peripherals/ads1220.h"
#include "mcu_periph/pwm_input.h"
#include "generated/airframe.h"

/** Default scale and offset
 *  send raw values if nothing defined in airframe file
 */
#ifndef MS_PRESSURE_OFFSET
#define MS_PRESSURE_OFFSET 0
#endif
#ifndef MS_PRESSURE_SCALE
#define MS_PRESSURE_SCALE 1.0
#endif
#ifndef MS_TEMPERATURE_OFFSET
#define MS_TEMPERATURE_OFFSET 0
#endif
#ifndef MS_TEMPERATURE_SCALE
#define MS_TEMPERATURE_SCALE 1.0
#endif
#ifndef MS_HUMIDTY_OFFSET
#define MS_HUMIDTY_OFFSET 0
#endif
#ifndef MS_HUMIDTY_SCALE
#define MS_HUMIDTY_SCALE 1.0
#endif
#ifndef MS_DIFF_PRESSURE_OFFSET
#define MS_DIFF_PRESSURE_OFFSET 0
#endif
#ifndef MS_DIFF_PRESSURE_SCALE
#define MS_DIFF_PRESSURE_SCALE 1.0
#endif

/** General structure */
struct MeteoStick meteo_stick;

/** Includes to log on SD card
 */
#if LOG_MS
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
bool_t log_ptu_started;
#endif

/* Includes and function to send over telemetry
 */
#if SEND_MS
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps.h"

#define MS_DATA_SIZE 4

static inline void meteo_stick_send_data(void)
{
  float ptu_data[MS_DATA_SIZE];
  ptu_data[0] = (float)(MS_PRESSURE_SCALE * ((int32_t)meteo_stick.pressure.data - MS_PRESSURE_OFFSET));
  ptu_data[1] = (float)(MS_TEMPERATURE_SCALE * ((int32_t)meteo_stick.temperature.data - MS_TEMPERATURE_OFFSET));
  ptu_data[2] = (float)(MS_HUMIDTY_SCALE * ((int32_t)meteo_stick.humidity_period - MS_HUMIDTY_OFFSET));
  ptu_data[3] = (float)(MS_DIFF_PRESSURE_SCALE * ((int32_t)meteo_stick.diff_pressure.data - MS_DIFF_PRESSURE_OFFSET));
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, MS_DATA_SIZE, ptu_data);
}

#endif



/** Init function
 */
void meteo_stick_init(void)
{
  // Init absolute pressure
  meteo_stick.pressure.config.mux = ADS1220_MUX_AIN0_AVSS;
  meteo_stick.pressure.config.gain = ADS1220_GAIN_1;
  meteo_stick.pressure.config.pga_bypass = TRUE;
  meteo_stick.pressure.config.rate = ADS1220_RATE_45_HZ;
  meteo_stick.pressure.config.conv = ADS1220_CONTINIOUS_CONVERSION;
  meteo_stick.pressure.config.vref = ADS1220_VREF_VDD;
  meteo_stick.pressure.config.idac = ADS1220_IDAC_OFF;
  meteo_stick.pressure.config.i1mux = ADS1220_IMUX_OFF;
  meteo_stick.pressure.config.i2mux = ADS1220_IMUX_OFF;
  ads1220_init(&meteo_stick.pressure, &(MS_SPI_DEV), MS_PRESSURE_SLAVE_IDX);

  // Init differential pressure
  meteo_stick.diff_pressure.config.mux = ADS1220_MUX_AIN0_AVSS;
  meteo_stick.diff_pressure.config.gain = ADS1220_GAIN_2;
  meteo_stick.diff_pressure.config.pga_bypass = TRUE;
  meteo_stick.diff_pressure.config.rate = ADS1220_RATE_45_HZ;
  meteo_stick.diff_pressure.config.conv = ADS1220_CONTINIOUS_CONVERSION;
  meteo_stick.diff_pressure.config.vref = ADS1220_VREF_VDD;
  meteo_stick.diff_pressure.config.idac = ADS1220_IDAC_OFF;
  meteo_stick.diff_pressure.config.i1mux = ADS1220_IMUX_OFF;
  meteo_stick.diff_pressure.config.i2mux = ADS1220_IMUX_OFF;
  ads1220_init(&meteo_stick.diff_pressure, &(MS_SPI_DEV), MS_DIFF_PRESSURE_SLAVE_IDX);

  // Init temperature
  meteo_stick.temperature.config.mux = ADS1220_MUX_AIN0_AIN1;
  meteo_stick.temperature.config.gain = ADS1220_GAIN_4;
  meteo_stick.temperature.config.pga_bypass = TRUE;
  meteo_stick.temperature.config.rate = ADS1220_RATE_45_HZ;
  meteo_stick.temperature.config.conv = ADS1220_CONTINIOUS_CONVERSION;
  meteo_stick.temperature.config.vref = ADS1220_VREF_EXTERNAL_REF;
  meteo_stick.temperature.config.idac = ADS1220_IDAC_250_uA;
  meteo_stick.temperature.config.i1mux = ADS1220_IMUX_OFF;
  meteo_stick.temperature.config.i2mux = ADS1220_IMUX_A0_RP1;
  ads1220_init(&meteo_stick.temperature, &(MS_SPI_DEV), MS_TEMPERATURE_SLAVE_IDX);

  // Init humidity
  meteo_stick.humidity_period = 0;

#if LOG_MS
  log_ptu_started = FALSE;
#endif
}

/** Periodic function
 */
void meteo_stick_periodic(void)
{
  // Read ADC
  ads1220_periodic(&meteo_stick.pressure);
  ads1220_periodic(&meteo_stick.diff_pressure);
  ads1220_periodic(&meteo_stick.temperature);
  // Read PWM
  meteo_stick.humidity_period = pwm_input_period_tics[MS_HUMIDITY_PWM_INPUT];

  // Log data
#if LOG_MS
  if (pprzLogFile.fs != NULL) {
    if (!log_ptu_started) {
      sdLogWriteLog(&pprzLogFile,
                    "P(adc) T(adc) H(ticks) P_diff(adc) GPS_fix TOW(ms) Week Lat(1e7rad) Lon(1e7rad) HMSL(mm) gpseed(cm/s) course(1e7rad) climb(cm/s)\n");
      log_ptu_started = TRUE;
    } else {
      sdLogWriteLog(&pprzLogFile, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                    meteo_stick.pressure.data, meteo_stick.temperature.data,
                    meteo_stick.humidity_period, meteo_stick.diff_pressure.data,
                    gps.fix, gps.tow, gps.week,
                    gps.lla_pos.lat, gps.lla_pos.lon, gps.hmsl,
                    gps.gspeed, gps.course, -gps.ned_vel.z);
    }
  }
#endif

  // Send data
#if SEND_MS
  meteo_stick_send_data();
#endif
}

/** Event function
 */
void meteo_stick_event(void)
{
  ads1220_event(&meteo_stick.pressure);
  ads1220_event(&meteo_stick.diff_pressure);
  ads1220_event(&meteo_stick.temperature);

  // send absolute pressure data over ABI as soon as available
  if (meteo_stick.pressure.data_available) {
    float abs = MS_PRESSURE_SCALE * (float)((int32_t)meteo_stick.pressure.data - MS_PRESSURE_OFFSET);
    AbiSendMsgBARO_ABS(METEO_STICK_SENDER_ID, abs);
    meteo_stick.pressure.data_available = FALSE;
  }

  // send differential pressure data over ABI as soon as available
  if (meteo_stick.diff_pressure.data_available) {
    float diff = MS_DIFF_PRESSURE_SCALE * (float)((int32_t)meteo_stick.diff_pressure.data - MS_DIFF_PRESSURE_OFFSET);
    AbiSendMsgBARO_DIFF(METEO_STICK_SENDER_ID, diff);
    meteo_stick.diff_pressure.data_available = FALSE;
  }
}


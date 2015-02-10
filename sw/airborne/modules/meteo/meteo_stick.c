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
 *  Only used if calibration from EEPROM is not used/available
 */
#ifndef MS_PRESSURE_OFFSET
#define MS_PRESSURE_OFFSET 0.0f
#endif
#ifndef MS_PRESSURE_SCALE
#define MS_PRESSURE_SCALE 1.0f
#endif
#ifndef MS_TEMPERATURE_OFFSET
#define MS_TEMPERATURE_OFFSET 0.0f
#endif
#ifndef MS_TEMPERATURE_SCALE
#define MS_TEMPERATURE_SCALE 1.0f
#endif
#ifndef MS_HUMIDTY_OFFSET
#define MS_HUMIDTY_OFFSET 0.0f
#endif
#ifndef MS_HUMIDTY_SCALE
#define MS_HUMIDTY_SCALE 1.0f
#endif
#ifndef MS_DIFF_PRESSURE_OFFSET
#define MS_DIFF_PRESSURE_OFFSET 0.0f
#endif
#ifndef MS_DIFF_PRESSURE_SCALE
#define MS_DIFF_PRESSURE_SCALE 1.0f
#endif

// Test if EEPROM slave index is configured
// if not, don't use EEPROM
#ifndef MS_EEPROM_SLAVE_IDX
#undef USE_MS_EEPROM
#endif

/** General structure */
struct MeteoStick meteo_stick;


/** Prescaling of data according to datasheets
 */

static const float maxAdc = 8388608.0f; // 2 ** 23

static float get_pressure(uint32_t raw)
{
  const float uncal_abs = ((raw / maxAdc) + 0.095f) / 0.0009f;

#if USE_MS_EEPROM
  return mtostk_get_calibrated_value(&meteo_stick.calib, MTOSTK_ABS_PRESS, uncal_abs, meteo_stick.current_temperature);
#else
  return (MS_PRESSURE_OFFSET * uncal_abs) + MS_PRESSURE_OFFSET;
#endif
}

static float get_temp(uint32_t raw)
{
  const float coeff_A = 3.9083e-3f;
  const float coeff_B = -5.775e-7f;
  const float R0 = 1000.0f;
  const float gain_factor = Ads1220GainTable[ADS1220_GAIN_4];
  const float pga_factor = 1.0f;
  const float Rref = 6.8e3f;
  const float Rt = (raw * Rref) / (gain_factor * pga_factor * maxAdc);
  const float uncal_temp = ((-R0 * coeff_A) + (sqrtf(powf((R0 * coeff_A), 2.0f) +
                            (4.0f * R0 * coeff_B * (Rt - R0))))) / (2.0f * R0 * coeff_B);

#if USE_MS_EEPROM
  return mtostk_get_calibrated_value(&meteo_stick.calib, MTOSTK_TEMP, uncal_temp, 0.);
#else
  return (MS_TEMPERATURE_SCALE * uncal_temp) + MS_TEMPERATURE_OFFSET;
#endif
}

static float pitot_offset;
static int pitot_counter;

static float get_diff(uint32_t raw)
{
  const float gain_factor = Ads1220GainTable[ADS1220_GAIN_2];
  const uint32_t raw_diff = raw > pitot_offset ? raw - pitot_offset : 0;
  const float uncal_diff = ((raw_diff / maxAdc) * 5000.f / gain_factor);

#if USE_MS_EEPROM
  return mtostk_get_calibrated_value(&meteo_stick.calib, MTOSTK_DIF_PRESS, uncal_diff, meteo_stick.current_temperature);
#else
  return (MS_DIFF_PRESSURE_SCALE * uncal_diff) + MS_DIFF_PRESSURE_OFFSET;
#endif
}

static float get_pitot(uint32_t raw)
{
  return sqrtf((2.0f * get_diff(raw)) / 1.293f);
}

static float get_humidity(uint32_t raw)
{
  const float icu_freq = 42e6f; // Freq
  const float Ra = 390e3f;
  const float Rb = 680e3f;
  const float S1 = 0.3e-12f; // pico farad by % of relative humidity
  const float calib_raw_period = 17800.f;
  const float calib_humidity = 45.f;
  const float k = 2.f * logf(2.f) * (Ra + Rb + Rb);
  const float uncal_hum = calib_humidity + ((raw - calib_raw_period) / (k * icu_freq * S1));

#if USE_MS_EEPROM
  return mtostk_get_calibrated_value(&meteo_stick.calib, MTOSTK_HUMIDITY, uncal_hum, meteo_stick.current_temperature);
#else
  return (MS_HUMIDTY_SCALE * uncal_hum) + MS_HUMIDTY_OFFSET;
#endif
}


/** Includes to log on SD card
 *
 * TRUE by default
 */
#ifndef LOG_MS
#define LOG_MS TRUE
#endif

#if LOG_MS
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
bool_t log_ptu_started;
#endif

/* Includes and function to send over telemetry
 *
 * TRUE by default
 */
#ifndef SEND_MS
#define SEND_MS TRUE
#endif

#if SEND_MS
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps.h"

#define MS_DATA_SIZE 4

static inline void meteo_stick_send_data(void)
{
  float ptu_data[MS_DATA_SIZE];
  ptu_data[0] = meteo_stick.current_pressure;
  ptu_data[1] = meteo_stick.current_temperature;
  ptu_data[2] = meteo_stick.current_humidity;
  ptu_data[3] = meteo_stick.current_airspeed;
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

  // Initial temperature (ISA at sea level)
  meteo_stick.current_temperature = 15.0f;

#if USE_MS_EEPROM
  // Set number of calibration to 0 for all sensors
  int i;
  for (i = 0; i < MTOSTK_NUM_SENSORS; i++) {
    meteo_stick.calib.params[i].num_temp = 0;
  }
  // Init eeprom
  eeprom25AA256_init(&meteo_stick.eeprom, &(MS_SPI_DEV), MS_EEPROM_SLAVE_IDX);
#endif

  // Number of measurements before setting pitor offset
  pitot_counter = 10;

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
  meteo_stick.current_humidity = get_humidity(meteo_stick.humidity_period);

#if USE_MS_EEPROM
  if (meteo_stick.eeprom.data_available) {
    // Extract calibration data
    if (!mtostk_populate_cal_from_buffer(&meteo_stick.calib, (uint8_t *)(meteo_stick.eeprom.rx_buf + 3))) {
      // Extraction failed
      // Force number of calibration to 0 for all sensors
      int i;
      for (i = 0; i < MTOSTK_NUM_SENSORS; i++) {
        meteo_stick.calib.params[i].num_temp = 0;
      }
    }
  } else if (meteo_stick.eeprom.spi_trans.status == SPITransDone) {
    // Load reading request (reading 1Kb from address 0x0)
    eeprom25AA256_read(&meteo_stick.eeprom, 0x0, 1024);
  }
#endif

  // Log data
#if LOG_MS
  if (pprzLogFile.fs != NULL) {
    if (!log_ptu_started) {
#if USE_MS_EEPROM
      if (meteo_stick.eeprom.data_available) {
        // Print calibration data in the log header
        sdLogWriteLog(&pprzLogFile, "# Calibration data (UUID: %s)\n#\n", meteo_stick.calib.uuid);
        int i, j, k;
        for (i = 0; i < MTOSTK_NUM_SENSORS; i++) {
          sdLogWriteLog(&pprzLogFile, "# Sensor: %d, time: %d, num_temp: %d, num_coeff: %d\n", i,
                        meteo_stick.calib.params[i].timestamp,
                        meteo_stick.calib.params[i].num_temp,
                        meteo_stick.calib.params[i].num_coeff);
          if (meteo_stick.calib.params[i].timestamp == 0) {
            continue; // No calibration
          }
          for (j = 0; j < meteo_stick.calib.params[i].num_temp; j++) {
            sdLogWriteLog(&pprzLogFile, "#  Reference temp: %.2f\n", meteo_stick.calib.params[i].temps[j]);
            sdLogWriteLog(&pprzLogFile, "#  Coeffs:");
            for (k = 0; k < meteo_stick.calib.params[i].num_coeff; k++) {
              sdLogWriteLog(&pprzLogFile, " %.5f", meteo_stick.calib.params[i].coeffs[j][k]);
            }
            sdLogWriteLog(&pprzLogFile, "\n");
          }
        }
        sdLogWriteLog(&pprzLogFile, "#\n");
        sdLogWriteLog(&pprzLogFile,
                      "P(adc) T(adc) H(ticks) P_diff(adc) P(hPa) T(C) H(\%) CAS(m/s) FIX TOW(ms) WEEK Lat(1e7rad) Lon(1e7rad) HMSL(mm) GS(cm/s) course(1e7rad) VZ(cm/s)\n");
        log_ptu_started = TRUE;
      }
#else
      sdLogWriteLog(&pprzLogFile,
                    "P(adc) T(adc) H(ticks) P_diff(adc) P(hPa) T(C) H(\%) CAS(m/s) FIX TOW(ms) WEEK Lat(1e7rad) Lon(1e7rad) HMSL(mm) GS(cm/s) course(1e7rad) VZ(cm/s)\n");
      log_ptu_started = TRUE;
#endif
    } else {
      sdLogWriteLog(&pprzLogFile, "%d %d %d %d %.2f %.2f %.2f %.2f %d %d %d %d %d %d %d %d %d\n",
                    meteo_stick.pressure.data, meteo_stick.temperature.data,
                    meteo_stick.humidity_period, meteo_stick.diff_pressure.data,
                    meteo_stick.current_pressure, meteo_stick.current_temperature,
                    meteo_stick.current_humidity, meteo_stick.current_airspeed,
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

  // send temperature data over ABI as soon as available
  if (meteo_stick.temperature.data_available) {
    meteo_stick.current_temperature = get_temp(meteo_stick.temperature.data);
    AbiSendMsgTEMPERATURE(METEO_STICK_SENDER_ID, meteo_stick.current_temperature);
    meteo_stick.temperature.data_available = FALSE;
  }

  // send absolute pressure data over ABI as soon as available
  if (meteo_stick.pressure.data_available) {
    meteo_stick.current_pressure = get_pressure(meteo_stick.pressure.data);
    AbiSendMsgBARO_ABS(METEO_STICK_SENDER_ID, meteo_stick.current_pressure);
    meteo_stick.pressure.data_available = FALSE;
  }

  // send differential pressure data over ABI as soon as available
  if (meteo_stick.diff_pressure.data_available) {
    if (pitot_counter > 0) {
      pitot_counter--;
      if (pitot_counter == 0) {
        pitot_offset = meteo_stick.diff_pressure.data;
      }
    }
    float diff = get_diff(meteo_stick.diff_pressure.data);
    AbiSendMsgBARO_DIFF(METEO_STICK_SENDER_ID, diff);
    meteo_stick.current_airspeed = get_pitot(meteo_stick.diff_pressure.data);
    meteo_stick.diff_pressure.data_available = FALSE;
  }

#if USE_MS_EEPROM
  eeprom25AA256_event(&meteo_stick.eeprom);
#endif
}


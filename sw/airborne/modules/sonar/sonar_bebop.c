/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/sonar/sonar_bebop.c
 *  @brief Parrot Bebop Sonar driver
 *
 *  This file contains the ADC driver for the sonar on the Parrot Bebop
 *  and Bebop 2 quadrotors. The sonar is both transmitter and receiver.
 *  An SPI interface is used to communicate with the 12 bit ADC which
 *  operates at 160kHz. The required waveform of the sonar is sent during
 *  the SPI transaction, with the returned data the recorded signal from
 *  the sonar. Two waveforms have been implemented dependent on the
 *  operating altitude of the Bebop.
 */

#include "sonar_bebop.h"
#include "generated/airframe.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/spi.h"
#include "subsystems/abi.h"
#include <pthread.h>
#include "subsystems/datalink/downlink.h"

#include "filters/median_filter.h"

struct MedianFilterFloat sonar_filt;

#ifdef SITL
#include "state.h"
#endif

/** SONAR_BEBOP_INX_DIFF_TO_DIST conversion from index difference to distance based on time of flight
 * ADC speed = 160kHz
 * speed of sound = 340m/s   */
#define SONAR_BEBOP_INX_DIFF_TO_DIST 340./(2.*160000.)

/** SONAR_BEBOP_ADC_MAX_VALUE maximum ADC output (12 bit ADC) */
#define SONAR_BEBOP_ADC_MAX_VALUE 4095

/** SONAR_BEBOP_ADC_BUFFER_SIZE ADC buffer size */
#define SONAR_BEBOP_ADC_BUFFER_SIZE 8192

/** mode holds the current sonar mode
 * mode = 0 used at high altitude, uses 16 wave patterns
 * mode = 1 used at low altitude, uses 4 wave patterns
 */
static uint8_t mode;

/** SONAR_BEBOP_TRANSITION_HIGH_TO_LOW below this altitude we should use mode 0 */
#define SONAR_BEBOP_TRANSITION_HIGH_TO_LOW 0.8

/** SONAR_BEBOP_TRANSITION_LOW_TO_HIGH above this altitude we should use mode 1 */
#define SONAR_BEBOP_TRANSITION_LOW_TO_HIGH 1.2

/** SONAR_BEBOP_TRANSITION_COUNT number of samples before switching mode */
#define SONAR_BEBOP_TRANSITION_COUNT 7

static uint8_t pulse_transition_counter;

/** SONAR_BEBOP_PEAK_THRESHOLD minimum samples from broadcast stop */
#define SONAR_BEBOP_PEAK_THRESHOLD 250

/** sonar_bebop_spi_d the waveforms emitted by the sonar
 * waveform 0 is long pulse used at high altitude
 * waveform 1 is shorter pulse used at low altitude
 */
static uint8_t sonar_bebop_spi_d[2][16] = {{ 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                                           { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00 }};

struct SonarBebop sonar_bebop;

static struct spi_transaction sonar_bebop_spi_t;
void *sonar_bebop_read(void *data);

void sonar_bebop_init(void)
{
  mode = 0; // default mode is low altitude
  pulse_transition_counter = 0;

  sonar_bebop.meas = 0;
  sonar_bebop.offset = 0;

  sonar_bebop_spi_t.status        = SPITransDone;
  sonar_bebop_spi_t.select        = SPISelectUnselect;
  sonar_bebop_spi_t.dss           = SPIDss8bit;
  sonar_bebop_spi_t.output_buf    = sonar_bebop_spi_d[mode];
  sonar_bebop_spi_t.output_length = 16;
  sonar_bebop_spi_t.input_buf     = NULL;
  sonar_bebop_spi_t.input_length  = 0;

#if USE_SONAR
  pthread_t tid;
  pthread_create(&tid, NULL, sonar_bebop_read, NULL);
  pthread_setname_np(tid, "pprz_sonar_thread");
#endif

  init_median_filter_f(&sonar_filt, 3);
}

uint16_t adc_buffer[SONAR_BEBOP_ADC_BUFFER_SIZE];
/** sonar_bebop_read
 * Read ADC value to update sonar measurement
 */
void *sonar_bebop_read(void *data __attribute__((unused)))
{
  while (true) {
#ifndef SITL
    uint16_t i;

    /* Start ADC and send sonar output */
    adc_enable(&adc0, 1);
    sonar_bebop_spi_t.status = SPITransDone;
    sonar_bebop_spi_t.output_buf    = sonar_bebop_spi_d[mode];
    spi_submit(&spi0, &sonar_bebop_spi_t);
    while (sonar_bebop_spi_t.status != SPITransSuccess);
    adc_read(&adc0, adc_buffer, SONAR_BEBOP_ADC_BUFFER_SIZE);
    adc_enable(&adc0, 0);

    /* Find the peaks */
    uint16_t start_send = 0;
    uint16_t stop_send = 0;
    uint16_t first_peak = 0;
    uint16_t lowest_value = SONAR_BEBOP_ADC_MAX_VALUE;
    uint16_t peak_value = 0;

    for (i = 0; i < SONAR_BEBOP_ADC_BUFFER_SIZE; i++) {
      uint16_t adc_val = adc_buffer[i] >> 4;
      if (start_send == 0 && adc_val == SONAR_BEBOP_ADC_MAX_VALUE) {
        start_send = i;
      } else if (start_send != 0 && stop_send == 0 && adc_val != SONAR_BEBOP_ADC_MAX_VALUE) {
        stop_send = i - 1;
        i += SONAR_BEBOP_PEAK_THRESHOLD;
        continue;
      }

      if (start_send != 0 && stop_send != 0 && first_peak == 0 && adc_val < lowest_value) {
        lowest_value = adc_val; // find trough from initial broadcast signal
      } else if (start_send != 0 && stop_send != 0 && adc_val > lowest_value + 100 && adc_val > peak_value) {
        first_peak = i;
        peak_value = adc_val;
      }
    }

    /* Calculate the distance from the peeks */
    uint16_t diff = stop_send - start_send;
    float peak_distance;
    if (diff > 250) {
      peak_distance = 0;
    } else {
      peak_distance = first_peak - (stop_send - diff / 2);
    }

    sonar_bebop.distance = update_median_filter_f(&sonar_filt, peak_distance * SONAR_BEBOP_INX_DIFF_TO_DIST);

    // set sonar pulse mode for next pulse based on altitude
    if (mode == 0 && sonar_bebop.distance > SONAR_BEBOP_TRANSITION_LOW_TO_HIGH) {
      if (++pulse_transition_counter > SONAR_BEBOP_TRANSITION_COUNT) {
        mode = 1;
        pulse_transition_counter = 0;
      }
    } else if (mode == 1 && sonar_bebop.distance < SONAR_BEBOP_TRANSITION_HIGH_TO_LOW) {
      if (++pulse_transition_counter > SONAR_BEBOP_TRANSITION_COUNT) {
        mode = 0;
        pulse_transition_counter = 0;
      }
    } else {
      pulse_transition_counter = 0;
    }

#else // SITL
    sonar_bebop.distance = stateGetPositionEnu_f()->z;
    Bound(sonar_bebop.distance, 0.1f, 7.0f);
    uint16_t peak_distance = 1;
#endif // SITL

    if (peak_distance > 0) {
      // Send ABI message
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, sonar_bebop.distance);
#ifdef SENSOR_SYNC_SEND_SONAR
      // Send Telemetry report
      DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_bebop.meas, &sonar_bebop.distance);
#endif
    }
  }
  usleep(10000); //100Hz
  return NULL;
}

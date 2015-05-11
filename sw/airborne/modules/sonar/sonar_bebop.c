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
 */

#include "sonar_bebop.h"
#include "generated/airframe.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/spi.h"
#include "subsystems/abi.h"
#include <pthread.h>
#include "subsystems/datalink/downlink.h"

#ifdef SITL
#include "state.h"
#endif


struct SonarBebop sonar_bebop;
static uint8_t sonar_bebop_spi_d[16] = { 0xF0,0xF0,0xF0,0xF0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
static struct spi_transaction sonar_bebop_spi_t;
static pthread_t sonar_bebop_thread;
static void *sonar_bebop_read(void *data);

void sonar_bebop_init(void)
{
  sonar_bebop.meas = 0;
  sonar_bebop.offset = 0;

  sonar_bebop_spi_t.status        = SPITransDone;
  sonar_bebop_spi_t.select        = SPISelectUnselect;
  sonar_bebop_spi_t.dss           = SPIDss8bit;
  sonar_bebop_spi_t.output_buf    = sonar_bebop_spi_d;
  sonar_bebop_spi_t.output_length = 16;
  sonar_bebop_spi_t.input_buf     = NULL;
  sonar_bebop_spi_t.input_length  = 0;

  int rc = pthread_create(&sonar_bebop_thread, NULL, sonar_bebop_read, NULL);
  if (rc < 0) {
    return;
  }
}

/**
 * Read ADC value to update sonar measurement
 */
static void *sonar_bebop_read(void *data __attribute__((unused)))
{
  while(true) {

#ifndef SITL
    uint16_t i;
    uint16_t adc_buffer[8192];


    /* Start ADC and send sonar output */
    adc_enable(&adc0, 1);
    sonar_bebop_spi_t.status = SPITransDone;
    spi_submit(&spi0, &sonar_bebop_spi_t);
    while(sonar_bebop_spi_t.status != SPITransSuccess);
    adc_read(&adc0, adc_buffer, 8192);
    adc_enable(&adc0, 0);

    /* Find the peeks */
    uint16_t start_send = 0;
    uint16_t stop_send = 0;
    uint16_t first_peek = 0;
    uint16_t lowest_value = 4095;
    for(i = 0; i < 8192; i++){
      uint16_t adc_val = adc_buffer[i]>>4;
      if(start_send == 0 && adc_val == 4095)
        start_send = i;
      else if(start_send != 0 && stop_send == 0 && adc_val != 4095)
      {
        stop_send = i-1;
        i += 300;
        continue;
      }

      if(start_send != 0 && stop_send != 0 && first_peek == 0 && adc_val < lowest_value)
        lowest_value = adc_val;
      else if (start_send != 0 && stop_send != 0 && adc_val > lowest_value + 100) {
        first_peek = i;
        lowest_value = adc_val - 100;
      }
      else if(start_send != 0 && stop_send != 0 && first_peek != 0 && adc_val+100 < (adc_buffer[first_peek]>>4)) {
        break;
      }
    }

    /* Calculate the distance from the peeks */
    uint16_t diff = stop_send - start_send;
    int16_t peek_distance = first_peek - (stop_send - diff/2);
    if(first_peek <= stop_send || diff > 250)
      peek_distance = 0;

    sonar_bebop.distance = peek_distance / 1000.0;
#else // SITL
    sonar_bebop.distance = stateGetPositionEnu_f()->z;
    Bound(sonar_bebop.distance, 0.1f, 7.0f);
    uint16_t peek_distance = 1;
#endif // SITL

    usleep(10000);

    if(peek_distance > 0)
    {
      // Send ABI message
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, sonar_bebop.distance);

#ifdef SENSOR_SYNC_SEND_SONAR
      // Send Telemetry report
      DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_bebop.meas, &sonar_bebop.distance);
#endif
    }
  }

  return NULL;
}


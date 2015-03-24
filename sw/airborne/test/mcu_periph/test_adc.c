/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/adc.h"
#include "subsystems/datalink/downlink.h"

int main_periodic(void);
static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

#define NB_ADC 8
#define ADC_NB_SAMPLES 16

static struct adc_buf buf_adc[NB_ADC];

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / 100), NULL);
  downlink_init();
  adc_init();

#ifdef ADC_0
  adc_buf_channel(ADC_0, &buf_adc[0], ADC_NB_SAMPLES);
#endif
#ifdef ADC_1
  adc_buf_channel(ADC_1, &buf_adc[1], ADC_NB_SAMPLES);
#endif
#ifdef ADC_2
  adc_buf_channel(ADC_2, &buf_adc[2], ADC_NB_SAMPLES);
#endif
#ifdef ADC_3
  adc_buf_channel(ADC_3, &buf_adc[3], ADC_NB_SAMPLES);
#endif
#ifdef ADC_4
  adc_buf_channel(ADC_4, &buf_adc[4], ADC_NB_SAMPLES);
#endif
#ifdef ADC_5
  adc_buf_channel(ADC_5, &buf_adc[5], ADC_NB_SAMPLES);
#endif
#ifdef ADC_6
  adc_buf_channel(ADC_6, &buf_adc[6], ADC_NB_SAMPLES);
#endif
#ifdef ADC_7
  adc_buf_channel(ADC_7, &buf_adc[7], ADC_NB_SAMPLES);
#endif

  mcu_int_enable();
}

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
  RunOnceEvery(100, {uint32_t sec = sys_time.nb_sec; DOWNLINK_SEND_TIME(DefaultChannel, DefaultDevice, &sec);});
  LED_PERIODIC();

  uint16_t values[NB_ADC];
  uint8_t i;
  for (i = 0; i < NB_ADC; i++) {
    values[i] = buf_adc[i].sum / ADC_NB_SAMPLES;
  }

  uint8_t id = 42;
  DOWNLINK_SEND_ADC(DefaultChannel, DefaultDevice, &id, NB_ADC, values);
}

static inline void main_event_task(void)
{
  mcu_event();
}

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
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

int main_periodic(void);
static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static struct adc_buf adc0_buf;
static struct adc_buf adc1_buf;
static struct adc_buf adc2_buf;
static struct adc_buf adc3_buf;
static struct adc_buf vsupply_buf;

#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) DefaultVoltageOfAdc(adc)
#endif

static inline void main_init( void ) {
    mcu_init();
    sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
    adc_init();
    adc_buf_channel(0, &adc0_buf, 8);
    adc_buf_channel(1, &adc1_buf, 3);
    adc_buf_channel(2, &adc2_buf, 3);
    adc_buf_channel(3, &adc3_buf, 3);
    adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_buf, DEFAULT_AV_NB_SAMPLE);
}

int main( void ) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
  RunOnceEvery(100, {DOWNLINK_SEND_TIME(DefaultChannel, DefaultDevice, &sys_time.nb_sec);});
  LED_PERIODIC();

  uint16_t v1 = 10 * VoltageOfAdc((vsupply_buf.sum/vsupply_buf.av_nb_sample));
  uint16_t v2 = 10 * VoltageOfAdc((vsupply_buf.values[0]));
  RunOnceEvery(50, {DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, DefaultDevice, &v1, &v2)});
}

static inline void main_event_task( void ) {

}

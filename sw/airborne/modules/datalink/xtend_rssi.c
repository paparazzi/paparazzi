/*
 * $Id$
 *
 * Copyright (C) 2010 2011 Stephen Dwyer, based on windturbine by Martin Mueller
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

/** \file xtend_rssi.c
 *
 *   This measures the rssi pwm signal from a Digi XTend radio modem
 *   and sends a message with the info.
 */


#include "datalink/xtend_rssi.h"
#include "core/pwm_measure.h"
#include "sys_time.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

//from Digi XTend manual
#define XTEND_RSSI_PWM_PERIOD_ST (uint32_t)(8320 * (PCLK/1000000) / T0_PCLK_DIV + 0.5) //rssi pwm period in sys tics

//architecture dependent pin assignment, update for new architectures? move to arch files?
#ifdef PWM_MEAS_ON_TWOG_ADC5
#define PWM_MEAS_XTEND_RSSI_INDEX 0
#define USE_PWM_INPUT1
#elif PWM_MEAS_ON_TWOG_ADC4
#define PWM_MEAS_XTEND_RSSI_INDEX 1
#define USE_PWM_INPUT2
#elif PWM_MEAS_ON_TWOG_ADC6
#define PWM_MEAS_XTEND_RSSI_INDEX 3
#define USE_PWM_INPUT4
#else
#error "Invalid or missing pwm measurement input pin defined for xtend_rssi module"
#endif

void xtend_rssi_periodic( void ) {

/* get the last duty if valid then reset valid flag (this says if we got another pulse since the last one)
   calculate the % and dB from the duty using datasheet specs
   send the %, dB, datalink time
*/

  uint32_t duty_tics = pwm_meas_duty_tics[PWM_MEAS_XTEND_RSSI_INDEX];
  uint8_t duty_percent = 0;
  uint8_t rssi_dB_fade_margin = 0; //shows dB fade margin above rated minimum sensitivity

  if (pwm_meas_duty_valid[PWM_MEAS_XTEND_RSSI_INDEX])
  {
      duty_percent = (duty_tics * 100) / XTEND_RSSI_PWM_PERIOD_ST;
      rssi_dB_fade_margin = (2 * duty_percent + 10) / 3; //not sure if this is right, datasheet isn't very informative
      pwm_meas_duty_valid[PWM_MEAS_XTEND_RSSI_INDEX] = FALSE;
  }
  DOWNLINK_SEND_XTEND_RSSI(DefaultChannel,
            &datalink_time,
            &rssi_dB_fade_margin,
            &duty_percent );
}

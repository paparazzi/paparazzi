/*
 * Copyright (C) 2011 The Paparazzi Team
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


#include "modules/datalink/xtend_rssi.h"
#include "mcu_periph/pwm_input.h"
#include "mcu_periph/sys_time.h"


#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

//from Digi XTend manual
#define XTEND_RSSI_PWM_PERIOD_USEC 8320 //rssi pwm period () in sys tics

#define XTEND_RSSI_PWM_ARRAY_INDEX (XTEND_RSSI_PWM_INPUT_CHANNEL - 1)

void xtend_rssi_periodic(void)
{

  /* get the last duty if valid then reset valid flag (this says if we got another pulse since the last one)
     calculate the % and dB from the duty using datasheet specs
     send the %, dB, datalink time
  */

  uint32_t duty_tics = pwm_input_duty_tics[XTEND_RSSI_PWM_ARRAY_INDEX];
  uint8_t duty_percent = 0;
  uint8_t rssi_dB_fade_margin = 0; //shows dB fade margin above rated minimum sensitivity

  if (pwm_input_duty_valid[XTEND_RSSI_PWM_ARRAY_INDEX]) {
    duty_percent = (duty_tics * 100) / cpu_ticks_of_usec(XTEND_RSSI_PWM_PERIOD_USEC);
    rssi_dB_fade_margin = (2 * duty_percent + 10) / 3; //not sure if this is right, datasheet isn't very informative
    pwm_input_duty_valid[XTEND_RSSI_PWM_ARRAY_INDEX] = FALSE;
  }
  DOWNLINK_SEND_XTEND_RSSI(DefaultChannel, DefaultDevice,
                           &datalink_time,
                           &rssi_dB_fade_margin,
                           &duty_percent);
}

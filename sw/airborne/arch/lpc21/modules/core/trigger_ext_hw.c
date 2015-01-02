/*
 * Copyright (C) 2010 Martin Mueller
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


#include "core/trigger_ext_hw.h"
#include "std.h"
#include "mcu_periph/sys_time.h"
#include "LPC21xx.h"
#include BOARD_CONFIG

void TRIG_ISR()
{
  static uint32_t last;
  uint32_t delta_t0_temp;
  trigger_t0 = TRIGGER_CR;
  delta_t0_temp = trigger_t0 - last;
  if (msec_of_cpu_ticks(delta_t0_temp) > 10) {
    trigger_delta_t0 = delta_t0_temp;
    last = trigger_t0;
    trigger_ext_valid = TRUE;
  }
}

void trigger_ext_init(void)
{
  /* select pin for capture */
  TRIG_EXT_PINSEL |= TRIG_EXT_PINSEL_VAL << TRIG_EXT_PINSEL_BIT;
  /* enable capture 0.2 on falling or rising edge + trigger interrupt */
#if defined TRIG_EXT_PULSE_TYPE && TRIG_EXT_PULSE_TYPE == TRIG_EXT_PULSE_TYPE_RISING
  T0CCR |= TRIGGER_CRR | TRIGGER_CRI;
#elif defined TRIG_EXT_PULSE_TYPE && TRIG_EXT_PULSE_TYPE == TRIG_EXT_PULSE_TYPE_FALLING
  T0CCR |= TRIGGER_CRF | TRIGGER_CRI;
#else
#error "trig_ext_hw.h: Unknown PULSE_TYPE"
#endif
  trigger_ext_valid = FALSE;
}


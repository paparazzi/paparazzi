/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 */
/**
 * @file arch/chibios/subsystems/radio_control/ppm_arch.c
 * PPM interface between ChibiOS and Paparazzi
 *
 * Input capture configuration has to be defined in board.h
 */
#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"

uint8_t  ppm_cur_pulse;
uint32_t ppm_last_pulse_time;
bool   ppm_data_valid;
static uint32_t timer_rollover_cnt;

#ifndef PPM_TIMER_FREQUENCY
#error "Undefined PPM_TIMER_FREQUENCY"
#endif

#ifndef PPM_CHANNEL
#error "PPM channel undefined"
#endif


/**
 * PPM Pulse period callback
 */
static void icuperiodcb(ICUDriver *icup)
{
  ppm_decode_frame_width(icuGetPeriodX(icup));
}

/**
 * PPM Overflow callback
 */
static void icuoverflowcb(ICUDriver *icup)
{
  (void)icup;
  // TODO do something with this ?
}

/**
 * ICU timer configuration
 *
 * There appears to be no difference between
 * ICU_INPUT_ACTIVE_HIGH and ICU_INPUT_ACTIVE_LOW,
 * it works in both cases. Further investigation needed.
 */
static ICUConfig ppm_icucfg = {
#if defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_POSITIVE
  ICU_INPUT_ACTIVE_HIGH,
#elif defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_NEGATIVE
  ICU_INPUT_ACTIVE_LOW,
#else
#error "Unknown PPM_PULSE_TYPE"
#endif
  PPM_TIMER_FREQUENCY,
  NULL,
  icuperiodcb,
  icuoverflowcb,
  PPM_CHANNEL,
  0,
  0xFFFFFFFFU
};

void ppm_arch_init(void)
{
  ppm_last_pulse_time = 0;
  ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;
  timer_rollover_cnt = 0;

  icuStart(&PPM_TIMER, &ppm_icucfg);
  icuStartCapture(&PPM_TIMER);
  icuEnableNotifications(&PPM_TIMER);
}

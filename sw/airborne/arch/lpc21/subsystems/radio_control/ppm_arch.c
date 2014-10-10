/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file arch/lpc21/subsystems/radio_control/ppm_arch.c
 *
 * LPC21xx specific implementation for PPM radio control.
 *
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"


void ppm_arch_init(void)
{
  /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on falling or rising edge + trigger interrupt */
#if defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_POSITIVE
  T0CCR |= PPM_CCR_CRR | PPM_CCR_CRI;
#elif defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_NEGATIVE
  T0CCR |= PPM_CCR_CRF | PPM_CCR_CRI;
#else
#error "Unknown PPM_PULSE_TYPE"
#endif

#ifdef USE_PPM_RSSI_GPIO
  /* select pin as GPIO (input) : should be default ?*/
  //PPM_RSSI_PINSEL |= PPM_RSSI_VAL << PPM_RSSI_PINSEL_BIT;
  //ClearBit(PPM_RSSI_IODIR, PPM_RSSI_BIT);
#endif
}

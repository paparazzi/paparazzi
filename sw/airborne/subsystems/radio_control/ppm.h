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
 */

#ifndef RC_PPM_H
#define RC_PPM_H

#include "std.h"

/* in case you want to override RADIO_CONTROL_NB_CHANNEL */
#include "generated/airframe.h"

/**
 * Architecture dependant code
 */
#include "subsystems/radio_control/ppm_arch.h"
/* must be implemented by arch dependant code */
extern void ppm_arch_init(void);

/**
 * Generated code holding the description of a given
 * transmitter
 */
#include "generated/radio.h"

/**
 * Default number of channels to actually use.
 */
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL RADIO_CTL_NB
#endif

#if RADIO_CONTROL_NB_CHANNEL > RADIO_CTL_NB
#error "RADIO_CONTROL_NB_CHANNEL mustn't be higher than number of channels in radio file."
#endif

/**
 *  ppm pulse type : futaba is falling edge clocked whereas JR is rising edge
 */
#define PPM_PULSE_TYPE_POSITIVE 0
#define PPM_PULSE_TYPE_NEGATIVE 1

extern uint16_t ppm_pulses[RADIO_CTL_NB];
extern volatile bool ppm_frame_available;

/**
 * RC event function with handler callback.
 * PPM frames are normalized using the IIR filter.
 */
extern void radio_control_impl_event(void (* _received_frame_handler)(void));

#define RadioControlEvent(_received_frame_handler) radio_control_impl_event(_received_frame_handler)


/**
 * Decode a PPM frame.
 * A valid ppm frame:
 * - synchro blank
 * - correct number of channels
 * - synchro blank
 */
extern void ppm_decode_frame(uint32_t ppm_time);

#endif /* RC_PPM_H */

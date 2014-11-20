/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include "led.h"
#include "generated/airframe.h"
#include "paparazzi.h"

/* underlying hardware, also include if RADIO_CONTROL is not defined for ap in dual mcu case */
#include RADIO_CONTROL_TYPE_H

/* RADIO_CONTROL_NB_CHANNEL needs to be defined to suitable default the implementation.
 * If not all available channels are needed, can be overridden in airframe file.
 */

#if defined RADIO_CONTROL
/* must be defined by underlying hardware */
extern void radio_control_impl_init(void);

/* timeouts - for now assumes 60Hz periodic */
#define RC_AVG_PERIOD 8  /* TODO remove if IIR filter is used */
#define RC_LOST_TIME 30  /* 500ms with a 60Hz timer */
#define RC_REALLY_LOST_TIME 60 /* ~1s */
/* Number of valid frames before going back to RC OK */
#define RC_OK_CPT 15

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2

struct RadioControl {
  uint8_t status;
  uint8_t time_since_last_frame;
  uint8_t radio_ok_cpt;
  uint8_t frame_rate;
  uint8_t frame_cpt;
  pprz_t  values[RADIO_CONTROL_NB_CHANNEL];
};

extern struct RadioControl radio_control;


extern void radio_control_init(void);

extern void radio_control_periodic_task(void);

// Event implemented in radio_control/*.h


#endif /* RADIO_CONTROL */

#endif /* RADIO_CONTROL_H */

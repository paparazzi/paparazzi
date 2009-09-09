/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ_RADIO_CONTROL_H
#define BOOZ_RADIO_CONTROL_H

#include "std.h"
#include "paparazzi.h"

/* underlying hardware */
#include RADIO_CONTROL_TYPE_H
/* must be defined by underlying hardware */
extern void radio_control_impl_init(void);

/* status */
#define RADIO_CONTROL_OK          0
#define RADIO_CONTROL_LOST        1
#define RADIO_CONTROL_REALLY_LOST 2

/* timeouts - for now assumes 60Hz periodic */
#define RADIO_CONTROL_LOST_TIME        30
#define RADIO_CONTROL_REALLY_LOST_TIME 60
/* number of valid frames before going back to OK */
#define RADIO_CONTROL_OK_CPT      15

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
extern void radio_control_periodic(void);


#endif /* BOOZ_RADIO_CONTROL_H */


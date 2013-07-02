/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file subsystems/radio_control/superbitrf.h
 * DSM2 and DSMX implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#ifndef RADIO_CONTROL_SUPERBITRF_H
#define RADIO_CONTROL_SUPERBITRF_H

#include "peripherals/cyrf6936.h"
#include "mcu_periph/gpio.h"

/* Theoretically you could have 14 channel over DSM2/DSMX */
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 14
#endif

/* The channel ordering is always the same for DSM2 and DSMX */
#define RADIO_THROTTLE   0
#define RADIO_ROLL       1
#define RADIO_PITCH      2
#define RADIO_YAW        3
#define RADIO_GEAR       4
#define RADIO_FLAP       5
#define RADIO_AUX1       5
#define RADIO_AUX2       6
#define RADIO_AUX3       7
#define RADIO_AUX4       8
#define RADIO_AUX5       9
#define RADIO_AUX6       10
#define RADIO_AUX7       11
#define RADIO_AUX8       12
#define RADIO_AUX9       13

/* Map the MODE default to the gear switch */
#ifndef RADIO_MODE
#define RADIO_MODE       RADIO_GEAR
#endif

/* The different statuses the superbitRF can be in */
enum SuperbitRFStatus {
  SUPERBITRF_UNINIT,                /**< The chip isn't initialized */
  SUPERBITRF_INIT_BINDING,          /**< The chip is initializing binding mode */
  SUPERBITRF_INIT_TRANSFER,         /**< The chip is initializing transfer mode */
  SUPERBITRF_BINDING,               /**< The chip is in binding mode */
  SUPERBITRF_TRANSFER,              /**< The chip is in transfer mode */
};

/* The superbitrf structure */
struct SuperbitRF {
  struct Cyrf6936 cyrf6936;                 /**< The cyrf chip used */
  volatile enum SuperbitRFStatus status;    /**< The status of the superbitRF */
};

/* The superbitrf functions and structures */
extern struct SuperbitRF superbitrf;
void superbitrf_event(void);

/* The radio control event handler */
#define RadioControlEvent(_received_frame_handler) {    \
  cyrf6936_event(&superbitrf.cyrf6936);                 \
  superbitrf_event();                                   \
}

#endif /* RADIO_CONTROL_SUPERBITRF_H */

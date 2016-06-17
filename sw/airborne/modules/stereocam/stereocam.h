/*
 * Copyright (C) 2015 Kirk Scheper
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

/** @file modules/stereocam/stereocam.h
 *  @brief interface to the TU Delft serial stereocam
 */

#ifndef STEREOCAM_H_
#define STEREOCAM_H_

#include <std.h>

typedef struct {
  uint8_t len;
  uint8_t *data;
  uint8_t fresh;
  uint8_t matrix_width;
  uint8_t matrix_height;
} uint8array;

extern uint8array stereocam_data;

extern void stereocam_disparity_to_meters(uint8_t *, float *, int);
extern void stereocam_start(void);
extern void stereocam_stop(void);
extern void stereocam_periodic(void);

#endif /* STEREOCAM_H_ */

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

#ifndef BOOZ2_HF_FLOAT_H
#define BOOZ2_HF_FLOAT_H

#include <inttypes.h>

#define B2_HFF_STATE_SIZE 3

#ifndef HFF_PRESCALER
#define HFF_PRESCALER 16
#endif

#define B2_HFF_UPDATE_SPEED
#define B2_HFF_UPDATE_POS

extern float b2_hff_x;
extern float b2_hff_xbias;
extern float b2_hff_xdot;
extern float b2_hff_xdotdot;

extern float b2_hff_y;
extern float b2_hff_ybias;
extern float b2_hff_ydot;
extern float b2_hff_ydotdot;

extern float b2_hff_xP[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE];
extern float b2_hff_yP[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE];

extern float b2_hff_x_meas;
extern float b2_hff_y_meas;

extern void b2_hff_init(float init_x, float init_xdot, float init_xbias, float init_y, float init_ydot, float init_ybias);
extern void b2_hff_propagate(float xaccel, float yaccel);
extern void b2_hff_update_gps(void);
extern void b2_hff_update_pos(float xpos, float ypos);
extern void b2_hff_update_v(float xspeed, float yspeed);

#ifdef GPS_LAG
extern void b2_hff_store_accel(float x, float y);
extern uint8_t lag_counter;
extern int8_t lag_counter_err;
extern int8_t save_counter;
#endif

#endif /* BOOZ2_HF_FLOAT_H */


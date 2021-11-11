/*
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

/**
 * @file modules/ins/hf_float.h
 *
 * Horizontal filter (x,y) to estimate position and velocity.
 *
 */

#ifndef HF_FLOAT_H
#define HF_FLOAT_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "generated/airframe.h"

// X = [ z zdot bias ]
#define HFF_STATE_SIZE 3

struct HfilterFloat {
  float x;
  float xdot;
  float xdotdot;
  float xbias;
  float y;
  float ydot;
  float ydotdot;
  float ybias;
  float xP[HFF_STATE_SIZE][HFF_STATE_SIZE];
  float yP[HFF_STATE_SIZE][HFF_STATE_SIZE];
  uint16_t lag_counter;
  bool rollback;
};

extern struct HfilterFloat hff;

extern void hff_init(float init_x, float init_xdot, float init_y, float init_ydot);
extern void hff_propagate(void);
extern void hff_update_gps(struct FloatVect2 *pos_ned, struct FloatVect2 *speed_ned);
extern void hff_update_pos(struct FloatVect2 pos, struct FloatVect2 Rpos);
extern void hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel);
extern void hff_realign(struct FloatVect2 pos, struct FloatVect2 vel);

#endif /* HF_FLOAT_H */

/*
 * Copyright (C) 2023 Ewoud Smeur
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

#include "state.h"

float g = -9.81f;

struct FloatVect2 ballistic_pos;

void ballistic_touchdown_init(void) {
  // nothing to be done here
}

/**
 * Function that predicts the ballistic crash location
 *
 * Uses ENU coordinates (vertical up!)
 **/
void ballistic_touchdown_run(void) {

  struct EnuCoor_f * v = stateGetSpeedEnu_f();
  float vz = v->z;

  struct FloatVect2 vh;
  VECT2_ASSIGN(vh, v->x, v->y);

  float h = fabsf(stateGetPositionEnu_f()->z); // Should be height above ground, make sure to initialize local frame on ground

  // With h always larger than 0, the sqrt can never give nan
  float time_fall = (-vz - sqrtf(vz*vz -2.f*h*g))/g;

  struct FloatVect2 crash_offset;

  VECT2_SMUL(crash_offset, vh, time_fall);

  struct FloatVect2 pos;
  pos.x = stateGetPositionEnu_f()->x;
  pos.y = stateGetPositionEnu_f()->y;

  // The predicted crash position is the current drone position + fall distance
  VECT2_SUM(ballistic_pos, pos, crash_offset);
}

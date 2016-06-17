/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


// Own header
#include "stereo_avoid.h"

// Navigate Based On Vision
#include "avoid_navigation.h"

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude

#include "std.h"


void stereo_avoid_init(void)
{
  // Simulated vision: obstacle at 0,0
  avoid_navigation_data.stereo_bin[0] = 0;
  avoid_navigation_data.stereo_bin[1] = 0;

  // Navigation Code
  init_avoid_navigation();
}

/** Normalize a degree angle between 0 and 359 */
#define NormAngleRad(x) { \
    int dont_loop_forever = 0;  \
    while ((x < (-M_PI)) && ++dont_loop_forever) x += (2 * M_PI); \
    while ((x >= (M_PI)) && ++dont_loop_forever) x -= (2 * M_PI); \
  }


void stereo_avoid_run(void)
{
  static int counter = 0;

  counter++;
  // Read Latest GST Module Results
  if (counter >= (2)) {
    // Vertical
    float dx = sqrt((stateGetPositionEnu_f()->x * stateGetPositionEnu_f()->x) + (stateGetPositionEnu_f()->y *
                    stateGetPositionEnu_f()->y));

    /*
    float dh = (17.0f - stateGetPositionEnu_f()->z);
    float vang = atan2(dh,dx) * 80.9F * 2.0f; // 255 / PI

    if (vang < 0.0f)
      vang = 0.0f;
    if (vang > 255.0f)
      vang = 255.0f;
    */


    // Horizontal
    float bearing = atan2(- stateGetPositionEnu_f()->x, - stateGetPositionEnu_f()->y);
    float heading = stateGetNedToBodyEulers_f()->psi;
    float diff = bearing - heading;
    NormAngleRad(diff);
    diff = DegOfRad(diff);

    //float hang = DegOfRad(atan2(10,dx)); // 255 / PI


    float viewangle = 50; // degrees
    float range = viewangle / 2.0f;
    //float deg_per_bin = viewangle / ((float) 2);

    float disperity = 0;

    if (fabs(diff) > range) {
      disperity = 0;
    } else if (dx > 2) {
      disperity = 0;
    } else {
      disperity = 30;
    }

    //float bin_nr_mid = range/deg_per_bin;
    //float bin_nr_start = bin_nr_mid + diff/deg_per_bin - hang/deg_per_bin;
    //float bin_nr_stop  = bin_nr_mid + diff/deg_per_bin + hang/deg_per_bin;

    avoid_navigation_data.stereo_bin[0] = disperity;
    avoid_navigation_data.stereo_bin[1] = diff;

    counter = 0;
    run_avoid_navigation_onvision();
  }
}


void stereo_avoid_start(void)
{
}


void stereo_avoid_stop(void)
{
}

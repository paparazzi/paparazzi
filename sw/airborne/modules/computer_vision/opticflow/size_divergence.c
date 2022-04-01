/*
 * Copyright (C) 2015 Guido de Croon <guido.de.croon@gmail.com>
 *
 * From:
 * Characterization of Flow Field Divergence for Vertical Landing Control of MAVs
 * by H.W. Ho and G.C.H.E. de Croon (submitted)
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/size_divergence.c
 * @brief Calculate divergence from flow vectors by looking at linefloat get_size_divergence(struct flow_t *vectors, int count, int n_samples)
 *  sizes between the points.
 *
 * Uses optical flow vectors as determined with a corner tracker and Lucas Kanade to estimate divergence.
 *
 * MAV COURSE ADDITION:
 * Extra function added, based on existing get_size_divergence to split divergence into left and half
 */

#include "size_divergence.h"
#include <stdlib.h>

/**
 * Get divergence from optical flow vectors based on line sizes between corners
 * @param[in] vectors    The optical flow vectors
 * @param[in] count      The number of optical flow vectors
 * @param[in] n_samples  The number of line segments that will be taken into account. 0 means all line segments will be considered.
 * @return divergence
 */
float get_size_divergence(struct flow_t *vectors, int count, int n_samples)
{
  float distance_1, distance_2;
  float divs_sum = 0.f;
  uint32_t used_samples = 0;
  float dx, dy;
  int32_t i, j;

  int32_t max_samples = (count * count - count) / 2;

  if (count < 2) {
    return 0.f;
  } else if (count >= max_samples) {
    n_samples = 0;
  }

  if (n_samples == 0) {
    // go through all possible lines:
    for (i = 0; i < count; i++) {
      for (j = i + 1; j < count; j++) {
        // distance in previous image:
        dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
        dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
        distance_1 = sqrtf(dx * dx + dy * dy);

        if (distance_1 < 1E-5) {
          continue;
        }

        // distance in current image:
        dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
        dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
        distance_2 = sqrtf(dx * dx + dy * dy);

        divs_sum += (distance_2 - distance_1) / distance_1;
        used_samples++;
      }
    }
  } else {
    // take random samples:
    for (uint16_t sample = 0; sample < n_samples; sample++) {
      // take two random indices:
      i = rand() % count;
      j = rand() % count;
      // ensure it is not the same index:
      while (i == j) {
        j = rand() % count;
      }

      // distance in previous image:
      dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
      dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
      distance_1 = sqrtf(dx * dx + dy * dy);

      if (distance_1 < 1E-5) {
        continue;
      }

      // distance in current image:
      dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
      dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
      distance_2 = sqrtf(dx * dx + dy * dy);

      divs_sum += (distance_2 - distance_1) / distance_1;
      used_samples++;
    }
  }

  if (used_samples < 1){
    return 0.f;
  }

  // return the calculated mean divergence:
  return divs_sum / used_samples;
}

/**
 * Get divergence on left and right hand side of image from optical flow vectors based on line sizes between corners
 * @param[in] vectors    The optical flow vectors
 * @param[in] img        The paramaters of the image on which optical flow was computed
 * @param[in] count      The number of optical flow vectors
 * @param[in] n_samples  The number of line segments that will be taken into account. 0 means all line segments will be considered.
 * @param[in] divs[]	 Empty array that is passed into the function and can be used to access left and right divergence.
 * @return void
 *
 */


void get_size_divergence_split(struct flow_t *vectors, struct image_t *img,  int count, int n_samples, float divs[])
{
  float distance_1, distance_2;
  float divs_sum_left  = 0.f;
  float divs_sum_right = 0.f;
  uint32_t used_samples_left = 0;
  uint32_t used_samples_right =0;
  float dx, dy;
  int32_t i, j;

  int32_t max_samples = (count * count - count) / 2;

  if (count < 2) {
    return ;
  } else if (count >= max_samples) {
    n_samples = 0;
  }

  if (n_samples == 0) {
    // go through all possible lines:
    for (i = 0; i < count; i++) {
      for (j = i + 1; j < count; j++) {
        // distance in previous image:
        dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
        dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
        distance_1 = sqrtf(dx * dx + dy * dy);

        if (distance_1 < 1E-5) {
          continue;
        }

        // distance in current image:
        dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
        dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
        distance_2 = sqrtf(dx * dx + dy * dy);

        if ((float)vectors[i].pos.y <= img->w/2) {

        	divs_sum_left += (distance_2 - distance_1) / distance_1;

        	used_samples_left++;
        } else {
        	divs_sum_right += (distance_2 - distance_1) / distance_1;

        	        	used_samples_right++;}
      }
    }
  } else {
    // take random samples:
    for (uint16_t sample = 0; sample < n_samples; sample++) {
      // take two random indices:
      i = rand() % count;
      j = rand() % count;
      // ensure it is not the same index:
      while (i == j) {
        j = rand() % count;
      }

      // distance in previous image:
      dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
      dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
      distance_1 = sqrtf(dx * dx + dy * dy);

      if (distance_1 < 1E-5) {
        continue;
      }

      // distance in current image:
      dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
      dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
      distance_2 = sqrtf(dx * dx + dy * dy);

      //check whether point that is analysed is in the left or right hand side of the image
      if ((float)vectors[i].pos.y <= img->h/2) {
    	  //if left, add divergence to divergence sum of left image
      	divs_sum_left += (distance_2 - distance_1) / distance_1;

      	used_samples_left++;
      } else {
    	  //if right, add divergence to divergence sum of right image
      	divs_sum_right += (distance_2 - distance_1) / distance_1;

      	        	used_samples_right++;}
    }
  }

  // return the calculated mean divergence:

  if (used_samples_left < 1 ){
    divs[0]=0;
  }

  else {divs[0] = divs_sum_left/used_samples_left;}

  if (used_samples_right < 1){
	  divs[1]=0;
  }

  else {divs[1] = divs_sum_right/used_samples_right;}


}

/*
 * Copyright (C) 2015
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/computer_vision/blob/imavmarker.c"
 * Find a IMAV pattern
 */



#include "imavmarker.h"


/**
 * Find the marker location.
 * @param[in] *input The input image to filter
 * @param[in] M The distance between the pixel of interest and farthest neighbor pixel [pixel]
 * @return The deviation of the marker location wrt the center.
 */


#define Img(X,Y)(source[(Y)*input->w*2+(X)*2+1])
#define Out(X,Y)(source[(Y)*input->w*2+(X)*2])

inline int AbsDiff(int A, int B);
inline int AbsDiff(int A, int B)
{
  if (A < B) {
    return B - A;
  }
  return A - B;
}

struct marker_deviation_t marker(struct image_t *input, uint8_t M)
{
  struct marker_deviation_t marker_deviation;

  uint8_t *source = (uint8_t *) input->buf;
  uint16_t x, y, i, j, k;

  if (M < 1) { M = 1; }
  source = (uint8_t *) input->buf;

  int maxx = 160;
  int maxy = 120;
  int maxv = 0;

  for (j = M; j < (input->h - M); j++) {
    for (i = M; i < (input->w - M); i++) {
      int bad, good;
      good = bad = 0;
      for (k = 1; k < M; k++) {
        // Pattern must be symmetric
        bad += AbsDiff(Img(i - k, j)   , Img(i + k, j));
        bad += AbsDiff(Img(i, j - k)   , Img(i, j + k));
        bad += AbsDiff(Img(i - k, j - k) , Img(i + k, j + k));
        bad += AbsDiff(Img(i + k, j - k) , Img(i - k, j + k));

        // Pattern: Must have perpendicular contrast
        good += AbsDiff(Img(i - k, j) + Img(i + k, j),   Img(i, j - k) + Img(i, j + k));
        good += AbsDiff(Img(i - k, j - k) + Img(i + k, j + k), Img(i + k, j - k) + Img(i - k, j + k));
      }

      for (k = 4; k < M; k += 2) {
        // Pattern must be symmetric
        bad += AbsDiff(Img(i - k, j - k / 2)   , Img(i + k, j + k / 2));
        bad += AbsDiff(Img(i + k / 2, j - k)   , Img(i - k / 2, j + k));
        bad += AbsDiff(Img(i - k / 2, j - k) , Img(i + k / 2, j + k));
        bad += AbsDiff(Img(i + k, j - k / 2) , Img(i - k, j + k / 2));

        // Pattern: Must have perpendicular contrast
        good += AbsDiff(Img(i - k, j - k / 2) + Img(i + k, j + k / 2),   Img(i + k / 2, j - k) + Img(i - k / 2, j + k));
        good += AbsDiff(Img(i - k / 2, j - k) + Img(i + k / 2, j + k), Img(i + k, j - k / 2) + Img(i - k, j + k / 2));
      }

      int v = good - bad;
      if (v < 0) {
        v = 0;
      }

      if (v > maxv) {
        maxv = v;
        maxx = i;
        maxy = j;
      }

      if (v > 0) {
        Out(i, j) = 0xff;
      }
    }
  }

  // Display the marker location and center-lines.
  for (y = 0; y < input->h; y++) {
    Out(maxx, y) = 0xff;
  }
  for (x = 0; x < input->w; x++) {
    Out(x, maxy) = 0xff;
  }

  marker_deviation.x = ((int32_t)0) - ((int32_t)(input->w) / 2);
  marker_deviation.y = -((int32_t)0) + ((int32_t)(input->h) / 2);
  marker_deviation.inlier = 0;

  //printf("The number of inliers = %i\n", counter3);
  return marker_deviation;
}


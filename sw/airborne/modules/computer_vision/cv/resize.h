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


#include <stdint.h>
#include "image.h"

/** Simplified high-speed low CPU downsample function without averaging
 *
 *  downsample factor must be 1, 2, 4, 8 ... 2^X
 *  image of typ UYVY expected. Only one color UV per 2 pixels
 *
 *  we keep the UV color of the first pixel pair
 *  and sample the intensity evenly 1-3-5-7-... or 1-5-9-...
 *
 *  input:         u1y1 v1y2 u3y3 v3y4 u5y5 v5y6 u7y7 v7y8 ...
 *  downsample=1   u1y1 v1y2 u3y3 v3y4 u5y5 v5y6 u7y7 v7y8 ...
 *  downsample=2   u1y1v1 (skip2) y3 (skip2) u5y5v5 (skip2 y7 (skip2) ...
 *  downsample=4   u1y1v1 (skip6) y5 (skip6) ...
 */

inline void resize_uyuv(struct img_struct *input, struct img_struct *output, int downsample);
inline void resize_uyuv(struct img_struct *input, struct img_struct *output, int downsample)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  int pixelskip = (downsample - 1) * 2;
  for (int y = 0; y < output->h; y++) {
    for (int x = 0; x < output->w; x += 2) {
      // YUYV
      *dest++ = *source++; // U
      *dest++ = *source++; // Y
      *dest++ = *source++; // V
      source += pixelskip;
      *dest++ = *source++; // Y
      source += pixelskip;
    }
    // read 1 in every 'downsample' rows, so skip (downsample-1) rows after reading the first
    source += (downsample-1) * input->w * 2;
  }
}


/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/lib/vision/bayer.h
 */

#ifndef Bayer_H
#define Bayer_H

#include "lib/vision/image.h"

/**
 * @brief Decode Bayer Pattern
 * @param RedX, RedY the coordinates of the upper-rightmost green pixel
 *        which has a red pixel next to it and a blue underneath
 *
 */

inline void BayerToYUV(struct image_t *in, struct image_t *out,
                 int RedX, int RedY)
{
  uint16_t *ii = (uint16_t *) in->buf;
  uint8_t *oi = (uint8_t *) out->buf;
  int x, y;

  for (y = 0; y < out->h; y++) {
    for (x = 0; x < out->w; x += 2) {
      /* RGB Bayer:
       * RBRBRBRBRBRBRBRB
       * GRGRGRGRGRGRGRGR
       */
      int i = 2 * (out->h - y) + RedX;
      int j = 2 * x + RedY;
      if ((i < in->w) && (j < in->h)) {
        uint16_t G1 = ii[i + j * in->w] / 2;
        uint16_t R1 = ii[i + j * in->w + 1] / 2;
        uint16_t B1 = ii[i + (j + 1) * in->w] / 2;
        j += 2;
        uint16_t G2 = ii[i + j * in->w] / 2;
        uint16_t R2 = ii[i + j * in->w + 1] / 2;
        uint16_t B2 = ii[i + (j + 1) * in->w] / 2;

        uint32_t u, my1, v, my2;

        my1 = (0.256788 * R1 + 0.504129 * G1 + 0.097906 * B1) / 128 +  16;
        my2 = (0.256788 * R2 + 0.504129 * G2 + 0.097906 * B2) / 128 +  16;
        u = (-0.148223 * (R1 + R2) - 0.290993 * (G1 + G2) + 0.439216 * (B1 + B2)) / 256 + 128;
        v = (0.439216 * (R1 + R2) - 0.367788 * (G1 + G2) - 0.071427 * (B1 + B2)) / 256 + 128;

        oi[(x + y * out->w) * 2] = Clip(u, 0, 255);
        oi[(x + y * out->w) * 2 + 1] = Clip(my1, 0, 255);
        oi[(x + y * out->w) * 2 + 2] = Clip(v, 0, 255);
        oi[(x + y * out->w) * 2 + 3] = Clip(my2, 0, 255);
      } else {
        oi[(x + y * out->w) * 2] = 0;
        oi[(x + y * out->w) * 2 + 1] = 0;
        oi[(x + y * out->w) * 2 + 2] = 0;
        oi[(x + y * out->w) * 2 + 3] = 0;
      }
    }
  }
}

#endif /* Bayer_H */

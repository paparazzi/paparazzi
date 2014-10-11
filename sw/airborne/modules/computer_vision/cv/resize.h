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

inline void resize_uyuv(struct img_struct* input, struct img_struct* output, int downsample);
inline void resize_uyuv(struct img_struct* input, struct img_struct* output, int downsample)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  int pixelskip = downsample-1;
  for (int y=0;y<output->h;y++)
  {
    for (int x=0;x<output->w;x+=2)
    {
      // YUYV
      *dest++ = *source++; // U
      *dest++ = *source++; // Y
      // now skip 3 pixels
      if (pixelskip) source+=(pixelskip+1)*2;
      *dest++ = *source++; // U
      *dest++ = *source++; // V
      if (pixelskip) source+=(pixelskip-1)*2;
    }
    // skip 3 rows
    if (pixelskip) source += pixelskip * input->w * 2;
  }
}


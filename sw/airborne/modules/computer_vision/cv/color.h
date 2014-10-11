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

inline void grayscale_uyvy(struct img_struct* input, struct img_struct* output);
inline void grayscale_uyvy(struct img_struct* input, struct img_struct* output)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  source++;

  for (int y=0;y<output->h;y++)
  {
    for (int x=0;x<output->w;x++)
    {
      // UYVY
      *dest++ = 127;        // U
      *dest++ = *source;    // Y
      source+=2;
    }
  }
}

inline int colorfilt_uyvy(struct img_struct* input, struct img_struct* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);
inline int colorfilt_uyvy(struct img_struct* input, struct img_struct* output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  int cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  for (int y=0;y<output->h;y++)
  {
    for (int x=0;x<output->w;x+=2)
    {
      // Color Check:
      if (
          // Light
               (dest[1] >= y_m)
            && (dest[1] <= y_M)
            && (dest[0] >= u_m)
            && (dest[0] <= u_M)
            && (dest[2] >= v_m)
            && (dest[2] <= v_M)
         )// && (dest[2] > 128))
      {
        cnt ++;
        // UYVY
        dest[0] = 64;        // U
        dest[1] = source[1];  // Y
        dest[2] = 255;        // V
        dest[3] = source[3];  // Y
      }
      else
      {
        // UYVY
        char u = source[0]-127;
        u/=4;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2]-127;
        u/=4;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
      }

      dest+=4;
      source+=4;
    }
  }
  return cnt;
}


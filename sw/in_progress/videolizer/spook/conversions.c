/*
 * Copyright (C) 2000-2001 Dan Dennedy  <dan@dennedy.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/* This file was borrowed from Coriander 0.18 for Spook */

#include <string.h>
#include <conversions.h>

extern void swab();

inline void
uyvy2yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
	swab(src, dest, NumPixels << 1);
}

inline void
yuv2yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
	swab(src, dest, NumPixels << 1);
}

inline void
y41p2yuyv (unsigned char *src, unsigned char *dest, int NumPixels) {
  register int i=0,j=0;
  register int y0, y1, y2, y3, y4, y5, y6, y7, u0, u4, v0, v4;
  while (i < (NumPixels << 1) )
    {
      u0 = src[i++];
      y0 = src[i++];
      v0 = src[i++];
      y1 = src[i++];
      
      u4 = src[i++];
      y2 = src[i++];
      v4 = src[i++];
      y3 = src[i++];

      y4 = src[i++];
      y5 = src[i++];
      y6 = src[i++];
      y7 = src[i++];

      dest[j++] = y0;
      dest[j++] = u0;
      dest[j++] = y1;
      dest[j++] = v0;

      dest[j++] = y2;
      dest[j++] = u0;
      dest[j++] = y3;
      dest[j++] = v0;

      dest[j++] = y4;
      dest[j++] = u4;
      dest[j++] = y5;
      dest[j++] = v4;

      dest[j++] = y6;
      dest[j++] = u4;
      dest[j++] = y7;
      dest[j++] = v4;

    }
}


inline void
iyu12yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
  register int i=0,j=0;
  register int y0, y1, y2, y3, u, v;
  while (i < (NumPixels + (NumPixels >> 1)))
    {
      u = src[i++];
      y0 = src[i++];
      y1 = src[i++];
      v = src[i++];
      y2 = src[i++];
      y3 = src[i++];

      dest[j++] = y0;
      dest[j++] = u;
      dest[j++] = y1;
      dest[j++] = v;

      dest[j++] = y2;
      dest[j++] = u;
      dest[j++] = y3;
      dest[j++] = v;
    }
}

inline void
iyu22yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
  int i=0,j=0;
  register int y0, y1, u0, u1, v0, v1;
  while (i < (NumPixels + (NumPixels << 1)))
    {
      u0 = src[i++];
      y0 = src[i++];
      v0 = src[i++];
      u1 = src[i++];
      y1 = src[i++];
      v1 = src[i++];

      dest[j++] = y0;
      dest[j++] = (u0+u1)/2;
      dest[j++] = y1;
      dest[j++] = (v0+v1)/2;
    }
}

inline void
y2yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
  int i=0,j=0;
  register int y0, y1;
  while (i < NumPixels)
    {
      y0 = src[i++];
      y1 = src[i++];

      dest[j++] = y0;
      dest[j++] = 128;
      dest[j++] = y1;
      dest[j++] = 128;
    }
}

/*macro used to convert a YUV pixel to RGB format
  from Bart Nabbe

  v*1434 / 2048
  (u*406 - v*595)/2048
  u*2078 / 2048
  bitshift performance enhanced by Damien Douxchamps
*/

#define YUV2RGB(y, u, v, r, g, b)\
  r = y + ( ((v << 10) + (v << 8) + (v << 7) + (v << 4) + (v << 3) + (v << 1)) >> 11);\
  g = y - ( ((u << 8) + (u << 7) + (u << 4) + (u << 2) + (u << 1)) >> 11 ) - \
          ( ((v << 9) + (v << 6) + (v << 4) + (v << 1) + v) >> 11 );\
  b = y + ( ((u << 11) + (u << 5) - (u << 1)) >> 11);\
  r = r < 0 ? 0 : r;\
  g = g < 0 ? 0 : g;\
  b = b < 0 ? 0 : b;\
  r = r > 255 ? 255 : r;\
  g = g > 255 ? 255 : g;\
  b = b > 255 ? 255 : b
  

#define RGB2YUV(r, g, b, y, u, v)\
  y = (9798*r + 19235*g + 3736*b)  >> 15;\
  u = ((-4784*r - 9437*g + 14221*b) >> 15)  + 128;\
  v = ((20218*r - 16941*g - 3277*b) >> 15) + 128;\
  y = y < 0 ? 0 : y;\
  u = u < 0 ? 0 : u;\
  v = v < 0 ? 0 : v;\
  y = y > 255 ? 255 : y;\
  u = u > 255 ? 255 : u;\
  v = v > 255 ? 255 : v

inline void
rgb2yuy2 (char *RGB, char *YUV, int NumPixels) {
  int i, j;
  register int y0, y1, u0, u1, v0, v1 ;
  register int r, g, b;

  for (i = 0, j = 0; i < (NumPixels + (NumPixels << 1)); i += 6, j += 4)
    {
      r = (unsigned char) RGB[i + 0];
      g = (unsigned char) RGB[i + 1];
      b = (unsigned char) RGB[i + 2];
      RGB2YUV (r, g, b, y0, u0 , v0);
      r = (unsigned char) RGB[i + 3];
      g = (unsigned char) RGB[i + 4];
      b = (unsigned char) RGB[i + 5];
      RGB2YUV (r, g, b, y1, u1 , v1);
      YUV[j + 0] = y0;
      YUV[j + 1] = (u0+u1) >> 1;
      YUV[j + 2] = y1;
      YUV[j + 3] = (v0+v1) >> 1;
    }
}

/*routine to convert an array of YUV data to RGB format
  from Bart Nabbe
*/
inline void
uyvy2rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  register int y0, y1, u, v;
  register int r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      u = (unsigned char) YUV[i + 0] - 128;
      y0 = (unsigned char) YUV[i + 1];
      v = (unsigned char) YUV[i + 2] - 128;
      y1 = (unsigned char) YUV[i + 3];
      YUV2RGB (y0, u, v, r, g, b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, r, g, b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}


inline void
yuy22rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  register int y0, y1, u, v;
  register int r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      y0 = (unsigned char) YUV[i + 0];
      u = (unsigned char) YUV[i + 1] - 128;
      y1 = (unsigned char) YUV[i + 2];
      v = (unsigned char) YUV[i + 3] - 128;
      YUV2RGB (y0, u, v, r, g, b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, r, g, b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}

inline void
iyu12rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  register int y0, y1, y2, y3, u, v;
  register int r, g, b;

  for (i = 0, j = 0; i < (NumPixels + (NumPixels >> 1)); i += 6, j += 12)
    {
      u  = (unsigned char) YUV[i + 0] - 128;
      y0 = (unsigned char) YUV[i + 1];
      y1 = (unsigned char) YUV[i + 2];
      v  = (unsigned char) YUV[i + 3] - 128;
      y2 = (unsigned char) YUV[i + 4];
      y3 = (unsigned char) YUV[i + 5];
      YUV2RGB (y0, u, v, r, g, b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, r, g, b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
      YUV2RGB (y2, u, v, r, g, b);
      RGB[j + 6] = r;
      RGB[j + 7] = g;
      RGB[j + 8] = b;
      YUV2RGB (y3, u, v, r, g, b);
      RGB[j + 9] = r;
      RGB[j + 10] = g;
      RGB[j + 11] = b;
    }
}

inline void
iyu22rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  register int y0, y1, u0, u1, v0, v1;
  register int r, g, b;

  for (i = 0, j = 0; i < (NumPixels + (NumPixels << 1)); i += 6, j += 6)
    {
      u0 = (unsigned char) YUV[i + 0] - 128;
      y0 = (unsigned char) YUV[i + 1];
      v0 = (unsigned char) YUV[i + 2] - 128;
      u1 = (unsigned char) YUV[i + 3] - 128;
      y1 = (unsigned char) YUV[i + 4];
      v1 = (unsigned char) YUV[i + 5] - 128;
      YUV2RGB (y0, u0, v0, r, g, b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u1, v1, r, g, b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}

inline void
y2rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  register int y;
  register int r, g, b;

  for (i = 0, j = 0; i < NumPixels; i++, j += 3)
    {
      y = (unsigned char) YUV[i];
      YUV2RGB (y, 0, 0, r, g, b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
    }
}

/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file modules/computer_vision/lib/vision/image.c
 * Image helper functions, like resizing, color filter, converters...
 */

#include "image.h"
#include <stdlib.h>
#include <string.h>

/**
 * Create a new image
 * @param[out] *img The output image
 * @param[in] width The width of the image
 * @param[in] height The height of the image
 * @param[in] type The type of image (YUV422 or grayscale)
 */
void image_create(struct image_t *img, uint16_t width, uint16_t height, enum image_type type)
{
  // Set the variables
  img->type = type;
  img->w = width;
  img->h = height;

  // Depending on the type the size differs
  if(type == IMAGE_YUV422)
    img->buf_size = sizeof(uint8_t)*2 * width * height;
  else if(type == IMAGE_JPEG)
    img->buf_size = sizeof(uint8_t)*1.1 * width * height; // At maximum quality this is enough
  else
    img->buf_size = sizeof(uint8_t) * width * height;

  img->buf = malloc(img->buf_size);
}

/**
 * Free the image
 * @param[in] *img The image to free
 */
void image_free(struct image_t *img)
{
  free(img->buf);
}

/**
 * Copy an image from inut to output
 * This will only work if the formats are the same
 * @param[in] *input The input image to copy from
 * @param[out] *output The out image to copy to
 */
void image_copy(struct image_t *input, struct image_t *output)
{
  if(input->type != output->type)
    return;

  output->w = input->w;
  output->h = input->h;
  output->buf_size = input->buf_size;
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));
  memcpy(output->buf, input->buf, input->buf_size);
}

/**
 * Convert an image to grayscale.
 * Depending on the output type the U/V bytes are removed
 * @param[in] *input The input image (Needs to be YUV422)
 * @param[out] *output The output image
 */
void image_to_grayscale(struct image_t *input, struct image_t *output)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  source++;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Copy the pixels
  for (int y = 0; y < output->h; y++) {
    for (int x = 0; x < output->w; x++) {
      if(output->type == IMAGE_YUV422)
        *dest++ = 127;      // U / V
      *dest++ = *source;    // Y
      source += 2;
    }
  }
}

/**
 * Filter colors in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @return The amount of filtered pixels
 */
uint16_t image_yuv422_colorfilt(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                          uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  uint16_t cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values
      if (
        (dest[1] >= y_m)
        && (dest[1] <= y_M)
        && (dest[0] >= u_m)
        && (dest[0] <= u_M)
        && (dest[2] >= v_m)
        && (dest[2] <= v_M)
      ) {
        cnt ++;
        // UYVY
        dest[0] = 64;        // U
        dest[1] = source[1];  // Y
        dest[2] = 255;        // V
        dest[3] = source[3];  // Y
      } else {
        // UYVY
        char u = source[0] - 127;
        u /= 4;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2] - 127;
        u /= 4;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
      }

      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
  return cnt;
}

/**
* Simplified high-speed low CPU downsample function without averaging
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
* @param[in] *input The input YUV422 image
* @param[out] *output The downscaled YUV422 image
* @param[in] downsample The downsampel facter (must be downsample=2^X)
*/
void image_yuv422_downsample(struct image_t *input, struct image_t *output, uint16_t downsample)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  uint16_t pixelskip = (downsample - 1) * 2;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
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

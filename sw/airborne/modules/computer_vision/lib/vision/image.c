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
  if (type == IMAGE_YUV422) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;
  } else if (type == IMAGE_JPEG) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;  // At maximum quality this is enough
  } else if (type == IMAGE_GRADIENT) {
    img->buf_size = sizeof(int16_t) * width * height;
  } else {
    img->buf_size = sizeof(uint8_t) * width * height;
  }

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
  if (input->type != output->type) {
    return;
  }

  output->w = input->w;
  output->h = input->h;
  output->buf_size = input->buf_size;
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));
  memcpy(output->buf, input->buf, input->buf_size);
}

/**
 * This will switch image *a and *b
 * This is faster as image_copy because it doesn't copy the
 * whole image buffer.
 * @param[in,out] *a The image to switch
 * @param[in,out] *b The image to switch with
 */
void image_switch(struct image_t *a, struct image_t *b)
{
  /* Remember everything from image a */
  struct image_t old_a;
  memcpy(&old_a, a, sizeof(struct image_t));

  /* Copy everything from b to a */
  memcpy(a, b, sizeof(struct image_t));

  /* Copy everything from the remembered a to b */
  memcpy(b, &old_a, sizeof(struct image_t));
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
      if (output->type == IMAGE_YUV422) {
        *dest++ = 127;  // U / V
      }
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
    source += (downsample - 1) * input->w * 2;
  }
}

/**
 * This outputs a subpixel window image in grayscale
 * Currently only works with Grayscale images as input but could be upgraded to
 * also support YUV422 images.
 * @param[in] *input Input image (grayscale only)
 * @param[out] *output Window output (width and height is used to calculate the window size)
 * @param[in] *center Center point in subpixel coordinates
 * @param[in] subpixel_factor The subpixel factor per pixel
 */
void image_subpixel_window(struct image_t *input, struct image_t *output, struct point_t *center, uint16_t subpixel_factor)
{
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint8_t *output_buf = (uint8_t *)output->buf;

  // Calculate the window size
  uint16_t half_window = output->w / 2;
  uint16_t subpixel_w = input->w * subpixel_factor;
  uint16_t subpixel_h = input->h * subpixel_factor;

  // Go through the whole window size in normal coordinates
  for (uint16_t i = 0; i < output->w; i++) {
    for (uint16_t j = 0; j < output->h; j++) {
      // Calculate the subpixel coordinate
      uint16_t x = center->x + (i - half_window) * subpixel_factor;
      uint16_t y = center->y + (j - half_window) * subpixel_factor;
      Bound(x, 0, subpixel_w);
      Bound(y, 0, subpixel_h);

      // Calculate the original pixel coordinate
      uint16_t orig_x = x / subpixel_factor;
      uint16_t orig_y = y / subpixel_factor;

      // Calculate top left (in subpixel coordinates)
      uint16_t tl_x = orig_x * subpixel_factor;
      uint16_t tl_y = orig_y * subpixel_factor;

      // Check if it is the top left pixel
      if (tl_x == x &&  tl_y == y) {
        output_buf[output->w * j + i] = input_buf[input->w * orig_y + orig_x];
      } else {
        // Calculate the difference from the top left
        uint16_t alpha_x = (x - tl_x);
        uint16_t alpha_y = (y - tl_y);

        // Blend from the 4 surrounding pixels
        uint32_t blend = (subpixel_factor - alpha_x) * (subpixel_factor - alpha_y) * input_buf[input->w * orig_y + orig_x];
        blend += alpha_x * (subpixel_factor - alpha_y) * input_buf[input->w * orig_y + (orig_x + 1)];
        blend += (subpixel_factor - alpha_x) * alpha_y * input_buf[input->w * (orig_y + 1) + orig_x];
        blend += alpha_x * alpha_y * input_buf[input->w * (orig_y + 1) + (orig_x + 1)];

        // Set the normalized pixel blend
        output_buf[output->w * j + i] = blend / (subpixel_factor * subpixel_factor);
      }
    }
  }
}

/**
 * Calculate the  gradients using the following matrix:
 * [0 -1 0; -1 0 1; 0 1 0]
 * @param[in] *input Input grayscale image
 * @param[out] *dx Output gradient in the X direction (dx->w = input->w-2, dx->h = input->h-2)
 * @param[out] *dy Output gradient in the Y direction (dx->w = input->w-2, dx->h = input->h-2)
 */
void image_gradients(struct image_t *input, struct image_t *dx, struct image_t *dy)
{
  // Fetch the buffers in the correct format
  uint8_t *input_buf = (uint8_t *)input->buf;
  int16_t *dx_buf = (int16_t *)dx->buf;
  int16_t *dy_buf = (int16_t *)dy->buf;

  // Go trough all pixels except the borders
  for (uint16_t x = 1; x < input->w - 1; x++) {
    for (uint16_t y = 1; y < input->h - 1; y++) {
      dx_buf[(y - 1)*dx->w + (x - 1)] = (int16_t)input_buf[y * input->w + x + 1] - (int16_t)input_buf[y * input->w + x - 1];
      dy_buf[(y - 1)*dy->w + (x - 1)] = (int16_t)input_buf[(y + 1) * input->w + x] - (int16_t)input_buf[(y - 1) * input->w + x];
    }
  }
}

/**
 * Calculate the G vector of an image gradient
 * This is used for optical flow calculation.
 * @param[in] *dx The gradient in the X direction
 * @param[in] *dy The gradient in the Y direction
 * @param[out] *g The G[4] vector devided by 255 to keep in range
 */
void image_calculate_g(struct image_t *dx, struct image_t *dy, int32_t *g)
{
  int32_t sum_dxx = 0, sum_dxy = 0, sum_dyy = 0;

  // Fetch the buffers in the correct format
  int16_t *dx_buf = (int16_t *)dx->buf;
  int16_t *dy_buf = (int16_t *)dy->buf;

  // Calculate the different sums
  for (uint16_t x = 0; x < dx->w; x++) {
    for (uint16_t y = 0; y < dy->h; y++) {
      sum_dxx += ((int32_t)dx_buf[y * dx->w + x] * dx_buf[y * dx->w + x]);
      sum_dxy += ((int32_t)dx_buf[y * dx->w + x] * dy_buf[y * dy->w + x]);
      sum_dyy += ((int32_t)dy_buf[y * dy->w + x] * dy_buf[y * dy->w + x]);
    }
  }

  // ouput the G vector
  g[0] = sum_dxx / 255;
  g[1] = sum_dxy / 255;
  g[2] = g[1];
  g[3] = sum_dyy / 255;
}

/**
 * Calculate the difference between two images and return the error
 * This will only work with grayscale images
 * @param[in] *img_a The image to substract from
 * @param[in] *img_b The image to substract from img_a
 * @param[out] *diff The image difference (if not needed can be NULL)
 * @return The squared difference summed
 */
uint32_t image_difference(struct image_t *img_a, struct image_t *img_b, struct image_t *diff)
{
  uint32_t sum_diff2 = 0;
  int16_t *diff_buf = NULL;

  // Fetch the buffers in the correct format
  uint8_t *img_a_buf = (uint8_t *)img_a->buf;
  uint8_t *img_b_buf = (uint8_t *)img_b->buf;

  // If we want the difference image back
  if (diff != NULL) {
    diff_buf = (int16_t *)diff->buf;
  }

  // Go trough the imagge pixels and calculate the difference
  for (uint16_t x = 0; x < img_b->w; x++) {
    for (uint16_t y = 0; y < img_b->h; y++) {
      int16_t diff_c = img_a_buf[(y + 1) * img_a->w + (x + 1)] - img_b_buf[y * img_b->w + x];
      sum_diff2 += diff_c * diff_c;

      // Set the difference image
      if (diff_buf != NULL) {
        diff_buf[y * diff->w + x] = diff_c;
      }
    }
  }

  return sum_diff2;
}

/**
 * Calculate the multiplication between two images and return the error
 * This will only work with image gradients
 * @param[in] *img_a The image to multiply
 * @param[in] *img_b The image to multiply with
 * @param[out] *mult The image multiplication (if not needed can be NULL)
 * @return The sum of the multiplcation
 */
int32_t image_multiply(struct image_t *img_a, struct image_t *img_b, struct image_t *mult)
{
  int32_t sum = 0;
  int16_t *img_a_buf = (int16_t *)img_a->buf;
  int16_t *img_b_buf = (int16_t *)img_b->buf;
  int16_t *mult_buf = NULL;

  // When we want an output
  if (mult != NULL) {
    mult_buf = (int16_t *)mult->buf;
  }

  // Calculate the multiplication
  for (uint16_t x = 0; x < img_a->w; x++) {
    for (uint16_t y = 0; y < img_a->h; y++) {
      int16_t mult_c = img_a_buf[y * img_a->w + x] * img_b_buf[y * img_b->w + x];
      sum += mult_c;

      // Set the difference image
      if (mult_buf != NULL) {
        mult_buf[y * mult->w + x] = mult_c;
      }
    }
  }

  return sum;
}

/**
 * Show points in an image by coloring them through giving
 * the pixels the maximum value.
 * This works with YUV422 and grayscale images
 * @param[in,out] *img The image to place the points on
 * @param[in] *points The points to sohw
 * @param[in] *points_cnt The amount of points to show
 */
void image_show_points(struct image_t *img, struct point_t *points, uint16_t points_cnt)
{
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;

  // Go trough all points and color them
  for (int i = 0; i < points_cnt; i++) {
    uint32_t idx = pixel_width * points[i].y * img->w + points[i].x * pixel_width;
    img_buf[idx] = 255;

    // YUV422 consists of 2 pixels
    if (img->type == IMAGE_YUV422) {
      idx++;
      img_buf[idx] = 255;
    }
  }
}

/**
 * Shows the flow from a specific point to a new point
 * This works on YUV422 and Grayscale images
 * @param[in,out] *img The image to show the flow on
 * @param[in] *vectors The flow vectors to show
 * @param[in] *points_cnt The amount of points and vectors to show
 */
void image_show_flow(struct image_t *img, struct flow_t *vectors, uint16_t points_cnt, uint8_t subpixel_factor)
{
  // Go through all the points
  for (uint16_t i = 0; i < points_cnt; i++) {
    // Draw a line from the original position with the flow vector
    struct point_t from = {
      vectors[i].pos.x / subpixel_factor,
      vectors[i].pos.y / subpixel_factor
    };
    struct point_t to = {
      (vectors[i].pos.x + vectors[i].flow_x) / subpixel_factor,
      (vectors[i].pos.y + vectors[i].flow_y) / subpixel_factor
    };
    image_draw_line(img, &from, &to);
  }
}

/**
 * Draw a line on the image
 * @param[in,out] *img The image to show the line on
 * @param[in] *from The point to draw from
 * @param[in] *to The point to draw to
 */
void image_draw_line(struct image_t *img, struct point_t *from, struct point_t *to)
{
  int xerr = 0, yerr = 0;
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;
  uint16_t startx = from->x;
  uint16_t starty = from->y;

  /* compute the distances in both directions */
  int32_t delta_x = to->x - from->x;
  int32_t delta_y = to->y - from->y;

  /* Compute the direction of the increment,
     an increment of 0 means either a horizontal or vertical
     line.
  */
  int8_t incx, incy;
  if (delta_x > 0) { incx = 1; }
  else if (delta_x == 0) { incx = 0; }
  else { incx = -1; }

  if (delta_y > 0) { incy = 1; }
  else if (delta_y == 0) { incy = 0; }
  else { incy = -1; }

  /* determine which distance is greater */
  uint16_t distance = 0;
  delta_x = abs(delta_x);
  delta_y = abs(delta_y);
  if (delta_x > delta_y) { distance = delta_x * 20; }
  else { distance = delta_y * 20; }

  /* draw the line */
  for (uint16_t t = 0; starty >= 0 && starty < img->h && startx >= 0 && startx < img->w && t <= distance + 1; t++) {
    img_buf[img->w * pixel_width * starty + startx * pixel_width] = (t <= 3) ? 0 : 255;

    if (img->type == IMAGE_YUV422) {
      img_buf[img->w * pixel_width * starty + startx * pixel_width + 1] = 255;

      if (startx + 1 < img->w) {
        img_buf[img->w * pixel_width * starty + startx * pixel_width + 2] = (t <= 3) ? 0 : 255;
        img_buf[img->w * pixel_width * starty + startx * pixel_width + 3] = 255;
      }
    }

    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance) {
      xerr -= distance;
      startx += incx;
    }
    if (yerr > distance) {
      yerr -= distance;
      starty += incy;
    }
  }
}

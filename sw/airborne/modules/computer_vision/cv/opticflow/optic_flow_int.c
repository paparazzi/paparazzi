/*
 * Copyright (C) 2014
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
 * @file modules/computer_vision/cv/opticflow/optic_flow_int.c
 * @brief efficient fixed-point optical-flow
 *
 * - Initial fixed-point C implementation by G. de Croon
 * - Algorithm: Lucas-Kanade by Yves Bouguet
 * - Publication: http://robots.stanford.edu/cs223b04/algo_tracking.pdf
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "optic_flow_int.h"
#include "modules/computer_vision/opticflow_module.h"

#define int_index(x,y) (y * IMG_WIDTH + x)
#define uint_index(xx, yy) (((yy * IMG_WIDTH + xx) * 2) & 0xFFFFFFFC)
#define NO_MEMORY -1
#define OK 0
#define N_VISUAL_INPUTS 51
#define N_ACTIONS 3
#define MAX_COUNT_PT 50

unsigned int IMG_WIDTH, IMG_HEIGHT;

void multiplyImages(int *ImA, int *ImB, int *ImC, int width, int height)
{
  int x, y;
  unsigned int ix;

  // printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  for (x = 0; x < width; x++) {
    for (y = 0; y < height; y++) {
      ix = (y * width + x);
      ImC[ix] = ImA[ix] * ImB[ix];
      // If we want to keep the values in [0, 255]:
      // ImC[ix] /= 255;
    }
  }
}

void getImageDifference(int *ImA, int *ImB, int *ImC, int width, int height)
{
  int x, y;
  unsigned int ix;

  // printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  for (x = 0; x < width; x++) {
    for (y = 0; y < height; y++) {
      ix = (y * width + x);
      ImC[ix] = ImA[ix] - ImB[ix];
    }
  }

}

void getSubPixel_gray(int *Patch, unsigned char *frame_buf, int center_x, int center_y, int half_window_size,
                      int subpixel_factor)
{
  int x, y, x_0, y_0, x_0_or, y_0_or, i, j, window_size, alpha_x, alpha_y, max_x, max_y;
  unsigned int ix1, ix2, Y;
  window_size = half_window_size * 2 + 1;
  max_x = (IMG_WIDTH - 1) * subpixel_factor;
  max_y = (IMG_HEIGHT - 1) * subpixel_factor;

  for (i = 0; i < window_size; i++) {
    for (j = 0; j < window_size; j++) {
      // index for this position in the patch:
      ix1 = (j * window_size + i);

      // determine subpixel coordinates of the current pixel:
      x = center_x + (i - half_window_size) * subpixel_factor;
      if (x < 0) { x = 0; }
      if (x > max_x) { x = max_x; }
      y = center_y + (j - half_window_size) * subpixel_factor;
      if (y < 0) { y = 0; }
      if (y > max_y) { y = max_y; }
      // pixel to the top left:
      x_0_or = (x / subpixel_factor);
      x_0 = x_0_or * subpixel_factor;
      y_0_or = (y / subpixel_factor);
      y_0 = y_0_or * subpixel_factor;


      if (x == x_0 && y == y_0) {
        ix2 = y_0_or * IMG_WIDTH + x_0_or;
        Y = (unsigned int)frame_buf[ix2 + 1];
        Patch[ix1] = (int) Y;
      } else {
        // blending according to how far the subpixel coordinates are from the pixel coordinates
        alpha_x = (x - x_0);
        alpha_y = (y - y_0);

        // the patch pixel is a blend from the four surrounding pixels:
        ix2 = y_0_or * IMG_WIDTH + x_0_or;
        Y = (unsigned int)frame_buf[ix2 + 1];
        Patch[ix1] = (subpixel_factor - alpha_x) * (subpixel_factor - alpha_y) * ((int) Y);

        ix2 = y_0_or * IMG_WIDTH + (x_0_or + 1);
        Y = (unsigned int)frame_buf[ix2 + 1];
        Patch[ix1] += alpha_x * (subpixel_factor - alpha_y) * ((int) Y);

        ix2 = (y_0_or + 1) * IMG_WIDTH + x_0_or;
        Y = (unsigned int)frame_buf[ix2 + 1];
        Patch[ix1] += (subpixel_factor - alpha_x) * alpha_y * ((int) Y);

        ix2 = (y_0_or + 1) * IMG_WIDTH + (x_0_or + 1);
        Y = (unsigned int)frame_buf[ix2 + 1];
        Patch[ix1] += alpha_x * alpha_y * ((int) Y);

        // normalize patch value
        Patch[ix1] /= (subpixel_factor * subpixel_factor);
      }
    }
  }

  return;
}

void getGradientPatch(int *Patch, int *DX, int *DY, int half_window_size)
{
  unsigned int ix1, ix2;
  int x, y, padded_patch_size, patch_size, Y1, Y2;
  //  int printed; printed = 0;

  padded_patch_size = 2 * (half_window_size + 1) + 1;
  patch_size = 2 * half_window_size + 1;
  // currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
  for (x = 1; x < padded_patch_size - 1; x++) {
    for (y = 1; y < padded_patch_size - 1; y++) {
      // index in DX, DY:
      ix2 = (unsigned int)((y - 1) * patch_size + (x - 1));

      ix1 = (unsigned int)(y * padded_patch_size + x - 1);
      Y1 = Patch[ix1];
      ix1 = (unsigned int)(y * padded_patch_size + x + 1);
      Y2 = Patch[ix1];
      DX[ix2] = (Y2 - Y1) / 2;

      ix1 = (unsigned int)((y - 1) * padded_patch_size + x);
      Y1 = Patch[ix1];
      ix1 = (unsigned int)((y + 1) * padded_patch_size + x);
      Y2 = Patch[ix1];
      DY[ix2] = (Y2 - Y1) / 2;


    }
  }

  return;
}

int getSumPatch(int *Patch, int size)
{
  int x, y, sum; // , threshold
  unsigned int ix;

  // in order to keep the sum within range:
  //threshold = 50000; // typical values are far below this threshold
  sum = 0;
  for (x = 0; x < size; x++) {
    for (y = 0; y < size; y++) {
      ix = (y * size) + x;
      sum += Patch[ix]; // do not check thresholds
    }
  }

  return sum;
}

int calculateG(int *G, int *DX, int *DY, int half_window_size)
{
  int patch_size;
  int *DXX; int *DXY; int *DYY;

  patch_size = 2 * half_window_size + 1;

  // allocate memory:
  DXX = (int *) malloc(patch_size * patch_size * sizeof(int));
  DXY = (int *) malloc(patch_size * patch_size * sizeof(int));
  DYY = (int *) malloc(patch_size * patch_size * sizeof(int));

  if (DXX == 0 || DXY == 0 || DYY == 0) {
    return NO_MEMORY;
  }

  // then determine the second order gradients
  multiplyImages(DX, DX, DXX, patch_size, patch_size);
  multiplyImages(DX, DY, DXY, patch_size, patch_size);
  multiplyImages(DY, DY, DYY, patch_size, patch_size);

  // calculate G:
  G[0] = getSumPatch(DXX, patch_size);
  G[1] = getSumPatch(DXY, patch_size);
  G[2] = G[1];
  G[3] = getSumPatch(DYY, patch_size);

  // free memory:
  free((char *) DXX); free((char *) DXY); free((char *) DYY);

  // no errors:
  return OK;
}



int calculateError(int *ImC, int width, int height)
{
  int x, y, error;
  unsigned int ix;

  error = 0;

  for (x = 0; x < width; x++) {
    for (y = 0; y < height; y++) {
      ix = (y * width + x);
      error += ImC[ix] * ImC[ix];
    }
  }

  return error;
}

int opticFlowLK(unsigned char *new_image_buf, unsigned char *old_image_buf, int *p_x, int *p_y, int n_found_points,
                int imW, int imH, int *new_x, int *new_y, int *status, int half_window_size, int max_iterations)
{
  // A straightforward one-level implementation of Lucas-Kanade.
  // For all points:
  // (1) determine the subpixel neighborhood in the old image
  // (2) get the x- and y- gradients
  // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
  // (4) iterate over taking steps in the image to minimize the error:
  //     [a] get the subpixel neighborhood in the new image
  //     [b] determine the image difference between the two neighborhoods
  //     [c] calculate the 'b'-vector
  //     [d] calculate the additional flow step and possibly terminate the iteration
  int p, subpixel_factor, x, y, it, step_threshold, step_x, step_y, v_x, v_y, Det;
  int b_x, b_y, patch_size, padded_patch_size, error;
  unsigned int ix1, ix2;
  int *I_padded_neighborhood; int *I_neighborhood; int *J_neighborhood;
  int *DX; int *DY; int *ImDiff; int *IDDX; int *IDDY;
  int G[4];
  int error_threshold;

  // set the image width and height
  IMG_WIDTH = imW;
  IMG_HEIGHT = imH;
  // spatial resolution of flow is 1 / subpixel_factor
  subpixel_factor = 10;
  // determine patch sizes and initialize neighborhoods
  patch_size = (2 * half_window_size + 1);
  error_threshold = (25 * 25) * (patch_size * patch_size);

  padded_patch_size = (2 * half_window_size + 3);
  I_padded_neighborhood = (int *) malloc(padded_patch_size * padded_patch_size * sizeof(int));
  I_neighborhood = (int *) malloc(patch_size * patch_size * sizeof(int));
  J_neighborhood = (int *) malloc(patch_size * patch_size * sizeof(int));
  if (I_padded_neighborhood == 0 || I_neighborhood == 0 || J_neighborhood == 0) {
    return NO_MEMORY;
  }

  DX = (int *) malloc(patch_size * patch_size * sizeof(int));
  DY = (int *) malloc(patch_size * patch_size * sizeof(int));
  IDDX = (int *) malloc(patch_size * patch_size * sizeof(int));
  IDDY = (int *) malloc(patch_size * patch_size * sizeof(int));
  ImDiff = (int *) malloc(patch_size * patch_size * sizeof(int));
  if (DX == 0 || DY == 0 || ImDiff == 0 || IDDX == 0 || IDDY == 0) {
    return NO_MEMORY;
  }

  for (p = 0; p < n_found_points; p++) {
    // status: point is not yet lost:
    status[p] = 1;

    // We want to be able to take steps in the image of 1 / subpixel_factor:
    p_x[p] *= subpixel_factor;
    p_y[p] *= subpixel_factor;

    // if the pixel is outside the ROI in the image, do not track it:
    if (!(p_x[p] > ((half_window_size + 1) * subpixel_factor) && p_x[p] < (IMG_WIDTH - half_window_size) * subpixel_factor
          && p_y[p] > ((half_window_size + 1) * subpixel_factor) && p_y[p] < (IMG_HEIGHT - half_window_size)*subpixel_factor)) {
      status[p] = 0;
    }

    // (1) determine the subpixel neighborhood in the old image
    // we determine a padded neighborhood with the aim of subsequent gradient processing:
    getSubPixel_gray(I_padded_neighborhood, old_image_buf, p_x[p], p_y[p], half_window_size + 1, subpixel_factor);

    // Also get the original-sized neighborhood
    for (x = 1; x < padded_patch_size - 1; x++) {
      for (y = 1; y < padded_patch_size - 1; y++) {
        ix1 = (y * padded_patch_size + x);
        ix2 = ((y - 1) * patch_size + (x - 1));
        I_neighborhood[ix2] = I_padded_neighborhood[ix1];
      }
    }

    // (2) get the x- and y- gradients
    getGradientPatch(I_padded_neighborhood, DX, DY, half_window_size);

    // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
    error = calculateG(G, DX, DY, half_window_size);
    if (error == NO_MEMORY) { return NO_MEMORY; }

    for (it = 0; it < 4; it++) {
      G[it] /= 255; // to keep values in range
    }
    // calculate G's determinant:
    Det = G[0] * G[3] - G[1] * G[2];
    Det = Det / subpixel_factor; // so that the steps will be expressed in subpixel units
    if (Det < 1) {
      status[p] = 0;
    }

    // (4) iterate over taking steps in the image to minimize the error:
    it = 0;
    step_threshold = 2; // 0.2 as smallest step (L1)
    v_x = 0;
    v_y = 0;
    step_x = step_threshold + 1;
    step_y = step_threshold + 1;

    while (status[p] == 1 && it < max_iterations && (abs(step_x) >= step_threshold || abs(step_y) >= step_threshold)) {
      // if the pixel goes outside the ROI in the image, stop tracking:
      if (!(p_x[p] + v_x > ((half_window_size + 1) * subpixel_factor)
            && p_x[p] + v_x < ((int)IMG_WIDTH - half_window_size) * subpixel_factor
            && p_y[p] + v_y > ((half_window_size + 1) * subpixel_factor)
            && p_y[p] + v_y < ((int)IMG_HEIGHT - half_window_size)*subpixel_factor)) {
        status[p] = 0;
        break;
      }

      //     [a] get the subpixel neighborhood in the new image
      // clear J:
      for (x = 0; x < patch_size; x++) {
        for (y = 0; y < patch_size; y++) {
          ix2 = (y * patch_size + x);
          J_neighborhood[ix2] = 0;
        }
      }


      getSubPixel_gray(J_neighborhood, new_image_buf, p_x[p] + v_x, p_y[p] + v_y, half_window_size, subpixel_factor);
      //     [b] determine the image difference between the two neighborhoods
      getImageDifference(I_neighborhood, J_neighborhood, ImDiff, patch_size, patch_size);
      error = calculateError(ImDiff, patch_size, patch_size) / 255;

      if (error > error_threshold && it > max_iterations / 2) {
        status[p] = 0;
        break;
      }
      multiplyImages(ImDiff, DX, IDDX, patch_size, patch_size);
      b_x = getSumPatch(IDDX, patch_size) / 255;
      b_y = getSumPatch(IDDY, patch_size) / 255;
      //printf("b_x = %d; b_y = %d;\n\r", b_x, b_y);
      //     [d] calculate the additional flow step and possibly terminate the iteration
      step_x = (G[3] * b_x - G[1] * b_y) / Det;
      step_y = (G[0] * b_y - G[2] * b_x) / Det;
      v_x += step_x;
      v_y += step_y; // - (?) since the origin in the image is in the top left of the image, with y positive pointing down
      // next iteration
      it++;
    } // iteration to find the right window in the new image

    new_x[p] = (p_x[p] + v_x) / subpixel_factor;
    new_y[p] = (p_y[p] + v_y) / subpixel_factor;
    p_x[p] /= subpixel_factor;
    p_y[p] /= subpixel_factor;
  }



  // free all allocated variables:
  free((int *) I_padded_neighborhood);
  free((int *) I_neighborhood);
  free((int *) J_neighborhood);
  free((int *) DX);
  free((int *) DY);
  free((int *) ImDiff);
  free((int *) IDDX);
  free((int *) IDDY);
  // no errors:
  return OK;
}

void quick_sort(float *a, int n)
{
  if (n < 2) {
    return;
  }
  float p = a[n / 2];
  float *l = a;
  float *r = a + n - 1;
  while (l <= r) {
    if (*l < p) {
      l++;
      continue;
    }
    if (*r > p) {
      r--;
      continue; // we need to check the condition (l <= r) every time we change the value of l or r
    }
    float t = *l;
    *l++ = *r;
    *r-- = t;
  }
  quick_sort(a, r - a + 1);
  quick_sort(l, a + n - l);
}

void quick_sort_int(int *a, int n)
{
  if (n < 2) {
    return;
  }
  int p = a[n / 2];
  int *l = a;
  int *r = a + n - 1;
  while (l <= r) {
    if (*l < p) {
      l++;
      continue;
    }
    if (*r > p) {
      r--;
      continue;
    }
    int t = *l;
    *l++ = *r;
    *r-- = t;
  }
  quick_sort_int(a, r - a + 1);
  quick_sort_int(l, a + n - l);
}

void CvtYUYV2Gray(unsigned char *grayframe, unsigned char *frame, int imW, int imH)
{
  int x, y;
  unsigned char *Y, *gray;
  for (y = 0; y < imH; y++) {
    Y = frame + (imW * 2 * y) + 1;
    gray = grayframe + (imW * y);
    for (x = 0; x < imW; x += 2) {
      gray[x] = *Y;
      Y += 2;
      gray[x + 1] = *Y;
      Y += 2;
    }
  }
}

unsigned int OF_buf_point = 0;
unsigned int OF_buf_point2 = 0;
float x_avg, y_avg, x_buf[24], y_buf[24], x_buf2[24], y_buf2[24];

void OFfilter(float *OFx, float *OFy, float dx, float dy, int count, int OF_FilterType)
{

  if (OF_FilterType == 1) { //1. moving average 2. moving median

    x_avg = 0.0;
    y_avg = 0.0;

    if (count) {
      x_buf[OF_buf_point] = dx;
      y_buf[OF_buf_point] = dy;
    } else {
      x_buf[OF_buf_point] = 0.0;
      y_buf[OF_buf_point] = 0.0;
    }
    OF_buf_point = (OF_buf_point + 1) % 20;

    for (int i = 0; i < 20; i++) {
      x_avg += x_buf[i] * 0.05;
      y_avg += y_buf[i] * 0.05;
    }

    *OFx = x_avg;
    *OFy = y_avg;

  } else if (OF_FilterType == 2) {
    if (count) {
      x_buf2[OF_buf_point2] = dx;
      y_buf2[OF_buf_point2] = dy;
    } else {
      x_buf2[OF_buf_point2] = 0.0;
      y_buf2[OF_buf_point2] = 0.0;
    }
    OF_buf_point2 = (OF_buf_point2 + 1) % 11; // 11

    quick_sort(x_buf2, 11); // 11
    quick_sort(y_buf2, 11); // 11

    *OFx = x_buf2[6]; // 6
    *OFy = y_buf2[6]; // 6
  } else {
    printf("no filter type selected!\n");
  }
}


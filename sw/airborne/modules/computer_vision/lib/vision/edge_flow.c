/*
 * edge_flow.c
 *
 *  Created on: Feb 22, 2016
 *      Author: knmcguire
 */
#include <lib/vision/edge_flow.h>


/**
 * Calculate a edge/gradient histogram for each dimension of the image
 * @param[in] *img  The image frame to calculate the edge histogram from
 * @param[out] *edge_histogram  The edge histogram from the current frame_step
 * @param[in] direction  Indicating if the histogram is made in either x or y direction
 * @param[in] edge_threshold  A threshold if a gradient is considered a edge or not
 */
void calculate_edge_histogram(struct image_t *img, int32_t edge_histogram[],
                              char direction, uint16_t edge_threshold)
{
  uint8_t *img_buf = (uint8_t *)img->buf;

  // TODO use arm_conv_q31()
  int32_t sobel_sum = 0;
  int32_t Sobel[3] = { -1, 0, 1};

  uint32_t y = 0, x = 0;
  int32_t c = 0;

  uint32_t idx = 0;

  uint16_t image_width = img->w;
  uint16_t image_height = img->h;
  uint32_t interlace;
  if (img->type == IMAGE_GRAYSCALE) {
    interlace = 1;
  } else {
    if (img->type == IMAGE_YUV422) {
      interlace = 2;
    } else
      while (1);   // hang to show user something isn't right
  }


  // compute edge histogram
  if (direction == 'x') {
    // set values that are not visited
    edge_histogram[0] = edge_histogram[image_width - 1] = 0;
    for (x = 1; x < image_width - 1; x++) {
      edge_histogram[x] = 0;
      for (y = 0; y < image_height; y++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = interlace * (image_width * y + (x + c)); // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx + 1];
        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold) {
          edge_histogram[x] += sobel_sum;
        }
      }
    }
  } else if (direction == 'y') {
    // set values that are not visited
    edge_histogram[0] = edge_histogram[image_height - 1] = 0;
    for (y = 1; y < image_height - 1; y++) {
      edge_histogram[y] = 0;
      for (x = 0; x < image_width; x++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = interlace * (image_width * (y + c) + x); // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx + 1];
        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold) {
          edge_histogram[y] += sobel_sum;
        }
      }
    }
  } else
    while (1);  // hang to show user something isn't right
}

/**
 * Calculate_displacement calculates the displacement between two histograms
 * @param[in] *edge_histogram  The edge histogram from the current frame_step
 * @param[in] *edge_histogram_prev  The edge histogram from the previous frame_step
 * @param[in] *displacement array with pixel displacement of the sequential edge histograms
 * @param[in] size  Indicating the size of the displacement array
 * @param[in] window Indicating the search window size
 * @param[in] disp_range  Indicating the maximum disparity range for the block matching
 * @param[in] der_shift  The pixel shift estimated by the angle rate of the IMU
 */
void calculate_edge_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                                 uint16_t size,
                                 uint8_t window, uint8_t disp_range, int32_t der_shift)
{
  int32_t c = 0, r = 0;
  uint32_t x = 0;
  uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1

  int32_t W = window;
  int32_t D = disp_range;


  uint8_t SHIFT_TOO_FAR = 0;
  memset(displacement, 0, size);

  int32_t border[2];

  if (der_shift < 0) {
    border[0] =  W + D + der_shift;
    border[1] = size - W - D;
  } else if (der_shift > 0) {
    border[0] =  W + D;
    border[1] = size - W - D - der_shift;
  } else {
    border[0] =  W + D;
    border[1] = size - W - D;
  }

  if (border[0] >= border[1] || abs(der_shift) >= 10) {
    SHIFT_TOO_FAR = 1;
  }
  {
    // TODO: replace with arm offset subtract
    for (x = border[0]; x < border[1]; x++) {
      displacement[x] = 0;
      for (c = -D; c <= D; c++) {
        SAD_temp[c + D] = 0;
        for (r = -W; r <= W; r++) {
          SAD_temp[c + D] += abs(edge_histogram[x + r] - edge_histogram_prev[x + r + c + der_shift]);
        }
      }
      if (!SHIFT_TOO_FAR) {
        displacement[x] = (int32_t)getMinimum(SAD_temp, 2 * D + 1) - D;
      } else {
        displacement[x] = 0;
      }
    }
  }

}

/**
 * Calculate minimum of an array
 * @param[in] *a Array containing values
 * @param[in] *n The size of the array
 * @return The index of the smallest value of the array
 */
uint32_t getMinimum(uint32_t *a, uint32_t n)
{
  uint32_t i;
  uint32_t min_ind = 0;
  uint32_t min_err = a[min_ind];
  uint32_t min_err_tot = 0;
  for (i = 1; i < n; i++) {
    if (a[i] <= min_err) {
      min_ind = i;
      min_err = a[i];
      min_err_tot += min_err;
    }
  }
  //*min_error = min_err_tot;
  return min_ind;
}


/**
 * Fits a linear model to an array with pixel displacements with least squares
 * @param[in] *displacements Array with Pixel Displacements
 * @param[out] *divergence Global divergence of pixel displacements
 * @param[out] *flow  Global translational flow from pixel displacements
 * @param[in] *size Size of displacement array
 * @param[in] border  A border offset of the array that should not be considerd for the line fit
 * @param[in] RES  Resolution to be used for the integer based linefit
 */
void line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
              uint16_t RES)
{
  int32_t x;

  int32_t count = 0;
  int32_t sumY = 0;
  int32_t sumX = 0;
  int32_t sumX2 = 0;
  int32_t sumXY = 0;
  int32_t xMean = 0;
  int32_t yMean = 0;
  int32_t divergence_int = 0;
  int32_t border_int = (int32_t)border;
  int32_t size_int = (int32_t)size;
  uint32_t total_error = 0;

  *divergence = 0;
  *flow = 0;

  // compute fixed sums
  int32_t xend = size_int - border_int - 1;
  sumX = xend * (xend + 1) / 2 - border_int * (border_int + 1) / 2 + border_int;
  sumX2 = xend * (xend + 1) * (2 * xend + 1) / 6;
  xMean = (size_int - 1) / 2;
  count = size_int - 2 * border_int;

  for (x = border_int; x < size - border_int; x++) {
    sumY += displacement[x];
    sumXY += x * displacement[x];
  }

  yMean = RES * sumY / count;

  divergence_int = (RES * sumXY - sumX * yMean) / (sumX2 - sumX * xMean);    // compute slope of line ax + b
  *divergence = divergence_int;
  *flow = yMean - *divergence * xMean;  // compute b (or y) intercept of line ax + b

  for (x = border_int; x < size - border_int; x++) {
    total_error += abs(RES * displacement[x] - divergence_int * x + yMean);
  }
}

/**
 * Draws edgehistogram, displacement and linefit directly on the image for debugging (only for edgeflow in horizontal direction!!)
 * @param[out] *img The image structure where will be drawn on
 * @param[in] edgeflow Information structure for flow information
 * @param[in] Displacement Pixel wise Displacement array
 * @param[in] *edge_hist_x Horizontal edge_histogram
 */
void draw_edgeflow_img(struct image_t *img, struct edge_flow_t edgeflow, struct edgeflow_displacement_t displacement,
                       int32_t *edge_hist_x)
{
  struct point_t point1;
  struct point_t point2;
  struct point_t point1_prev;
  struct point_t point2_prev;
  struct point_t point1_extra;
  struct point_t point2_extra;
  uint16_t i;

  for (i = 120; i < 240; i++) {
    point1.y = -(uint16_t)edge_hist_x[i] / 100 + img->h / 3;
    point1.x = i;
    point2.y = -(uint16_t)edge_hist_x[i + 1] / 100 + img->h / 3;
    point2.x = i + 1;

    point1_prev.y = -(uint16_t)displacement.x[i] * 5 + img->h * 2 / 3;
    point1_prev.x = i;
    point2_prev.y = -(uint16_t)displacement.x[i + 1] * 5 + img->h * 2 / 3;
    point2_prev.x = i + 1;

    image_draw_line(img, &point1, &point2);
    image_draw_line(img, &point1_prev, &point2_prev);
  }

  point1_extra.y = (edgeflow.flow_x + edgeflow.div_x * -180) / 100 + img->h / 2;
  point1_extra.x = 0;
  point2_extra.y = (edgeflow.flow_x + edgeflow.div_x * 180) / 100 + img->h / 2;
  point2_extra.x = 360;
  image_draw_line(img, &point1_extra, &point2_extra);
}

/**
 * getAmountPeaks, calculates the amount of peaks in a edge histogram
 * @param[in] *edgehist Horizontal edge_histogram
 * @param[in] thres The threshold from which a peak is considered significant peak or not
 * @param[in] size  Size of the array
 * @param[return] amount of peaks
 */
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t thres, int32_t size)
{
  uint32_t  amountPeaks = 0;
  uint32_t i = 0;

  for (i = 1; i < size + 1;  i ++) {
    if (edgehist[i - 1] < edgehist[i] && edgehist[i] > edgehist[i + 1] && edgehist[i] > thres) {
      amountPeaks ++;
    }
  }
  return amountPeaks;
}

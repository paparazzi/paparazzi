/*
 * edge_flow.c
 *
 *  Created on: Feb 22, 2016
 *      Author: knmcguire
 */
#include <opticflow/edge_flow.h>

// Local functions of the EDGEFLOW algorithm
void draw_edgeflow_img(struct image_t *img, struct edge_flow_t edgeflow, struct edgeflow_displacement_t displacement,
                       int32_t *edge_hist_x);
void calculate_edge_histogram(struct image_t *img, int32_t edge_histogram[],
                              char direction, uint16_t edge_threshold);
void calculate_edge_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                                 uint16_t size,
                                 uint8_t window, uint8_t disp_range, int32_t der_shift);

// Local assisting functions (only used here)
// TODO: find a way to incorperate/find these functions in paparazzi
static uint32_t timeval_diff2(struct timeval *starttime, struct timeval *finishtime);
static uint32_t getMinimum(uint32_t *a, uint32_t n);
void line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
              uint16_t RES);
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size);
/**
 * Run the optical flow with EDGEFLOW on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */


void edgeflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img,
                         struct opticflow_result_t *result)
{
  // Define Static Variables
  static struct edge_hist_t edge_hist[MAX_HORIZON];
  static uint8_t current_frame_nr = 0;
  static struct edge_flow_t edgeflow;
  static uint8_t previous_frame_offset[2] = {1, 1};

  // Define Normal variables
  struct edgeflow_displacement_t displacement;
  uint16_t disp_range;
  if (opticflow->search_distance < DISP_RANGE_MAX) {
    disp_range = opticflow->search_distance;
  } else {
    disp_range = DISP_RANGE_MAX;
  }

  uint16_t window_size;

  if (opticflow->window_size < MAX_WINDOW_SIZE) {
    window_size = opticflow->window_size;
  } else {
    window_size = MAX_WINDOW_SIZE;
  }

  uint16_t RES = opticflow->subpixel_factor;

  //......................Calculating EdgeFlow..................... //

  // Calculate current frame's edge histogram
  int32_t *edge_hist_x = edge_hist[current_frame_nr].x;
  int32_t *edge_hist_y = edge_hist[current_frame_nr].y;
  calculate_edge_histogram(img, edge_hist_x, 'x', 0);
  calculate_edge_histogram(img, edge_hist_y, 'y', 0);

  // Copy frame time and angles of image to calculated edge histogram
  memcpy(&edge_hist[current_frame_nr].frame_time, &img->ts, sizeof(struct timeval));
  edge_hist[current_frame_nr].pitch = state->theta;
  edge_hist[current_frame_nr].roll = state->phi;

  // Adaptive Time Horizon:
  // if the flow measured in previous frame is small,
  // the algorithm will choose an frame further away back from the
  // current frame to detect subpixel flow
  if (MAX_HORIZON > 2) {

    uint32_t flow_mag_x, flow_mag_y;
    flow_mag_x = abs(edgeflow.flow_x);
    flow_mag_y = abs(edgeflow.flow_y);
    uint32_t min_flow = 3;
    uint32_t max_flow = disp_range * RES - 3 * RES;

    uint8_t previous_frame_offset_x = previous_frame_offset[0];
    uint8_t previous_frame_offset_y = previous_frame_offset[1];

    // IF statements which will decrement the previous frame offset
    // if the measured flow of last loop is higher than max value (higher flow measured)
    // and visa versa
    if (flow_mag_x > max_flow && previous_frame_offset_x > 1) {
      previous_frame_offset[0] = previous_frame_offset_x - 1;
    }
    if (flow_mag_x < min_flow && previous_frame_offset_x < MAX_HORIZON - 1) {
      previous_frame_offset[0] = previous_frame_offset_x + 1;
    }
    if (flow_mag_y > max_flow && previous_frame_offset_y > 1) {
      previous_frame_offset[1] = previous_frame_offset_y - 1;
    }
    if (flow_mag_y < min_flow && previous_frame_offset_y < MAX_HORIZON - 1) {
      previous_frame_offset[1] = previous_frame_offset_y + 1;
    }
  }

  //Wrap index previous frame offset from current frame nr.
  uint8_t previous_frame_x = (current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
                             MAX_HORIZON;
  uint8_t previous_frame_y = (current_frame_nr - previous_frame_offset[1] + MAX_HORIZON) %
                             MAX_HORIZON;

  //Select edge histogram from the previous frame nr
  int32_t *prev_edge_histogram_x = edge_hist[previous_frame_x].x;
  int32_t *prev_edge_histogram_y = edge_hist[previous_frame_y].y;

  //Calculate the corrosponding derotation of the two frames
  int16_t der_shift_x = -(int16_t)((edge_hist[previous_frame_x].roll - edge_hist[current_frame_nr].roll) *
                                   (float)img->w / (OPTICFLOW_FOV_W));
  int16_t der_shift_y = -(int16_t)((edge_hist[previous_frame_x].pitch - edge_hist[current_frame_nr].pitch) *
                                   (float)img->h / (OPTICFLOW_FOV_H));

  // Estimate pixel wise displacement of the edge histograms for x and y direction
  calculate_edge_displacement(edge_hist_x, prev_edge_histogram_x,
                              displacement.x, img->w,
                              window_size, disp_range,  der_shift_x);
  calculate_edge_displacement(edge_hist_y, prev_edge_histogram_y,
                              displacement.y, img->h,
                              window_size, disp_range, der_shift_y);

  // Fit a line on the pixel displacement to estimate
  // the global pixel flow and divergence (RES is resolution)
  line_fit(displacement.x, &edgeflow.div_x,
           &edgeflow.flow_x, img->w,
           window_size + disp_range, RES);
  line_fit(displacement.y, &edgeflow.div_y,
           &edgeflow.flow_y, img->h,
           window_size + disp_range, RES);

  /* Save Resulting flow in results
   * Warning: The flow detected here is different in sign
   * and size, therefore this will be multiplied with
   * the same subpixel factor and -1 to make it on par with
   * the LK algorithm of t opticalflow_calculator.c
   * */
  edgeflow.flow_x = -1 * edgeflow.flow_x;
  edgeflow.flow_y = -1 * edgeflow.flow_y;

  result->flow_x = (int16_t)edgeflow.flow_x / previous_frame_offset[0];
  result->flow_y = (int16_t)edgeflow.flow_y / previous_frame_offset[1];

  //Fill up the results optic flow to be on par with LK_fast9
  result->flow_der_x =  result->flow_x;
  result->flow_der_y =  result->flow_y;
  result->corner_cnt = getAmountPeaks(edge_hist_x, 500 , img->w);
  result->tracked_cnt = getAmountPeaks(edge_hist_x, 500 , img->w);
  result->divergence = (float)edgeflow.flow_x / RES;
  result->div_size = 0.0f;
  result->noise_measurement = 0.0f;
  result->surface_roughness = 0.0f;

  //......................Calculating VELOCITY ..................... //

  /*Estimate fps per direction
   * This is the fps with adaptive horizon for subpixel flow, which is not similar
   * to the loop speed of the algorithm. The faster the quadcopter flies
   * the higher it becomes
  */
  float fps_x = 0;
  float fps_y = 0;
  float time_diff_x = (float)(timeval_diff2(&edge_hist[previous_frame_x].frame_time, &img->ts)) / 1000.;
  float time_diff_y = (float)(timeval_diff2(&edge_hist[previous_frame_y].frame_time, &img->ts)) / 1000.;
  fps_x = 1 / (time_diff_x);
  fps_y = 1 / (time_diff_y);

  result->fps = fps_x;

  // Calculate velocity
  float vel_x = edgeflow.flow_x * fps_x * state->agl * OPTICFLOW_FOV_W / (img->w * RES);
  float vel_y = edgeflow.flow_y * fps_y * state->agl * OPTICFLOW_FOV_H / (img->h * RES);
  result->vel_x = vel_x;
  result->vel_y = vel_y;

  /* Rotate velocities from camera frame coordinates to body coordinates.
  * IMPORTANT This frame to body orientation should bethe case for the parrot
  * ARdrone and Bebop, however this can be different for other quadcopters
  * ALWAYS double check!
  */
  result->vel_body_x = - vel_y;
  result->vel_body_y = vel_x;

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_FLOW
  draw_edgeflow_img(img, edgeflow, displacement, *edge_hist_x)
#endif
  // Increment and wrap current time frame
  current_frame_nr = (current_frame_nr + 1) % MAX_HORIZON;
}

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
static uint32_t getMinimum(uint32_t *a, uint32_t n)
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
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 */
static uint32_t timeval_diff2(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
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

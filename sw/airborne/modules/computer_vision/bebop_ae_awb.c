/*
 * Copyright (C) Freek van Tienen
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/bebop_ae_awb.c"
 * @author Freek van Tienen, Kirk Scheper
 * Auto exposure and Auto white balancing for the Bebop 1 and 2
 */

#include "bebop_ae_awb.h"
#include "boards/bebop.h"
#include "boards/bebop/mt9f002.h"
#include "lib/isp/libisp.h"

#ifndef CV_AE_AWB_CAMERA
#define CV_AE_AWB_CAMERA front_camera
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_CAMERA)

#ifndef CV_AE_AWB_VERBOSE
#define CV_AE_AWB_VERBOSE 0
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_VERBOSE)

#define PRINT(string,...) fprintf(stderr, "[cv_ae_awb->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if CV_AE_AWB_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef CV_AE_AWB_MIN_GAINS
#define CV_AE_AWB_MIN_GAINS 1.0
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_MIN_GAINS)

#ifndef CV_AE_AWB_MAX_GAINS
#define CV_AE_AWB_MAX_GAINS 63.5
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_MAX_GAINS)

#ifndef CV_AE_EXPOSURE_GAIN
#define CV_AE_EXPOSURE_GAIN 0.25
#endif

#ifndef CV_AE_MIDDLE_INDEX
#define CV_AE_MIDDLE_INDEX 90
#endif
PRINT_CONFIG_VAR(CV_AE_MIDDLE_INDEX)

#ifndef CV_AE_DARK_IGNORE
#define CV_AE_DARK_IGNORE 0.2
#endif
PRINT_CONFIG_VAR(CV_AE_DARK_IGNORE)

#ifndef CV_AE_BRIGHT_IGNORE
#define CV_AE_BRIGHT_IGNORE 0.6
#endif
PRINT_CONFIG_VAR(CV_AE_BRIGHT_IGNORE)

#ifndef CV_AE_DARK_BINS
#define CV_AE_DARK_BINS 73
#endif
PRINT_CONFIG_VAR(CV_AE_DARK_BINS)

#ifndef CV_AE_BRIGHT_BINS
#define CV_AE_BRIGHT_BINS 73
#endif
PRINT_CONFIG_VAR(CV_AE_BRIGHT_BINS)

#ifndef CV_AE_AWB_GAIN_SCHEDULING
#define CV_AE_AWB_GAIN_SCHEDULING 1
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_TARGET
#define CV_AE_AWB_GAIN_SCHEDULING_TARGET 10.0
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_TARGET)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE
#define CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE 7.5
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_STEP
#define CV_AE_AWB_GAIN_SCHEDULING_STEP 0.01
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_STEP)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_MARGIN
#define CV_AE_AWB_GAIN_SCHEDULING_MARGIN 1.5
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_MARGIN)

#ifndef CV_AE_AWB_GAIN
#define CV_AE_AWB_GAIN 1.0
#endif

#ifndef CV_AE_AWB_MIN_GREY_PIXELS
#define CV_AE_AWB_MIN_GREY_PIXELS 1000
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_MIN_GREY_PIXELS)

float   ae_exposure_gain = CV_AE_EXPOSURE_GAIN;
float   ae_bright_ignore = CV_AE_BRIGHT_IGNORE;
float   ae_dark_ignore   = CV_AE_DARK_IGNORE;
uint8_t ae_middle_index  = CV_AE_MIDDLE_INDEX;
float ae_awb_gain        = CV_AE_AWB_GAIN;

uint8_t ae_dark_bins     = CV_AE_DARK_BINS;
uint8_t ae_bright_bins   = CV_AE_BRIGHT_BINS;

float ae_current_level   = 0.0;
float awb_avgU           = 0.0;
float awb_avgV           = 0.0;
uint32_t awb_nb_pixels   = 0;


uint32_t histogram_plot[256];

int8_t gs_adjustDir      = 0;
bool gs_gains_maxed      = FALSE;
bool gs_gains_minned     = FALSE;

struct image_t *bebop_ae_awb_periodic(struct image_t *img);
struct image_t *bebop_ae_awb_periodic(struct image_t *img)
{
  static struct isp_yuv_stats_t yuv_stats;
  if (isp_get_statistics_yuv(&yuv_stats) == 0) {
    /*
     *      Auto-exposure (Histogram median centering)
     */
    static float prev_real_exposure = 0.0;
    static uint8_t prev_exposure_stuck = 0;
    // Calculate the cummulative histogram based on the histogram
    uint32_t cdf[MAX_HIST_Y];
    cdf[MIN_HIST_Y - 1] = 0;
    //printf("Histogram (%d)\n", yuv_stats.nb_valid_Y);
    histogram_plot[0] = yuv_stats.nb_valid_Y;
    for (int i = MIN_HIST_Y; i < MAX_HIST_Y; i++) {
      cdf[i] = cdf[i - 1] + yuv_stats.ae_histogram_Y[i];
      histogram_plot[i] = yuv_stats.ae_histogram_Y[i];
      //printf("%6d ",  yuv_stats.ae_histogram_Y[i]);
    }
    // Calculate the indices of the dark and bright bins
    uint8_t dark_index   = MIN_HIST_Y + ae_dark_bins;
    uint8_t bright_index = MAX_HIST_Y - ae_bright_bins;
    // Calculate the median number of pixels ignoring the dark_ignore % and bright-ignore %
    uint32_t median_pixels = (uint32_t) round(((1 - ae_dark_ignore) * cdf[dark_index] +
        (cdf[bright_index - 1] - cdf[dark_index]) + (1 - ae_bright_ignore) * (cdf[MAX_HIST_Y - 1] - cdf[bright_index - 1])) /
        2.0f);
    //printf("\nL: %d  M: %d  D: %d  =  %d\n", cdf[dark_index], (cdf[bright_index - 1] - cdf[dark_index]), (cdf[MAX_HIST_Y - 1] - cdf[bright_index - 1]), cdf[dark_index] + (cdf[bright_index - 1] - cdf[dark_index]) + (cdf[MAX_HIST_Y - 1] - cdf[bright_index - 1]));
    // Find the level that contains the median
    uint32_t current_pixels = 0;
    ae_current_level = MIN_HIST_Y - 1.0;
    while (current_pixels < median_pixels && ae_current_level < MAX_HIST_Y) {
      ae_current_level++;
      if (ae_current_level <= dark_index) {
        // In dark bin - ignore dark_ignore %
        current_pixels += (1 - ae_dark_ignore) * yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level];
      } else if (ae_current_level >= bright_index) {
        // In bright bin - ignore bright_ignore %
        current_pixels += (1 - ae_bright_ignore) * yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level];
      } else {
        // In centre bin - 100%
        current_pixels += yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level];
      }
    }
    if (yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level] > 0) {
      // Calculate decimal level
      ae_current_level -= (current_pixels - median_pixels) / ((float) yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level]);
    }
    //printf("\nMedian %d / %d - %0.3f\n\n", current_pixels, median_pixels, ae_current_level);
    // that level is supposed to be 'middle_index'
    float adjustment = 1 + ae_exposure_gain * (ae_middle_index / ae_current_level - 1);
    Bound(adjustment, 1 / 16.0f, 16.0f);
    // Calculate exposure based on adjustment
    mt9f002.target_exposure = mt9f002.real_exposure * adjustment;
    Bound(mt9f002.target_exposure, 1 / 128.0f, 40.0f);
    // Sometimes the exposure seems to freeze and it isn't updated. Try this hack to fix it
    // 10 consecutive YUV statistics leading to the exact same real_exposure is highly unlikely
    if(prev_real_exposure == mt9f002.real_exposure && prev_exposure_stuck < 10){
      prev_exposure_stuck++;
    }else if(prev_real_exposure == mt9f002.real_exposure && prev_exposure_stuck >= 10){
      // Bump the target exposure in order to unfreeze the exposure
      if(mt9f002.real_exposure < mt9f002.target_exposure){
        mt9f002.target_exposure *= 1.125;
      }else{
        mt9f002.target_exposure /= 1.125;
      }
    }else{
      prev_exposure_stuck = 0;
    }
    prev_real_exposure = mt9f002.real_exposure;
    mt9f002_set_exposure(&mt9f002);
    // Verbose prints
    VERBOSE_PRINT("AE lvl: target %d, actual %3.2f (%d pixels)\n", ae_middle_index, ae_current_level, yuv_stats.nb_valid_Y);
    VERBOSE_PRINT("AE exp: target %5.2f ms, real %5.2f ms (%f)\n", mt9f002.target_exposure, mt9f002.real_exposure,
        adjustment);
    /*
     *      Auto white-balance (Robust Automatic White Balance Algorithm using Gray Color Points in Images - Huo et al.)
     */
    awb_nb_pixels     = yuv_stats.awb_nb_grey_pixels;
    awb_avgU          = (yuv_stats.awb_sum_U) / ((float) yuv_stats.awb_nb_grey_pixels);
    awb_avgV          = (yuv_stats.awb_sum_V) / ((float) yuv_stats.awb_nb_grey_pixels);
    VERBOSE_PRINT("avgU = %d / %d - 128.0 = %f\n", yuv_stats.awb_sum_U, yuv_stats.awb_nb_grey_pixels, awb_avgU);
    VERBOSE_PRINT("avgV = %d / %d - 128.0 = %f\n", yuv_stats.awb_sum_V, yuv_stats.awb_nb_grey_pixels, awb_avgV);
    // |B| = |Y| + |U|  &&   |R| = |Y| + |V|
    // ideal:
    // |U| = |V| = 0  &&  |B| = |R| = |Y|
    // so:
    // gain_blue *= |Y| / (|Y| + |U|)  --> ^0.25 in order to make less agressive updates
    // gain_red  *= |Y| / (|Y| + |V|)  --> ^0.25 in order to make less agressive updates
    float blue_adj = (((float) (yuv_stats.awb_sum_Y)) / ((float)(yuv_stats.awb_sum_Y + yuv_stats.awb_sum_U - (128.0) *
        yuv_stats.awb_nb_grey_pixels)) - 1);
    mt9f002.gain_blue *= 1 + ae_awb_gain * blue_adj;
    if(mt9f002.gain_blue < mt9f002.gain_green1){
      mt9f002.gain_blue = mt9f002.gain_green1;
    }

    float red_adj = (((float) (yuv_stats.awb_sum_Y)) / ((float)(yuv_stats.awb_sum_Y + yuv_stats.awb_sum_V - (128.0) *
        yuv_stats.awb_nb_grey_pixels)) - 1);
    mt9f002.gain_red *= 1 + ae_awb_gain * red_adj;
    if(mt9f002.gain_red < mt9f002.gain_green1){
      mt9f002.gain_red = mt9f002.gain_green1;
    }
    /*
     *          Gain scheduling
     */
#if CV_AE_AWB_GAIN_SCHEDULING
    // Let's try to center the exposure, that way we'll be able to account for fast changes in lightness
    if (!gs_gains_maxed
        && mt9f002.target_exposure > (CV_AE_AWB_GAIN_SCHEDULING_TARGET + CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE)) {
      gs_gains_minned = FALSE;
      gs_adjustDir = +1;
    } else if (!gs_gains_minned
        && mt9f002.target_exposure < (CV_AE_AWB_GAIN_SCHEDULING_TARGET - CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE)) {
      gs_gains_maxed = FALSE;
      gs_adjustDir = -1;
    } else {
      gs_adjustDir = 0;
    }
    mt9f002.gain_blue     *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
    mt9f002.gain_red      *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
    mt9f002.gain_green1   *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
    mt9f002.gain_green2   *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
#endif
    Bound(mt9f002.gain_blue,    CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
    Bound(mt9f002.gain_red,     CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
    Bound(mt9f002.gain_green1,  CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
    Bound(mt9f002.gain_green2,  CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
    // Check to see if gains have reached max level
    if (mt9f002.gain_blue >= CV_AE_AWB_MAX_GAINS / CV_AE_AWB_GAIN_SCHEDULING_MARGIN
        || mt9f002.gain_red >= CV_AE_AWB_MAX_GAINS / CV_AE_AWB_GAIN_SCHEDULING_MARGIN
        || mt9f002.gain_green1 >= CV_AE_AWB_MAX_GAINS / CV_AE_AWB_GAIN_SCHEDULING_MARGIN) {
      gs_gains_maxed = TRUE;
    }
    // Check to see if gains have reached min level
    if (mt9f002.gain_blue <= CV_AE_AWB_MIN_GAINS * CV_AE_AWB_GAIN_SCHEDULING_MARGIN
        || mt9f002.gain_red <= CV_AE_AWB_MIN_GAINS * CV_AE_AWB_GAIN_SCHEDULING_MARGIN
        || mt9f002.gain_green1 <= CV_AE_AWB_MIN_GAINS * CV_AE_AWB_GAIN_SCHEDULING_MARGIN) {
      gs_gains_minned = TRUE;
    }
    if((mt9f002.gain_blue > 4 * mt9f002.gain_red || mt9f002.gain_blue > 4 * mt9f002.gain_green1) ||
       (mt9f002.gain_red > 4 * mt9f002.gain_blue || mt9f002.gain_red > 4 * mt9f002.gain_green1) ||
       yuv_stats.awb_nb_grey_pixels < CV_AE_AWB_MIN_GREY_PIXELS){
      cv_awb_reset();
    }else{
      // Set gains
      mt9f002_set_gains(&mt9f002);
    }
  } else {
    VERBOSE_PRINT("Error: no YUV stats\n");
  }
  return NULL;
}

void bebop_ae_awb_init(void)
{
  for (int i = 0; i < 256; i++) {
    histogram_plot[i] = 0.0;
  }
  cv_add_to_device(&CV_AE_AWB_CAMERA, cv_ae_awb_periodic);
}

void bebop_awb_reset(void){
  mt9f002.gain_red    = MT9F002_GAIN_RED;
  mt9f002.gain_blue   = MT9F002_GAIN_BLUE;
  mt9f002.gain_green1 = MT9F002_GAIN_GREEN1;
  mt9f002.gain_green2 = MT9F002_GAIN_GREEN2;
  // Set gains
  mt9f002_set_gains(&mt9f002);
}

void bebop_ae_reset(void){
  mt9f002.target_exposure = MT9F002_TARGET_EXPOSURE;
  mt9f002_set_exposure(&mt9f002);
}

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
#include "modules/computer_vision/cv.h"

#define MAX_HIST_Y 255
#define MIN_HIST_Y 1

#ifndef BEBOP_AE_AWB_CAMERA
#define BEBOP_AE_AWB_CAMERA front_camera
#endif
PRINT_CONFIG_VAR(BEBOP_AE_AWB_CAMERA)

#ifndef BEBOP_AE_AWB_VERBOSE
#define BEBOP_AE_AWB_VERBOSE 0
#endif
PRINT_CONFIG_VAR(BEBOP_AE_AWB_VERBOSE)

#define PRINT(string,...) fprintf(stderr, "[bebop_ae_awb->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if BEBOP_AE_AWB_VERBOSE
#include "stdio.h"
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// thread nice setting
#ifndef BEBOP_AE_AWB_NICE
#define BEBOP_AE_AWB_NICE 5
#endif

#ifndef BEBOP_AUTO_EXPOSURE
#define BEBOP_AUTO_EXPOSURE true
#endif

// Gain to apply to autoexposure change
#ifndef BEBOP_AE_EXPOSURE_GAIN
#define BEBOP_AE_EXPOSURE_GAIN 0.8
#endif
PRINT_CONFIG_VAR(BEBOP_AE_EXPOSURE_GAIN)

// Bin index to be considered as the center of the brightness bins
#ifndef BEBOP_AE_MIDDLE_INDEX
#define BEBOP_AE_MIDDLE_INDEX 180
#endif
PRINT_CONFIG_VAR(BEBOP_AE_MIDDLE_INDEX)

// Proportion of dark bins to ignore in the computation of the autoexposure
#ifndef BEBOP_AE_DARK_IGNORE
#define BEBOP_AE_DARK_IGNORE 0.4
#endif
PRINT_CONFIG_VAR(BEBOP_AE_DARK_IGNORE)

// Proportion of bright bins to ignore in the computation of the autoexposure
#ifndef BEBOP_AE_BRIGHT_IGNORE
#define BEBOP_AE_BRIGHT_IGNORE 0.2
#endif
PRINT_CONFIG_VAR(BEBOP_AE_BRIGHT_IGNORE)

// Number of dark bins to consider as dark in the autoexposure
#ifndef BEBOP_AE_DARK_BINS
#define BEBOP_AE_DARK_BINS 80
#endif
PRINT_CONFIG_VAR(BEBOP_AE_DARK_BINS)

// Number of bright bins to consider as bright in autoexposure
#ifndef BEBOP_AE_BRIGHT_BINS
#define BEBOP_AE_BRIGHT_BINS 20
#endif
PRINT_CONFIG_VAR(BEBOP_AE_BRIGHT_BINS)

#ifndef BEBOP_AUTO_WHITE_BALANCE
#define BEBOP_AUTO_WHITE_BALANCE true
#endif

// Minimum gain for AWB
#ifndef BEBOP_AWB_MIN_GAINS
#define BEBOP_AWB_MIN_GAINS 1.0
#endif
PRINT_CONFIG_VAR(BEBOP_AE_MIN_GAINS)

// Maximum gain for AWB
#ifndef BEBOP_AWB_MAX_GAINS
#define BEBOP_AWB_MAX_GAINS 63.5
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_MAX_GAINS)

//Gain to apply to AWB changes
#ifndef BEBOP_AWB_GAIN
#define BEBOP_AWB_GAIN 0.25
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_GAIN)

// Activates gain scheduling for the AWB, helps to handle fast changes in brightness
#ifndef BEBOP_AWB_GAIN_SCHEDULING
#define BEBOP_AWB_GAIN_SCHEDULING false
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_GAIN_SCHEDULING)

// Target white balance
#ifndef BEBOP_AWB_GAIN_SCHEDULING_TARGET
#define BEBOP_AWB_GAIN_SCHEDULING_TARGET 10.0
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_GAIN_SCHEDULING_TARGET)

// Tolerance around the target exposure outside of which gain scheduling is activated
#ifndef BEBOP_AWB_GAIN_SCHEDULING_TOLERANCE
#define BEBOP_AWB_GAIN_SCHEDULING_TOLERANCE 7.5
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_GAIN_SCHEDULING_TOLERANCE)

// Step size of the AWB gain scheduling correction
#ifndef BEBOP_AWB_GAIN_SCHEDULING_STEP
#define BEBOP_AWB_GAIN_SCHEDULING_STEP 0.02
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_GAIN_SCHEDULING_STEP)

// Minimum number of grey pixels in image, when lower, the AWB is reset
#ifndef BEBOP_AWB_MIN_GREY_PIXELS
#define BEBOP_AWB_MIN_GREY_PIXELS 1000
#endif
PRINT_CONFIG_VAR(BEBOP_AWB_MIN_GREY_PIXELS)

struct ae_setting_t ae_set;
struct awb_setting_t awb_set;

static void bebop_awb_reset(void)
{
  mt9f002.gain_red    = MT9F002_GAIN_RED;
  mt9f002.gain_blue   = MT9F002_GAIN_BLUE;
  mt9f002.gain_green1 = MT9F002_GAIN_GREEN1;
  mt9f002.gain_green2 = MT9F002_GAIN_GREEN2;
  // Set gains
  mt9f002_set_gains(&mt9f002);
}

static void bebop_ae_reset(void)
{
  mt9f002.target_exposure = MT9F002_TARGET_EXPOSURE;
  mt9f002_set_exposure(&mt9f002);
}

static struct image_t *update_ae_awb(struct image_t *img)
{
  static struct isp_yuv_stats_t yuv_stats;

  if (isp_get_statistics_yuv(&yuv_stats) == 0 && yuv_stats.nb_valid_Y > 1) {
    // get timing stats
    static uint32_t prev_time = 0;
    float dt = (img->pprz_ts - prev_time) * 1e-6f;
    Bound(dt, 0.f, 1.f);
    prev_time = img->pprz_ts;

    // handle any change in active setting
    if (ae_set.prev_active != ae_set.active) {
      bebop_ae_reset();
      ae_set.prev_active = ae_set.active;
    }

    if (awb_set.prev_active != awb_set.active) {
      bebop_awb_reset();
      awb_set.prev_active = awb_set.active;
    }

    /*
     *      Auto-exposure (Histogram median centering)
     */
    if (ae_set.active) {
      static float prev_real_exposure = 0.f;
      static uint8_t prev_exposure_stuck = 0;

      // Calculate the cummulative histogram based on the histogram
      static uint32_t cdf[MAX_HIST_Y];
      cdf[MIN_HIST_Y - 1] = 0;

      for (int i = MIN_HIST_Y; i < MAX_HIST_Y; i++) {
        cdf[i] = cdf[i - 1] + yuv_stats.ae_histogram_Y[i];
      }

      // Calculate the indices of the dark and bright bins
      uint8_t dark_index = MIN_HIST_Y + ae_set.dark_bins;
      if (dark_index > MAX_HIST_Y) {
        dark_index = MAX_HIST_Y;
      }
      uint8_t bright_index = MAX_HIST_Y - ae_set.bright_bins;
      if (bright_index < MIN_HIST_Y || bright_index > MAX_HIST_Y) {
        bright_index = dark_index;
      }

      // Calculate the median number of pixels ignoring the dark_ignore % and bright-ignore %
      uint32_t median_pixels = (uint32_t) roundf(((1.f - ae_set.dark_ignore) * cdf[dark_index] +
                               cdf[bright_index - 1] - cdf[dark_index] +
                               (1.f - ae_set.bright_ignore) * (cdf[MAX_HIST_Y - 1] - cdf[bright_index - 1]))
                               / 2.0f);

      // Find the level that contains the median
      uint32_t current_pixels = 0;
      uint8_t ae_current_ind = MIN_HIST_Y - 1;
      while (current_pixels < median_pixels && ae_current_ind < MAX_HIST_Y) {
        ae_current_ind++;
        if (ae_current_ind <= dark_index) {
          // In dark bin - ignore dark_ignore %
          current_pixels += (1 - ae_set.dark_ignore) * yuv_stats.ae_histogram_Y[ae_current_ind];
        } else if (ae_current_ind >= bright_index) {
          // In bright bin - ignore bright_ignore %
          current_pixels += (1 - ae_set.bright_ignore) * yuv_stats.ae_histogram_Y[ae_current_ind];
        } else {
          // In centre bin - 100%
          current_pixels += yuv_stats.ae_histogram_Y[ae_current_ind];
        }
      }

      float ae_current_level = (float)ae_current_ind;
      if (yuv_stats.ae_histogram_Y[ae_current_ind] > 0) {
        // Calculate decimal level
        ae_current_level -= (current_pixels - median_pixels) / (float)yuv_stats.ae_histogram_Y[ae_current_ind];
      }

      if (yuv_stats.awb_sum_Y > ae_current_level * yuv_stats.awb_nb_grey_pixels) {
        // too many saturated pixels so median no longer good estimate
        ae_current_level = (float)yuv_stats.awb_sum_Y / yuv_stats.awb_nb_grey_pixels;
      }

      // that level is supposed to be 'middle_index'
      float adjustment = 1.f;
      if (ae_current_level > 1e-5) {
        adjustment = 1.f + ae_set.exposure_gain * (ae_set.middle_index / ae_current_level - 1.f) * dt;
        Bound(adjustment, 1 / 16.f, 16.f);
      }

      // Calculate exposure based on adjustment
      mt9f002.target_exposure = mt9f002.real_exposure * adjustment;
      Bound(mt9f002.target_exposure, 1 / 128.f, 80.f);

      // Sometimes the exposure seems to freeze and it isn't updated. Try this hack to fix it
      // 10 consecutive YUV statistics leading to the exact same real_exposure is highly unlikely
      if (prev_real_exposure == mt9f002.real_exposure && prev_exposure_stuck < 10) {
        prev_exposure_stuck++;
      } else if (prev_real_exposure == mt9f002.real_exposure && prev_exposure_stuck >= 10) {
        // Bump the target exposure in order to unfreeze the exposure
        if (mt9f002.real_exposure < mt9f002.target_exposure) {
          mt9f002.target_exposure *= 1.125f;
        } else {
          mt9f002.target_exposure /= 1.125f;
        }
      } else {
        prev_exposure_stuck = 0;
      }
      prev_real_exposure = mt9f002.real_exposure;

      // Verbose prints
      VERBOSE_PRINT("AE lvl: target %d, actual %3.2f (%d pixels)\n", ae_set.middle_index, ae_current_level,
                    yuv_stats.nb_valid_Y);
      VERBOSE_PRINT("AE exp: target %5.2f ms, real %5.2f ms (%f)\n", mt9f002.target_exposure, mt9f002.real_exposure,
                    adjustment);

      mt9f002_set_exposure(&mt9f002);
    }

    /*
     *      Auto white-balance (Robust Automatic White Balance Algorithm using Gray Color Points in Images - Huo et al.)
     */
    if (awb_set.active) {
      if (yuv_stats.awb_nb_grey_pixels) {
        VERBOSE_PRINT("avgU = %d / %d = %d\n", yuv_stats.awb_sum_U, yuv_stats.awb_nb_grey_pixels,
                      yuv_stats.awb_sum_U / yuv_stats.awb_nb_grey_pixels);
        VERBOSE_PRINT("avgV = %d / %d = %d\n", yuv_stats.awb_sum_V, yuv_stats.awb_nb_grey_pixels,
                      yuv_stats.awb_sum_V / yuv_stats.awb_nb_grey_pixels);
      }
      // |B| = |Y| + |U|  &&   |R| = |Y| + |V|
      // ideal:
      // |U| = |V| = 0  &&  |B| = |R| = |Y|
      // so:
      // gain_blue *= |Y| / (|Y| + |U|)  --> ^0.25 in order to make less aggressive updates
      // gain_red  *= |Y| / (|Y| + |V|)  --> ^0.25 in order to make less aggressive updates
      float blue_adj = ((float)(yuv_stats.awb_sum_Y)) / ((float)(yuv_stats.awb_sum_Y + yuv_stats.awb_sum_U - 128.f *
                       yuv_stats.awb_nb_grey_pixels)) - 1.f;
      mt9f002.gain_blue *= 1 + awb_set.gain * blue_adj * dt;
      if (mt9f002.gain_blue < mt9f002.gain_green1) {
        mt9f002.gain_blue = mt9f002.gain_green1;
      }

      float red_adj = ((float)(yuv_stats.awb_sum_Y)) / ((float)(yuv_stats.awb_sum_Y + yuv_stats.awb_sum_V - 128.f *
                      yuv_stats.awb_nb_grey_pixels)) - 1.f;
      mt9f002.gain_red *= 1 + awb_set.gain * red_adj * dt;
      if (mt9f002.gain_red < mt9f002.gain_green1) {
        mt9f002.gain_red = mt9f002.gain_green1;
      }

      /*
       *          Gain scheduling
       */
      if (awb_set.gain_scheduling) {
        int8_t gs_adjustDir = 0;
        // Let's try to center the exposure, that way we'll be able to account for fast changes in brightness
        if (mt9f002.target_exposure > (awb_set.gain_scheduling_target + awb_set.gain_scheduling_tolerance)) {
          gs_adjustDir = +1;
        } else if (mt9f002.target_exposure < (awb_set.gain_scheduling_target - awb_set.gain_scheduling_tolerance)) {
          gs_adjustDir = -1;
        }
        mt9f002.gain_blue   *= 1 + gs_adjustDir * awb_set.gain_scheduling_step;
        mt9f002.gain_red    *= 1 + gs_adjustDir * awb_set.gain_scheduling_step;
        mt9f002.gain_green1 *= 1 + gs_adjustDir * awb_set.gain_scheduling_step;
        mt9f002.gain_green2 *= 1 + gs_adjustDir * awb_set.gain_scheduling_step;
      }

      // Bound gains
      Bound(mt9f002.gain_blue,   BEBOP_AWB_MIN_GAINS, BEBOP_AWB_MAX_GAINS);
      Bound(mt9f002.gain_red,    BEBOP_AWB_MIN_GAINS, BEBOP_AWB_MAX_GAINS);
      Bound(mt9f002.gain_green1, BEBOP_AWB_MIN_GAINS, BEBOP_AWB_MAX_GAINS);
      Bound(mt9f002.gain_green2, BEBOP_AWB_MIN_GAINS, BEBOP_AWB_MAX_GAINS);

      // check that gains have not exploded
      if ((mt9f002.gain_blue > 4 * mt9f002.gain_red || mt9f002.gain_blue > 4 * mt9f002.gain_green1) ||
          (mt9f002.gain_red > 4 * mt9f002.gain_blue || mt9f002.gain_red > 4 * mt9f002.gain_green1) ||
          yuv_stats.awb_nb_grey_pixels < BEBOP_AWB_MIN_GREY_PIXELS) {
        bebop_awb_reset();
      } else {
        VERBOSE_PRINT("blue %f, red %f, green 1 %f green 2 %f\n", mt9f002.gain_blue, mt9f002.gain_red, mt9f002.gain_green1,
                      mt9f002.gain_green2);
        // Set gains
        mt9f002_set_gains(&mt9f002);
      }
    }
  }

  return NULL;
}

void bebop_ae_awb_init(void)
{
  ae_set.active        = BEBOP_AUTO_EXPOSURE;
  ae_set.prev_active   = ae_set.active;
  ae_set.exposure_gain = BEBOP_AE_EXPOSURE_GAIN;
  ae_set.bright_ignore = BEBOP_AE_BRIGHT_IGNORE;
  ae_set.dark_ignore   = BEBOP_AE_DARK_IGNORE;
  ae_set.middle_index  = BEBOP_AE_MIDDLE_INDEX;
  ae_set.dark_bins     = BEBOP_AE_DARK_BINS;
  ae_set.bright_bins   = BEBOP_AE_BRIGHT_BINS;

  awb_set.active                    = BEBOP_AUTO_WHITE_BALANCE;
  awb_set.prev_active               = awb_set.active;
  awb_set.gain                      = BEBOP_AWB_GAIN;
  awb_set.gain_scheduling           = BEBOP_AWB_GAIN_SCHEDULING;
  awb_set.gain_scheduling_target    = BEBOP_AWB_GAIN_SCHEDULING_TARGET;
  awb_set.gain_scheduling_tolerance = BEBOP_AWB_GAIN_SCHEDULING_TOLERANCE;
  awb_set.gain_scheduling_step      = BEBOP_AWB_GAIN_SCHEDULING_STEP;

  cv_add_to_device_async(&BEBOP_AE_AWB_CAMERA, update_ae_awb, BEBOP_AE_AWB_NICE, 0);

  bebop_ae_reset();
  bebop_awb_reset();
}

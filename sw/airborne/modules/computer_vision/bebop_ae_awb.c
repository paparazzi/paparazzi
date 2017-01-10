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

#define MAX_HIST_Y 255

#define sgn(x) (float)((x < 0) ? -1 : (x > 0))

#ifndef BEBOP_AUTO_EXPOSURE
#define BEBOP_AUTO_EXPOSURE true
#endif

#ifndef BEBOP_AUTO_WHITE_BALANCE
#define BEBOP_AUTO_WHITE_BALANCE true
#endif

#define BEBOP_AWB_MIN_GAIN 2
#define BEBOP_AWB_MAX_GAIN 75

void bebop_ae_awb_init(void) {}

void bebop_ae_awb_periodic(void)
{
  struct isp_yuv_stats_t yuv_stats;

  if (isp_get_statistics_yuv(&yuv_stats) == 0) {
#if BEBOP_AUTO_EXPOSURE
    // Calculate the CDF based on the histogram
    uint32_t cdf[MAX_HIST_Y];
    cdf[0] = yuv_stats.ae_histogram_Y[0];
    for (int i = 1; i < MAX_HIST_Y; i++) {
      cdf[i] = cdf[i - 1] + yuv_stats.ae_histogram_Y[i];
    }

    // Calculate bright and saturated pixels
    uint32_t bright_pixels = cdf[MAX_HIST_Y - 1] - cdf[MAX_HIST_Y - 26]; // Top 25 bins
    uint32_t saturated_pixels = cdf[MAX_HIST_Y - 1] - cdf[MAX_HIST_Y - 6]; // top 5 bins
    uint32_t target_bright_pixels = yuv_stats.nb_valid_Y / 20; // 5%
    uint32_t max_saturated_pixels = yuv_stats.nb_valid_Y / 100; // 1%
    float adjustment = 1.0f;

    if (saturated_pixels + max_saturated_pixels / 10 > max_saturated_pixels) {
      // Fix saturated pixels
      adjustment = 1.0f - (float)saturated_pixels / yuv_stats.nb_valid_Y;
      adjustment *= adjustment * adjustment;  // speed up
    } else if (bright_pixels + target_bright_pixels / 10 < target_bright_pixels) {
      // increase brightness to try and hit the desired number of well exposed pixels
      int l = MAX_HIST_Y - 1;
      while (bright_pixels < target_bright_pixels && l > 0) {
        bright_pixels += cdf[l];
        bright_pixels -= cdf[l - 1];
        l--;
      }

      adjustment = (float)MAX_HIST_Y / (l + 1);
    } else if (bright_pixels - target_bright_pixels / 10 > target_bright_pixels) {
      // decrease brightness to try and hit the desired number of well exposed pixels
      int l = MAX_HIST_Y - 20;
      while (bright_pixels > target_bright_pixels && l < MAX_HIST_Y) {
        bright_pixels -= cdf[l];
        bright_pixels += cdf[l - 1];
        l++;
      }

      adjustment = (float)(MAX_HIST_Y - 20) / l;
      adjustment *= adjustment;   // speedup
    }

    // Calculate exposure
    Bound(adjustment, 1 / 16.0f, 4.0);
    mt9f002.target_exposure = mt9f002.real_exposure * adjustment;
    mt9f002_set_exposure(&mt9f002);
#endif

#if BEBOP_AUTO_WHITE_BALANCE
    // It is very important that the auto exposure converges faster than the color correction
    // Calculate AWB and project from original scale [0,255] onto more typical scale[-0.5,0.5]
    float avgU = ((float) yuv_stats.awb_sum_U / (float) yuv_stats.awb_nb_grey_pixels) / 256. - 0.5;
    float avgV = ((float) yuv_stats.awb_sum_V / (float) yuv_stats.awb_nb_grey_pixels) / 256. - 0.5;
    float threshold = 0.002f;
    float gain = 0.5;
    bool changed = false;

    if (fabs(avgU) > threshold) {
      mt9f002.gain_blue -= gain * avgU;
      changed = true;
    }
    if (fabs(avgV) > threshold) {
      mt9f002.gain_red -= gain * avgV;
      changed = true;
    }

    if (changed) {
      Bound(mt9f002.gain_blue, 2, 75);
      Bound(mt9f002.gain_red, 2, 75);
      mt9f002_set_gains(&mt9f002);
    }
#endif
  }
}

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
 * @file "modules/computer_vision/cv_ae_awb.c"
 * @author Freek van Tienen
 * Auto exposure and Auto white balancing for the Bebop 1 and 2
 */

#include "modules/computer_vision/cv_ae_awb.h"
#include "lib/isp/libisp.h"

#define MAX_HIST_Y 256-40

#include "boards/bebop/mt9f002.h"
extern struct mt9f002_t mt9f002;

void cv_ae_awb_init(void) {}
void cv_ae_awb_periodic(void) {
  struct isp_yuv_stats_t yuv_stats;

  if(isp_get_statistics_yuv(&yuv_stats) == 0) {
    //printf("%d, %d, %d, %d, %d\r\n", yuv_stats.awb_sum_Y, yuv_stats.awb_sum_U, yuv_stats.awb_sum_V,
    //  yuv_stats.awb_nb_grey_pixels, yuv_stats.nb_valid_Y);

    // Calculate the CDF based on the histogram
    uint32_t cdf[MAX_HIST_Y];
    cdf[0] = yuv_stats.ae_histogram_Y[0];
    for(int i = 1; i < MAX_HIST_Y; i++) {
      cdf[i] = cdf[i-1] + yuv_stats.ae_histogram_Y[i];
      //printf("%d, ", yuv_stats.ae_histogram_Y[i]);
    }
    //printf("\r\n\r\n");

    // Calculate bright and saturated pixels
    uint32_t bright_pixels = cdf[MAX_HIST_Y-1] - cdf[MAX_HIST_Y-21]; // Top 20 bins
    uint32_t saturated_pixels = cdf[MAX_HIST_Y-1] - cdf[MAX_HIST_Y-6]; // top 5 bins
    uint32_t target_bright_pixels = yuv_stats.nb_valid_Y / 100;
    uint32_t max_saturated_pixels = yuv_stats.nb_valid_Y / 400;
    float adjustment = 1.0f;

    // Fix saturated pixels
    if(saturated_pixels > max_saturated_pixels) {
      adjustment = 1.0f - ((float)(saturated_pixels - max_saturated_pixels))/yuv_stats.nb_valid_Y;
    }
    // Fix bright pixels
    else if (bright_pixels < target_bright_pixels) {
      // increase brightness to try and hit the desired number of well exposed pixels
      int l = MAX_HIST_Y-11;
      while (bright_pixels < target_bright_pixels && l > 0) {
          bright_pixels += cdf[l];
          bright_pixels -= cdf[l-1];
          l--;
      }

      // that level is supposed to be at MAX_HIST_Y-11;
      adjustment = (float)(MAX_HIST_Y-11+1)/(l+1);
    }

    // Calculate exposure
    Bound(adjustment, 1/16.0f, 4.0);
    float desiredExposure = mt9f002.real_exposure * adjustment;
    mt9f002.target_exposure = desiredExposure;
    mt9f002_set_exposure(&mt9f002);
    //printf("New exposure: %f (old: %f)\r\n", desiredExposure, mt9f002.real_exposure);


    // Calculate AWB
    float avgU = (float) yuv_stats.awb_sum_U / (float) yuv_stats.nb_valid_Y;
    float avgV = (float) yuv_stats.awb_sum_V / (float) yuv_stats.nb_valid_Y;
    float fTolerance = 0.1f;
    float targetAWB = -0.1f;

    printf("U-V: %f\r\n", avgU - avgV);
    if (avgU - avgV + targetAWB < -fTolerance) {
      // Want more red
      printf("Too red...\r\n");
      mt9f002.gain_blue += 0.1;
      mt9f002.gain_red  -= 0.1;
      Bound(mt9f002.gain_blue, 2, 50);
      Bound(mt9f002.gain_red, 2, 50);
      mt9f002_set_gains(&mt9f002);
    }
    else if(avgU - avgV + targetAWB > fTolerance) {
      // Want more blue
      printf("Too blue...\r\n");
      mt9f002.gain_blue -= 0.1;
      mt9f002.gain_red  += 0.1;
      Bound(mt9f002.gain_blue, 2, 50);
      Bound(mt9f002.gain_red, 2, 50);
      mt9f002_set_gains(&mt9f002);
    }
  }
}



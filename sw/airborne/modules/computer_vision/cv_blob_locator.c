/*
 * Copyright (C) 2015
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
 * @file "modules/computer_vision/cv_blob_locator.c"
 * @author C. De Wagter
 * Find a colored item and track its geo-location and update a waypoint to it
 */

#include "modules/computer_vision/cv_blob_locator.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/blob/blob_finder.h"


uint8_t color_lum_min;
uint8_t color_lum_max;

uint8_t color_cb_min;
uint8_t color_cb_max;

uint8_t color_cr_min;
uint8_t color_cr_max;

uint8_t cv_blob_locator_reset;

volatile uint32_t blob_locator = 0;

// Computer vision thread
bool_t cv_blob_locator_func(struct image_t *img);
bool_t cv_blob_locator_func(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter[2];
  filter[0].y_min = color_lum_min;
  filter[0].y_max = color_lum_max;
  filter[0].u_min = color_cb_min;
  filter[0].u_max = color_cb_max;
  filter[0].v_min = color_cr_min;
  filter[0].v_max = color_cr_max;

  // Output image
  struct image_t dst;
  image_create(&dst,
                 img->w,
                 img->h,
                 IMAGE_GRADIENT);

  // Labels
  uint16_t labels_count = 512;
  struct image_label_t labels[512];

  // Blob finder
  image_labeling(img, &dst, filter, 1, labels, &labels_count);

  int largest_id = -1;
  int largest_size = 0;

  // Find largest
  for (int i=0; i<labels_count; i++) {
    // Only consider large blobs
    if (labels[i].pixel_cnt > 50) {
      if (labels[i].pixel_cnt > largest_size) {
        largest_size = labels[i].pixel_cnt;
        largest_id = i;
      }
    }
  }

  if (largest_id >= 0)
  {
    uint8_t *p = (uint8_t*) img->buf;
    uint16_t* l = (uint16_t*) dst.buf;
    for (int y=0;y<dst.h;y++) {
      for (int x=0;x<dst.w/2;x++) {
        if (l[y*dst.w+x] != 0xffff) {
          uint8_t c=0xff;
          if (l[y*dst.w+x] == largest_id) {
            c = 0;
          }
          p[y*dst.w*2+x*4]=c;
          p[y*dst.w*2+x*4+1]=0x80;
          p[y*dst.w*2+x*4+2]=c;
          p[y*dst.w*2+x*4+3]=0x80;
        }
      }
    }


    uint16_t cgx = labels[largest_id].x_sum / labels[largest_id].pixel_cnt * 2;
    uint16_t cgy = labels[largest_id].y_sum / labels[largest_id].pixel_cnt;

    if ((cgx > 1) && (cgx < (dst.w-2)) &&
        (cgy > 1) && (cgy < (dst.h-2))
        ) {
      p[cgy*dst.w*2+cgx*2-4] = 0xff;
      p[cgy*dst.w*2+cgx*2-2] = 0x00;
      p[cgy*dst.w*2+cgx*2] = 0xff;
      p[cgy*dst.w*2+cgx*2+2] = 0x00;
      p[cgy*dst.w*2+cgx*2+4] = 0xff;
      p[cgy*dst.w*2+cgx*2+6] = 0x00;
      p[(cgy-1)*dst.w*2+cgx*2] = 0xff;
      p[(cgy-1)*dst.w*2+cgx*2+2] = 0x00;
      p[(cgy+1)*dst.w*2+cgx*2] = 0xff;
      p[(cgy+1)*dst.w*2+cgx*2+2] = 0x00;
    }


    uint32_t temp = cgx;
    temp = temp << 16;
    temp += cgy;
    blob_locator = temp;
  }

  image_free(&dst);

  return FALSE;
}

#include "modules/computer_vision/cv_georeference.h"
#include "generated/flight_plan.h"
#include <stdio.h>


void cv_blob_locator_init(void) {
  // Red board in sunlight
  color_lum_min = 100;
  color_lum_max = 200;
  color_cb_min = 140;
  color_cb_max = 255;
  color_cr_min = 140;
  color_cr_max = 255;

  // Lamp during night
  color_lum_min = 180;
  color_lum_max = 255;
  color_cb_min = 100;
  color_cb_max = 150;
  color_cr_min = 100;
  color_cr_max = 150;

  cv_blob_locator_reset = 0;

  georeference_init();

  cv_add(cv_blob_locator_func);
}

void cv_blob_locator_periodic(void) {

}



void cv_blob_locator_event(void) {
  if (blob_locator != 0) {
    // CV thread has results: import
    uint32_t temp = blob_locator;
    blob_locator = 0;

    // Process
    uint16_t y = temp & 0x0000ffff;
    temp = temp >> 16;
    uint16_t x = temp & 0x0000ffff;
    printf("Found %d %d \n",x,y);

    struct camera_frame_t cam;
    cam.px = x/2;
    cam.py = y/2;
    cam.f = 400;
    cam.h = 240;
    cam.w = 320;

    georeference_project(&cam, WP_p2);
    georeference_filter(FALSE,WP_p1);

  }
}

extern void cv_blob_locator_start(void) {
  georeference_init();
}

extern void cv_blob_locator_stop(void) {

}

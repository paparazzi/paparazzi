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
#include "modules/computer_vision/blob/imavmarker.h"
#include "modules/computer_vision/detect_window.h"

#ifndef BLOB_LOCATOR_FPS
#define BLOB_LOCATOR_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(BLOB_LOCATOR_FPS)


uint8_t color_lum_min;
uint8_t color_lum_max;

uint8_t color_cb_min;
uint8_t color_cb_max;

uint8_t color_cr_min;
uint8_t color_cr_max;

uint8_t cv_blob_locator_reset;
uint8_t cv_blob_locator_type;

int geofilter_length = 5;
int marker_size = 18;
int record_video = 0;

volatile uint32_t blob_locator = 0;

volatile bool blob_enabled = false;
volatile bool marker_enabled = false;
volatile bool window_enabled = false;

// Computer vision thread
struct image_t *cv_marker_func(struct image_t *img);
struct image_t *cv_marker_func(struct image_t *img)
{

  if (!marker_enabled) {
    return NULL;
  }

  struct marker_deviation_t m = marker(img, marker_size);

  uint32_t temp = m.x;
  temp = temp << 16;
  temp += m.y;
  blob_locator = temp;

  return NULL;
}

#define Img(X,Y)(((uint8_t*)img->buf)[(Y)*img->w*2+(X)*2])


// Computer vision thread
struct image_t *cv_window_func(struct image_t *img);
struct image_t *cv_window_func(struct image_t *img)
{

  if (!window_enabled) {
    return NULL;
  }


  uint16_t coordinate[2] = {0, 0};
  uint16_t response = 0;
  uint32_t integral_image[img->w * img->h];

  struct image_t gray;
  image_create(&gray, img->w, img->h, IMAGE_GRAYSCALE);
  image_to_grayscale(img, &gray);

  response = detect_window_sizes((uint8_t *)gray.buf, (uint32_t)img->w, (uint32_t)img->h, coordinate, integral_image, MODE_BRIGHT);

  image_free(&gray);

  // Display the marker location and center-lines.
  int px = coordinate[0] & 0xFFFe;
  int py = coordinate[1] & 0xFFFe;

  if (response < 92) {

    for (int y = 0; y < img->h - 1; y++) {
      Img(px, y)   = 65;
      Img(px + 1, y) = 255;
    }
    for (int x = 0; x < img->w - 1; x += 2) {
      Img(x, py)   = 65;
      Img(x + 1, py) = 255;
    }

    uint32_t temp = coordinate[0];
    temp = temp << 16;
    temp += coordinate[1];
    blob_locator = temp;

  }

  return NULL;
}


struct image_t *cv_blob_locator_func(struct image_t *img);
struct image_t *cv_blob_locator_func(struct image_t *img)
{

  if (!blob_enabled) {
    return NULL;
  }


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
  for (int i = 0; i < labels_count; i++) {
    // Only consider large blobs
    if (labels[i].pixel_cnt > 50) {
      if (labels[i].pixel_cnt > largest_size) {
        largest_size = labels[i].pixel_cnt;
        largest_id = i;
      }
    }
  }

  if (largest_id >= 0) {
    uint8_t *p = (uint8_t *) img->buf;
    uint16_t *l = (uint16_t *) dst.buf;
    for (int y = 0; y < dst.h; y++) {
      for (int x = 0; x < dst.w / 2; x++) {
        if (l[y * dst.w + x] != 0xffff) {
          uint8_t c = 0xff;
          if (l[y * dst.w + x] == largest_id) {
            c = 0;
          }
          p[y * dst.w * 2 + x * 4] = c;
          p[y * dst.w * 2 + x * 4 + 1] = 0x80;
          p[y * dst.w * 2 + x * 4 + 2] = c;
          p[y * dst.w * 2 + x * 4 + 3] = 0x80;
        }
      }
    }


    uint16_t cgx = labels[largest_id].x_sum / labels[largest_id].pixel_cnt * 2;
    uint16_t cgy = labels[largest_id].y_sum / labels[largest_id].pixel_cnt;

    if ((cgx > 1) && (cgx < (dst.w - 2)) &&
        (cgy > 1) && (cgy < (dst.h - 2))
       ) {
      p[cgy * dst.w * 2 + cgx * 2 - 4] = 0xff;
      p[cgy * dst.w * 2 + cgx * 2 - 2] = 0x00;
      p[cgy * dst.w * 2 + cgx * 2] = 0xff;
      p[cgy * dst.w * 2 + cgx * 2 + 2] = 0x00;
      p[cgy * dst.w * 2 + cgx * 2 + 4] = 0xff;
      p[cgy * dst.w * 2 + cgx * 2 + 6] = 0x00;
      p[(cgy - 1)*dst.w * 2 + cgx * 2] = 0xff;
      p[(cgy - 1)*dst.w * 2 + cgx * 2 + 2] = 0x00;
      p[(cgy + 1)*dst.w * 2 + cgx * 2] = 0xff;
      p[(cgy + 1)*dst.w * 2 + cgx * 2 + 2] = 0x00;
    }


    uint32_t temp = cgx;
    temp = temp << 16;
    temp += cgy;
    blob_locator = temp;
  }

  image_free(&dst);

  return NULL; // No new image is available for follow up modules
}

#include "modules/computer_vision/cv_georeference.h"
#include "generated/flight_plan.h"
#include <stdio.h>


void cv_blob_locator_init(void)
{
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

  cv_add_to_device(&BLOB_LOCATOR_CAMERA, cv_blob_locator_func, BLOB_LOCATOR_FPS);
  cv_add_to_device(&BLOB_LOCATOR_CAMERA, cv_marker_func, BLOB_LOCATOR_FPS);
  cv_add_to_device(&BLOB_LOCATOR_CAMERA, cv_window_func, BLOB_LOCATOR_FPS);
}

void cv_blob_locator_periodic(void)
{

}



void cv_blob_locator_event(void)
{
  switch (cv_blob_locator_type) {
    case 1:
      blob_enabled = true;
      marker_enabled = false;
      window_enabled = false;
      break;
    case 2:
      blob_enabled = false;
      marker_enabled = true;
      window_enabled = false;
      break;
    case 3:
      blob_enabled = false;
      marker_enabled = false;
      window_enabled = true;
      break;
    default:
      blob_enabled = false;
      marker_enabled = false;
      window_enabled = false;
      break;
  }
  if (blob_locator != 0) {
    // CV thread has results: import
    uint32_t temp = blob_locator;
    blob_locator = 0;

    // Process
    uint16_t y = temp & 0x0000ffff;
    temp = temp >> 16;
    uint16_t x = temp & 0x0000ffff;
    printf("Found %d %d \n", x, y);

    struct camera_frame_t cam;
    cam.px = x / 2;
    cam.py = y / 2;
    cam.f = 400;
    cam.h = 240;
    cam.w = 320;

#ifdef WP_p1
    georeference_project(&cam, WP_p1);
#endif
#ifdef WP_CAM
    georeference_filter(FALSE, WP_CAM, geofilter_length);
#endif

  }
}

extern void cv_blob_locator_start(void)
{
  georeference_init();
}

extern void cv_blob_locator_stop(void)
{

}

void start_vision(void)
{
  georeference_init();
  record_video = 1;
  cv_blob_locator_type = 3;
}
void start_vision_land(void)
{
  georeference_init();
  record_video = 1;
  cv_blob_locator_type = 2;
}
void stop_vision(void)
{
  georeference_init();
  record_video = 0;
  cv_blob_locator_type = 0;
}

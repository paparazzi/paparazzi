/*
 * Copyright (C) Tom van Dijk
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
 * @file "modules/pano_unwrap/pano_unwrap.c"
 * @author Tom van Dijk
 * Unwrap images taken through a panoramic lens.
 */

#include "modules/pano_unwrap/pano_unwrap.h"

#include "state.h"
#include "modules/computer_vision/cv.h"

#include <stdio.h>

#ifndef PANO_UNWRAP_CAMERA
#define PANO_UNWRAP_CAMERA bottom_camera
#endif

#ifndef PANO_UNWRAP_CENTER_X
#define PANO_UNWRAP_CENTER_X 0.50
#endif
#ifndef PANO_UNWRAP_CENTER_Y
#define PANO_UNWRAP_CENTER_Y 0.50
#endif
#ifndef PANO_UNWRAP_RADIUS_BOTTOM
#define PANO_UNWRAP_RADIUS_BOTTOM 0.20
#endif
#ifndef PANO_UNWRAP_RADIUS_TOP
#define PANO_UNWRAP_RADIUS_TOP 0.30
#endif
#ifndef PANO_UNWRAP_FORWARD_DIRECTION
#define PANO_UNWRAP_FORWARD_DIRECTION 270.0
#endif
#ifndef PANO_UNWRAP_FLIP_HORIZONTAL
#define PANO_UNWRAP_FLIP_HORIZONTAL FALSE
#endif

#ifndef PANO_UNWRAP_VERTICAL_RESOLUTION
#define PANO_UNWRAP_VERTICAL_RESOLUTION 0.08
#endif
#ifndef PANO_UNWRAP_DEROTATE_ATTITUDE
#define PANO_UNWRAP_DEROTATE_ATTITUDE FALSE
#endif

#ifndef PANO_UNWRAP_WIDTH
#define PANO_UNWRAP_WIDTH 640
#endif
#ifndef PANO_UNWRAP_HEIGHT
#define PANO_UNWRAP_HEIGHT 0
#endif

#ifndef PANO_UNWRAP_OVERWRITE_VIDEO_THREAD
#define PANO_UNWRAP_OVERWRITE_VIDEO_THREAD TRUE
#endif

struct pano_unwrap_t pano_unwrap = {
    .center = {
        .x = PANO_UNWRAP_CENTER_X,
        .y = PANO_UNWRAP_CENTER_Y,
    },
    .radius_bottom = PANO_UNWRAP_RADIUS_BOTTOM,
    .radius_top = PANO_UNWRAP_RADIUS_TOP,
    .forward_direction = PANO_UNWRAP_FORWARD_DIRECTION,
    .flip_horizontal = PANO_UNWRAP_FLIP_HORIZONTAL,

    .vertical_resolution = PANO_UNWRAP_VERTICAL_RESOLUTION,
    .derotate_attitude = PANO_UNWRAP_DEROTATE_ATTITUDE,

    .width = PANO_UNWRAP_WIDTH,
    .height = PANO_UNWRAP_HEIGHT,

    .overwrite_video_thread = PANO_UNWRAP_OVERWRITE_VIDEO_THREAD,

    .show_calibration = FALSE,
};

struct image_t pano_unwrapped_image;

#define PIXEL_UV(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 2*(y)*(img)->w] )
#define PIXEL_U(img,x,y) ( ((uint8_t*)((img)->buf))[4*(int)((x)/2) + 2*(y)*(img)->w] )
#define PIXEL_V(img,x,y) ( ((uint8_t*)((img)->buf))[4*(int)((x)/2) + 2*(y)*(img)->w + 2] )
#define PIXEL_Y(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 1 + 2*(y)*(img)->w] )

static void set_output_image_size(void)
{
  // Find vertical size of output image
  if (pano_unwrap.height == 0) {
    pano_unwrap.height = (uint16_t) (fabsf((pano_unwrap.width / (2 * M_PI))
        * (pano_unwrap.radius_bottom - pano_unwrap.radius_top)
        / pano_unwrap.vertical_resolution) + 0.5);
    printf("[pano_unwrap] Automatic output image height: %d\n",
        pano_unwrap.height);
  }
  // Replace output image if required
  if (pano_unwrapped_image.w != pano_unwrap.width ||
      pano_unwrapped_image.h != pano_unwrap.height) {
    printf("[pano_unwrap] Creating output image with size %d x %d\n",
        pano_unwrap.width, pano_unwrap.height);
    image_free(&pano_unwrapped_image);
    image_create(&pano_unwrapped_image, pano_unwrap.width, pano_unwrap.height,
        IMAGE_YUV422);
  }
}

static void get_point(struct point_t *pt_out, const struct image_t * img,
    float bearing, float height)
{
  float radius = pano_unwrap.radius_top
      + (pano_unwrap.radius_bottom - pano_unwrap.radius_top) * height;
  if (pano_unwrap.derotate_attitude) {
    // Note: assumes small pitch/roll angles (<= approx. 20 deg)
    const struct FloatRMat *R = stateGetNedToBodyRMat_f();
    radius += pano_unwrap.vertical_resolution
        * (cosf(bearing) * MAT33_ELMT(*R, 0, 2)
            + sinf(bearing) * MAT33_ELMT(*R, 1, 2));
  }
  float angle = (pano_unwrap.forward_direction / 180.0 * M_PI)
      - bearing * (pano_unwrap.flip_horizontal ? -1.f : 1.f);

  struct FloatVect2 pt;
  pt.x = img->w * pano_unwrap.center.x + cosf(angle) * radius * img->h;
  pt.y = img->h * pano_unwrap.center.y - sinf(angle) * radius * img->h;

  // Check bounds
  if (pt.x < 0) {
    pt.x = 0;
  }
  if (pt.x > img->w - 1) {
    pt.x = img->w - 1;
  }
  if (pt.y < 0) {
    pt.y = 0;
  }
  if (pt.y > img->h - 1) {
    pt.y = img->h - 1;
  }

  pt_out->x = (uint16_t) (pt.x + 0.5);
  pt_out->y = (uint16_t) (pt.y + 0.5);
}

static void draw_calibration(struct image_t *img)
{
  struct point_t pt;
  for (float bearing = 0; bearing <= 2 * M_PI; bearing += 0.01) {
    get_point(&pt, img, bearing, 0.f);
    PIXEL_U(img, pt.x, pt.y) = 84;
    PIXEL_V(img, pt.x, pt.y) = 255;
    get_point(&pt, img, bearing, 1.f);
    PIXEL_U(img, pt.x, pt.y) = 84;
    PIXEL_V(img, pt.x, pt.y) = 255;
  }
  for (float height = 0; height < 1; height += 0.1) {
    get_point(&pt, img, 0.f, height);
    PIXEL_Y(img, pt.x, pt.y) = 76;
    PIXEL_U(img, pt.x, pt.y) = 84;
    PIXEL_V(img, pt.x, pt.y) = 255;
    get_point(&pt, img, M_PI_2, height);
    PIXEL_Y(img, pt.x, pt.y) = 149;
    PIXEL_U(img, pt.x, pt.y) = 43;
    PIXEL_V(img, pt.x, pt.y) = 21;
  }
  pt.x = img->w * pano_unwrap.center.x;
  pt.y = img->h * pano_unwrap.center.y;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      PIXEL_Y(img, pt.x+x, pt.y+y) = 29;
      PIXEL_U(img, pt.x+x, pt.y+y) = 255;
      PIXEL_V(img, pt.x+x, pt.y+y) = 107;
    }
  }
}

static struct image_t *camera_cb(struct image_t *img)
{
  set_output_image_size();
  if (pano_unwrap.show_calibration) {
    draw_calibration(img);
  }
  for (uint16_t x = 0; x < pano_unwrapped_image.w; x++) {
    for (uint16_t y = 0; y < pano_unwrapped_image.h; y++) {
      float bearing = -M_PI + (float) x / pano_unwrapped_image.w * 2 * M_PI;
      float height = (float) y / pano_unwrapped_image.h;
      struct point_t pt;
      get_point(&pt, img, bearing, height);
      PIXEL_Y(&pano_unwrapped_image, x, y) = PIXEL_Y(img, pt.x, pt.y);
      PIXEL_U(&pano_unwrapped_image, x, y) = PIXEL_U(img, pt.x, pt.y);
      PIXEL_V(&pano_unwrapped_image, x, y) = PIXEL_V(img, pt.x, pt.y);
    }
  }
  pano_unwrapped_image.ts = img->ts;
  pano_unwrapped_image.pprz_ts = img->pprz_ts;
  return pano_unwrap.overwrite_video_thread ? &pano_unwrapped_image : NULL;
}

void pano_unwrap_init()
{
  image_create(&pano_unwrapped_image, 0, 0, IMAGE_YUV422);
  cv_add_to_device(&PANO_UNWRAP_CAMERA, camera_cb, 0);
}


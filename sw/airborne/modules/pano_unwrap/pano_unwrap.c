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
#define PANO_UNWRAP_VERTICAL_RESOLUTION 0.18
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

#ifndef PANO_UNWRAP_FPS
#define PANO_UNWRAP_FPS 0
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

#define PIXEL_U(img,x,y) ( ((uint8_t*)((img)->buf))[4*(int)((x)/2) + 2*(y)*(img)->w] )
#define PIXEL_V(img,x,y) ( ((uint8_t*)((img)->buf))[4*(int)((x)/2) + 2*(y)*(img)->w + 2] )
#define PIXEL_Y(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 1 + 2*(y)*(img)->w] )

#define RED_Y 76
#define RED_U 84
#define RED_V 255
#define GREEN_Y 149
#define GREEN_U 43
#define GREEN_V 21
#define BLUE_Y 29
#define BLUE_U 255
#define BLUE_V 107

struct LUT_t
{
  // Unwarping LUT
  // For each pixel (u,v) in the unwrapped image, contains pixel coordinates of
  // the same pixel in the raw image (x(u,v), y(u,v)).
  uint16_t *x;
  uint16_t *y;
  // Derotate LUT
  // For each bearing/column (u), contains the direction in which sampling should be
  // shifted based on euler angles phi and theta dxy(phi|u) dxy(theta|u).
  struct FloatVect2 *dphi;
  struct FloatVect2 *dtheta;
  // Settings for which the LUT was generated
  struct pano_unwrap_t settings;
};
static struct LUT_t LUT;  // Note: initialized NULL

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

static void update_LUT(const struct image_t *img)
{
  if (LUT.settings.center.x == pano_unwrap.center.x &&
      LUT.settings.center.y == pano_unwrap.center.y &&
      LUT.settings.radius_bottom == pano_unwrap.radius_bottom &&
      LUT.settings.radius_top == pano_unwrap.radius_top &&
      LUT.settings.forward_direction == pano_unwrap.forward_direction &&
      LUT.settings.flip_horizontal == pano_unwrap.flip_horizontal &&
      LUT.settings.width == pano_unwrap.width &&
      LUT.settings.height == pano_unwrap.height) {
    // LUT is still valid, do nothing
    return;
  }

  printf("[pano_unwrap] Regenerating LUT... ");
  // Remove old data
  if (LUT.x) {
    free(LUT.x);
    LUT.x = NULL;
  }
  if (LUT.y) {
    free(LUT.y);
    LUT.y = NULL;
  }
  if (LUT.dphi) {
    free(LUT.dphi);
    LUT.dphi = NULL;
  }
  if (LUT.dtheta) {
    free(LUT.dtheta);
    LUT.dtheta = NULL;
  }
  // Generate unwarping LUT
  LUT.x = malloc(sizeof(*LUT.x) * pano_unwrap.width * pano_unwrap.height);
  LUT.y = malloc(sizeof(*LUT.y) * pano_unwrap.width * pano_unwrap.height);
  if (!LUT.x || !LUT.y) {
    printf("[pano_unwrap] ERROR could not allocate x or y lookup table!\n");
  } else {
    for (uint16_t u = 0; u < pano_unwrap.width; u++) {
      float bearing = -M_PI + (float) u / pano_unwrapped_image.w * 2 * M_PI;
      float angle = (pano_unwrap.forward_direction / 180.0 * M_PI)
          + bearing * (pano_unwrap.flip_horizontal ? 1.f : -1.f);
      float c = cosf(angle);
      float s = sinf(angle);
      for (uint16_t v = 0; v < pano_unwrap.height; v++) {
        float radius = pano_unwrap.radius_top
            + (pano_unwrap.radius_bottom - pano_unwrap.radius_top)
                * ((float) v / (pano_unwrap.height - 1));
        LUT.x[v + u * pano_unwrap.height] = (uint16_t) (img->w
            * pano_unwrap.center.x + c * radius * img->h + 0.5);
        LUT.y[v + u * pano_unwrap.height] = (uint16_t) (img->h
            * pano_unwrap.center.y - s * radius * img->h + 0.5);
      }
    }
  }
  // Generate derotation LUT
  LUT.dphi = malloc(sizeof(*LUT.dphi) * pano_unwrap.width);
  LUT.dtheta = malloc(sizeof(*LUT.dtheta) * pano_unwrap.width);
  if (!LUT.dphi || !LUT.dtheta) {
    printf(
        "[pano_unwrap] ERROR could not allocate dphi or dtheta lookup table!\n");
  } else {
    for (uint16_t u = 0; u < pano_unwrap.width; u++) {
      float bearing = -M_PI + (float) u / pano_unwrapped_image.w * 2 * M_PI;
      float angle = (pano_unwrap.forward_direction / 180.0 * M_PI)
          + bearing * (pano_unwrap.flip_horizontal ? 1.f : -1.f);
      LUT.dphi[u].x = sinf(bearing) * cosf(angle);
      LUT.dphi[u].y = sinf(bearing) * -sinf(angle);
      LUT.dtheta[u].x = cosf(bearing) * cosf(angle);
      LUT.dtheta[u].y = cosf(bearing) * -sinf(angle);
    }
  }
  // Keep track of settings for which this LUT was generated
  LUT.settings = pano_unwrap;
  printf("ok\n");
}

static void unwrap_LUT(struct image_t *img_raw, struct image_t *img)
{
  for (uint16_t u = 0; u < pano_unwrap.width; u++) {
    // Derotation offset for this bearing
    int16_t dx = 0;
    int16_t dy = 0;
    if (pano_unwrap.derotate_attitude) {
      // Look up correction
      const struct FloatRMat *R = stateGetNedToBodyRMat_f();
      dx = (int16_t) (img_raw->h * pano_unwrap.vertical_resolution
          * (LUT.dphi[u].x * MAT33_ELMT(*R, 1, 2)
              + LUT.dtheta[u].x * MAT33_ELMT(*R, 0, 2)));
      dy = (int16_t) (img_raw->h * pano_unwrap.vertical_resolution
          * (LUT.dphi[u].y * MAT33_ELMT(*R, 1, 2)
              + LUT.dtheta[u].y * MAT33_ELMT(*R, 0, 2)));
    }
    // Fill this column of unwrapped image
    for (uint16_t v = 0; v < pano_unwrap.height; v++) {
      // Look up sampling point
      int16_t x = (int16_t) LUT.x[v + u * pano_unwrap.height] + dx;
      int16_t y = (int16_t) LUT.y[v + u * pano_unwrap.height] + dy;
      // Check bounds
      if (x < 0) {
        x = 0;
      } else if (x > img_raw->w - 1) {
        x = img_raw->w - 1;
      }
      if (y < 0) {
        y = 0;
      } else if (y > img_raw->h - 1) {
        y = img_raw->h - 1;
      }
      // Draw calibration pattern
      if (pano_unwrap.show_calibration) {
        if (v == 0 || v == pano_unwrap.height - 1) {
          PIXEL_U(img_raw,x,y) = BLUE_U;
          PIXEL_V(img_raw,x,y) = BLUE_V;
        }
        if ((u == pano_unwrap.width / 2)
            || (u == pano_unwrap.width / 2 + 1)) {
          PIXEL_Y(img_raw,x,y) = RED_Y;
          PIXEL_U(img_raw,x,y) = RED_U;
          PIXEL_V(img_raw,x,y) = RED_V;
        }
        if ((u == 3 * pano_unwrap.width / 4)
            || (u == 3 * pano_unwrap.width / 4 + 1)) {
          PIXEL_Y(img_raw,x,y) = GREEN_Y;
          PIXEL_U(img_raw,x,y) = GREEN_U;
          PIXEL_V(img_raw,x,y) = GREEN_V;
        }
      }
      // Copy pixel values
      PIXEL_Y(img,u,v) = PIXEL_Y(img_raw, x, y);
      PIXEL_U(img,u,v) = PIXEL_U(img_raw, x, y);
      PIXEL_V(img,u,v) = PIXEL_V(img_raw, x, y);
    }
  }
  // Draw lens center
  if (pano_unwrap.show_calibration) {
    uint16_t x = pano_unwrap.center.x * img_raw->w;
    uint16_t y = pano_unwrap.center.y * img_raw->h;
    for (int i = -5; i <= 5; i++) {
      for (int j = -5; j <= 5; j++) {
        if (i == 0 || j == 0) {
          PIXEL_Y(img_raw, x+i, y+j) = BLUE_Y;
          PIXEL_U(img_raw, x+i, y+j) = BLUE_U;
          PIXEL_V(img_raw, x+i, y+j) = BLUE_V;
        }
      }
    }
  }
}

static struct image_t *camera_cb(struct image_t *img, uint8_t camera_id)
{
  set_output_image_size();
  update_LUT(img);
  unwrap_LUT(img, &pano_unwrapped_image);
  pano_unwrapped_image.ts = img->ts;
  pano_unwrapped_image.pprz_ts = img->pprz_ts;
  return pano_unwrap.overwrite_video_thread ? &pano_unwrapped_image : NULL;
}

void pano_unwrap_init()
{
  image_create(&pano_unwrapped_image, 0, 0, IMAGE_YUV422);
  set_output_image_size();
  cv_add_to_device(&PANO_UNWRAP_CAMERA, camera_cb, PANO_UNWRAP_FPS, 0);
}


/*
 * Copyright (C) 2018, Guido de Croon
 *
 * @file modules/computer_vision/undistort_image.c
 */

// Own header
#include "modules/computer_vision/undistort_image.h"
#include <stdio.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/lib/vision/undistortion.h"

#ifndef UNDISTORT_FPS
#define UNDISTORT_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(UNDISTORT_FPS)

#ifndef UNDISTORT_MIN_X_NORMALIZED
#define UNDISTORT_MIN_X_NORMALIZED -2.0f  ///< Minimal normalized coordinate that will be shown in the undistorted image
#endif
PRINT_CONFIG_VAR(UNDISTORT_MIN_X_NORMALIZED)

#ifndef UNDISTORT_MAX_X_NORMALIZED
#define UNDISTORT_MAX_X_NORMALIZED 2.0f  ///< Maximal normalized coordinate that will be shown in the undistorted image
#endif
PRINT_CONFIG_VAR(UNDISTORT_MAX_X_NORMALIZED)

#ifndef UNDISTORT_DHANE_K
#define UNDISTORT_DHANE_K 1.25f  ///< Maximal normalized coordinate that will be shown in the undistorted image
#endif
PRINT_CONFIG_VAR(UNDISTORT_DHANE_K)

#ifndef UNDISTORT_CENTER_RATIO
#define UNDISTORT_CENTER_RATIO 1.00f  ///< Maximal normalized coordinate that will be shown in the undistorted image
#endif
PRINT_CONFIG_VAR(UNDISTORT_CENTER_RATIO)

float min_x_normalized;
float max_x_normalized;
float dhane_k;
float center_ratio;

struct video_listener *listener = NULL;

// Camera calibration matrix (of the Bebop2 for a specific part of the visual sensor):
static float K[9] = {189.69f, 0.0f, 165.04f,
                     0.0f, 188.60f, 118.44f,
                     0.0f, 0.0f, 1.0f};

//static float k;

// Function
struct image_t *undistort_image_func(struct image_t *img);
struct image_t *undistort_image_func(struct image_t *img)
{
  // TODO: These commands could actually only be run when the parameters or image size are changed
  float normalized_step = (max_x_normalized - min_x_normalized) / img->w;
  float h_w_ratio = img->h / (float) img->w;
  float min_y_normalized = h_w_ratio * min_x_normalized;
  float max_y_normalized = h_w_ratio * max_x_normalized;

  // create an image of the same size:
  struct image_t img_distorted;
  image_create(&img_distorted, img->w, img->h, img->type);
  // this info is not created in image_create, but is necessary for streaming...:
  //img_distorted.ts = img->ts;
  //img_distorted.eulers = img->eulers;
  //img_distorted.pprz_ts = img->pprz_ts;

  uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;

  image_copy(img, &img_distorted);

  // do the copy first, and then put the undistorted image directly in the img.

  uint8_t *dest = (uint8_t *)img->buf;
  uint8_t *source = (uint8_t *)img_distorted.buf;
  uint32_t index_dest, index_src;

  // set all pixels to black:
  for(uint32_t x = 0; x < img->w; x++) {
      for(uint32_t y = 0; y < img->h; y++) {
        index_dest = pixel_width*(y*img->w+x);
        dest[index_dest] = 128; // grey
        dest[index_dest+1] = 0; // black
      }
  }

  // fill the image again, now with the undistorted image:
  float x_pd, y_pd;
  uint32_t x_pd_ind, y_pd_ind;
  uint32_t x = 0;
  for(float x_n = min_x_normalized; x_n < max_x_normalized; x_n += normalized_step, x++) {
    uint32_t y = 0;
    for(float y_n = min_y_normalized; y_n < max_y_normalized; y_n += normalized_step, y++) {
      // for faster execution but smaller FOV, uncomment the next line:
      if(center_ratio == 1.0f ||
          (x_n > center_ratio * min_x_normalized && x_n < center_ratio * max_x_normalized && y_n > center_ratio * min_y_normalized && y_n < center_ratio * max_y_normalized)
          ) {
        normalized_coords_to_distorted_pixels(x_n, y_n, &x_pd, &y_pd, dhane_k, K);
        if(x_pd > 0.0f && y_pd > 0.0f) {
          x_pd_ind = (uint32_t) x_pd;
          y_pd_ind = (uint32_t) y_pd;
          if(x_pd_ind < img->w && y_pd_ind < img->h) {
            // Assuming UY VY (2 bytes per pixel, and U for even indices, V for odd indices)
            index_dest = pixel_width*(y*img->w+x);
            index_src = pixel_width*(y_pd_ind*img_distorted.w+x_pd_ind);
            dest[index_dest] = 128; // source[index_src]; // Colors will be a pain for undistortion...
            dest[index_dest+1] = source[index_src+1]; // no interpolation or anything, just the basics for now.
          }
        }
      }
    }
  }

  image_free(&img_distorted);
  return img;
}

void undistort_image_init(void)
{

  min_x_normalized = UNDISTORT_MIN_X_NORMALIZED;
  max_x_normalized = UNDISTORT_MAX_X_NORMALIZED;
  center_ratio = UNDISTORT_CENTER_RATIO;
  dhane_k = UNDISTORT_DHANE_K;
  listener = cv_add_to_device(&UNDISTORT_CAMERA, undistort_image_func, UNDISTORT_FPS);
}

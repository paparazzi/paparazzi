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

/*
#ifndef UNDISTORT_MIN_Y_NORMALIZED
#define UNDISTORT_MIN_Y_NORMALIZED -2.0f  ///< Minimal normalized coordinate that will be shown in the undistorted image
#endif
PRINT_CONFIG_VAR(UNDISTORT_MIN_Y_NORMALIZED)

#ifndef UNDISTORT_MAX_Y_NORMALIZED
#define UNDISTORT_MAX_Y_NORMALIZED 2.0f  ///< Maximal normalized coordinate that will be shown in the undistorted image
#endif
PRINT_CONFIG_VAR(UNDISTORT_MAX_Y_NORMALIZED)

#ifndef UNDISTORT_NORMALIZED_STEP
#define UNDISTORT_NORMALIZED_STEP 0.01f  ///< Step in the normalized domain to make the new image
#endif
PRINT_CONFIG_VAR(UNDISTORT_NORMALIZED_STEP)
*/

struct video_listener *listener = NULL;

// Camera calibration matrix (of the Bebop2 for a specific part of the visual sensor):
static float K[9] = {189.69f, 0.0f, 165.04f,
                     0.0f, 188.60f, 118.44f,
                     0.0f, 0.0f, 1.0f};

static float k;

// Function
struct image_t *undistort_image_func(struct image_t *img);
struct image_t *undistort_image_func(struct image_t *img)
{

  // TODO: These commands could actually only be run when the parameters or image size are changed
  float normalized_step = (UNDISTORT_MAX_X_NORMALIZED - UNDISTORT_MIN_X_NORMALIZED) / img->w;
  float h_w_ratio = img->h / (float) img->w;
  float min_y_normalized = h_w_ratio * UNDISTORT_MIN_X_NORMALIZED;
  float max_y_normalized = h_w_ratio * UNDISTORT_MAX_X_NORMALIZED;

  // create an image of the same size:
  struct image_t *img_undistorted;
  image_create(img_undistorted, img->w, img->h, img->type);

  uint8_t *source = (uint8_t *)img->buf;
  uint8_t *dest = (uint8_t *)img_undistorted->buf;

  float x_pd, y_pd;
  uint16_t x_pd_ind, y_pd_ind;
  uint16_t x = 0;
  for(float x_n = UNDISTORT_MIN_X_NORMALIZED; x_n < UNDISTORT_MAX_X_NORMALIZED; x_n += normalized_step, x++) {
    uint16_t y = 0;
    for(float y_n = min_y_normalized; y_n < max_y_normalized; y_n += normalized_step, y++) {
      normalized_coords_to_distorted_pixels(x_n, y_n, &x_pd, &y_pd, k, K);
      if(x_pd > 0.0f && y_pd > 0.0f) {
        x_pd_ind = (uint16_t) x_pd;
        y_pd_ind = (uint16_t) y_pd;
        if(x_pd_ind < img->w && y_pd_ind < img->h) {
          // Assuming UY VY (2 bytes per pixel, and U for even indices, V for odd indices)
          dest[y*img_undistorted->w*2+x*2] = 128; // source[y_pd_ind*img->w*2+x_pd_ind*2]; // Colors will be a pain for undistortion...
          dest[y*img_undistorted->w*2+x*2+1] = source[y_pd_ind*img->w*2+x_pd_ind*2+1]; // no interpolation or anything, just the basics for now.
        }
      }
    }
  }

  image_copy(img_undistorted, img);
  image_free(img_undistorted);

  return img; // Colorfilter did not make a new image
}

void undistort_image_init(void)
{
  k = UNDISTORT_DHANE_K;
  listener = cv_add_to_device(&UNDISTORT_CAMERA, undistort_image_func, UNDISTORT_FPS);
}

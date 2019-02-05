/*
 * Copyright (C) 2015 Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/video_capture.c
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "modules/computer_vision/video_capture.h"
#include "modules/computer_vision/cv.h"

#include "lib/encoding/jpeg.h"

// Note: this define is set automatically when the video_exif module is included,
// and exposes functions to write data in the image exif headers.
#if JPEG_WITH_EXIF_HEADER
#include "lib/exif/exif_module.h"
#endif

#ifndef VIDEO_CAPTURE_PATH
#define VIDEO_CAPTURE_PATH /data/video/images
#endif

#ifndef VIDEO_CAPTURE_JPEG_QUALITY
#define VIDEO_CAPTURE_JPEG_QUALITY 99  ///<Default to minimum compression
#endif

#ifndef VIDEO_CAPTURE_FPS
#define VIDEO_CAPTURE_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(VIDEO_CAPTURE_FPS)

// Module settings
bool video_capture_take_shot = false;
int video_capture_index = 0;

// Forward function declarations
struct image_t *video_capture_func(struct image_t *img);
void video_capture_save(struct image_t *img);


void video_capture_init(void)
{
  // Create the images directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIDEO_CAPTURE_PATH));
  if (system(save_name) != 0) {
    printf("[video_capture] Could not create images directory %s.\n", STRINGIFY(VIDEO_CAPTURE_PATH));
    return;
  }

  // Add function to computer vision pipeline
  cv_add_to_device(&VIDEO_CAPTURE_CAMERA, video_capture_func, VIDEO_CAPTURE_FPS);
}


struct image_t *video_capture_func(struct image_t *img)
{
  // If take_shot bool is set, save the image
  if (video_capture_take_shot) {
    video_capture_save(img);
    video_capture_take_shot = false;
  }

  // No modification to image
  return NULL;
}


void video_capture_shoot(void)
{
  // Set take_shot bool to true
  video_capture_take_shot = true;
}

void video_capture_save(struct image_t *img)
{
  // Declare storage for image location
  char save_name[128];

  // Simple shot counter to find first available image location
  for (/* no init */; video_capture_index < 9999; ++video_capture_index) { //why 9999 evil? int fits more ;)
    // Generate image location
    sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(VIDEO_CAPTURE_PATH), video_capture_index);

    // Continue with next number if file exists already
    if (access(save_name, F_OK) != -1) {
      continue;
    }

    printf("[video_capture] Saving image to %s.\n", save_name);

    // Create jpg image from raw frame
    struct image_t img_jpeg;
    image_create(&img_jpeg, img->w, img->h, IMAGE_JPEG);
    jpeg_encode_image(img, &img_jpeg, VIDEO_CAPTURE_JPEG_QUALITY, true);

#if JPEG_WITH_EXIF_HEADER
    write_exif_jpeg(save_name, img_jpeg.buf, img_jpeg.buf_size, img_jpeg.w, img_jpeg.h);
#else
    // Open file
    FILE *fp = fopen(save_name, "w");
    if (fp == NULL) {
      printf("[video_capture] Could not write shot %s.\n", save_name);
      break;
    }

    // Save it to the file and close it
    fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
    fclose(fp);
#endif

    // Free image
    image_free(&img_jpeg);

    // End loop here
    break;
  }
}

/*
 * Copyright (C) 2015 Christophe De Wagter
 * Copyright (C) 2016 Roland Meertens
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/computer_vision/video_usb_logger.c
 */

#include "video_usb_logger.h"

#include <stdio.h>
#include "state.h"
#include "viewvideo.h"
#include "cv.h"
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "computer_vision/lib/encoding/jpeg.h"
#include "pose_history/pose_history.h"

/** Set the default File logger path to the USB drive */
#ifndef VIDEO_USB_LOGGER_PATH
#define VIDEO_USB_LOGGER_PATH /data/video/usb
#endif

#ifndef VIDEO_USB_LOGGER_FPS
#define VIDEO_USB_LOGGER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(VIDEO_USB_LOGGER_FPS)

/** The file pointer */
static FILE *video_usb_logger = NULL;
struct image_t img_jpeg_global;
bool created_jpeg = FALSE;
char foldername[512];
int shotNumber = 0;

static void save_shot_on_disk(struct image_t *img, struct image_t *img_jpeg)
{

  // Search for a file where we can write to
  char save_name[128];

  snprintf(save_name, sizeof(save_name), "%s/img_%05d.jpg", foldername, shotNumber);

  shotNumber++;
  // Check if file exists or not
  if (access(save_name, F_OK) == -1) {

    // Create a high quality image (99% JPEG encoded)
    jpeg_encode_image(img, img_jpeg, 99, TRUE);

#if VIDEO_USB_LOGGER_JPEG_WITH_EXIF_HEADER
    write_exif_jpeg(save_name, img_jpeg->buf, img_jpeg->buf_size, img_jpeg->w, img_jpeg->h);
#else
    FILE *fp = fopen(save_name, "w");
    if (fp == NULL) {
      printf("[video_thread-thread] Could not write shot %s.\n", save_name);
    } else {
      // Save it to the file and close it
      fwrite(img_jpeg->buf, sizeof(uint8_t), img_jpeg->buf_size, fp);
      fclose(fp);
      printf("Wrote image\n");
    }
#endif



    /** Log the values to a csv file */
    if (video_usb_logger == NULL) {
      return;
    }

    static uint32_t counter = 0;
    struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
    struct NedCoor_i *ned = stateGetPositionNed_i();
    struct NedCoor_i *accel = stateGetAccelNed_i();
    static uint32_t sonar = 0;


    // Save current information to a file
    fprintf(video_usb_logger, "%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d\n", counter,
            shotNumber,
            pose.eulers.phi, pose.eulers.theta, pose.eulers.psi,
            ned->x, ned->y, ned->z,
            accel->x, accel->y, accel->z,
            pose.rates.p, pose.rates.q, pose.rates.r,
            sonar);
    counter++;
  }

}

static struct image_t *log_image(struct image_t *img, uint8_t camera_id)
{
  if (!created_jpeg) {

    // Create the jpeg image used later
    image_create(&img_jpeg_global, img->w, img->h, IMAGE_JPEG);
    created_jpeg = TRUE;
  }
  save_shot_on_disk(img, &img_jpeg_global);
  return img;
}

/** Start the file logger and open a new file */
void video_usb_logger_start(void)
{

  uint32_t counter = 0;
  char filename[512];
  struct stat st = {0};

  // Search and create a new folder
  do {
    snprintf(foldername, sizeof(foldername), "%s/pprzvideo%05d", STRINGIFY(VIDEO_USB_LOGGER_PATH), counter);
    counter++;
  } while (stat(foldername, &st) >= 0);

  mkdir(foldername, 0700);

// In this folder create a textlog
  snprintf(filename, sizeof(filename), "%s/log.csv", foldername);
  video_usb_logger = fopen(filename, "w");

  if (video_usb_logger != NULL) {
    fprintf(video_usb_logger, "counter,image,roll,pitch,yaw,x,y,z,accelx,accely,accelz,ratep,rateq,rater,sonar\n");
  }

  // Subscribe to a camera
  cv_add_to_device(&VIDEO_USB_LOGGER_CAMERA, log_image, VIDEO_USB_LOGGER_FPS, 0);
}

/** Stop the logger an nicely close the file */
void video_usb_logger_stop(void)
{
  if (video_usb_logger != NULL) {
    fclose(video_usb_logger);
    video_usb_logger = NULL;
  }
}

void video_usb_logger_periodic(void)
{

}

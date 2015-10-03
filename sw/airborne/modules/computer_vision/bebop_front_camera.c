/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/bebop_front_camera.c
 */

// Own header
#include "modules/computer_vision/bebop_front_camera.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/bayer.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

// Threaded computer vision
#include <pthread.h>

#define MT9F002_WIDTH 1408
#define MT9F002_HEIGHT 2112
#define BEBOP_FRONT_CAMERA_WIDTH 272
#define BEBOP_FRONT_CAMERA_HEIGHT 272

// The place where the shots are saved (without slash on the end)
#ifndef BEBOP_FRONT_CAMERA_SHOT_PATH
#define BEBOP_FRONT_CAMERA_SHOT_PATH /data/ftp/internal_000/images
#endif
PRINT_CONFIG_VAR(BEBOP_FRONT_CAMERA_SHOT_PATH)

// Main thread
static void *bebop_front_camera_thread(void *data);
static void bebop_front_camera_save_shot(struct image_t *img_color, struct image_t *img_jpeg, struct image_t *raw_img);
void bebop_front_camera_periodic(void) { }

// Initialize the bebop_front_camera structure with the defaults
struct bebopfrontcamera_t bebop_front_camera = {
  .is_streaming = FALSE,
  .take_shot = FALSE,
  .shot_number = 0,
  .take_shot = FALSE,
  .shot_number = 0
};

void bebop_front_camera_take_shot(bool_t take)
{
  bebop_front_camera.take_shot = TRUE;
}
/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *bebop_front_camera_thread(void *data __attribute__((unused)))
{
  struct UdpSocket video_sock;
  udp_socket_create(&video_sock, STRINGIFY(BEBOP_FRONT_CAMERA_HOST), BEBOP_FRONT_CAMERA_PORT_OUT, -1,
                    BEBOP_FRONT_CAMERA_BROADCAST);

  // Create the JPEG encoded image
  struct image_t img_jpeg;
  image_create(&img_jpeg, BEBOP_FRONT_CAMERA_WIDTH, BEBOP_FRONT_CAMERA_HEIGHT, IMAGE_JPEG);

  // Create the Color image
  struct image_t img_color;
  image_create(&img_color, BEBOP_FRONT_CAMERA_WIDTH, BEBOP_FRONT_CAMERA_HEIGHT, IMAGE_YUV422);

  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(bebop_front_camera.dev)) {
    printf("[bebop_front_camera-thread] Could not start capture of %s.\n", bebop_front_camera.dev->name);
    return 0;
  }

  // Start streaming
  bebop_front_camera.is_streaming = TRUE;
  while (bebop_front_camera.is_streaming) {
    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(bebop_front_camera.dev, &img);

    BayerToYUV(&img, &img_color, 0, 0);

    if (bebop_front_camera.take_shot) {
      // Save the image
      bebop_front_camera_save_shot(&img_color, &img_jpeg, &img);
      bebop_front_camera.take_shot = FALSE;
    }

    jpeg_encode_image(&img_color, &img_jpeg, 80, 0);

    // Send image with RTP
    rtp_frame_send(
      &video_sock,              // UDP socket
      &img_jpeg,
      0,                        // Format 422
      80, // Jpeg-Quality
      0,                        // DRI Header
      0    // 90kHz time increment
    );


    // Free the image
    v4l2_image_free(bebop_front_camera.dev, &img);
  }

  return 0;
}

/**
 * Initialize the view video
 */
void bebop_front_camera_init(void)
{
  // Initialize the V4L2 subdevice (TODO: fix hardcoded path, which and code)
  if (!v4l2_init_subdev("/dev/v4l-subdev1", 0, 0, V4L2_MBUS_FMT_SGBRG10_1X10, MT9F002_WIDTH, MT9F002_HEIGHT)) {
    printf("[bebop_front_camera] Could not initialize the v4l-subdev1 subdevice.\n");
    return;
  }

  // Initialize the V4L2 device
  bebop_front_camera.dev = v4l2_init("/dev/video1", MT9F002_WIDTH, MT9F002_HEIGHT, 10, V4L2_PIX_FMT_SGBRG10);
  if (bebop_front_camera.dev == NULL) {
    printf("[bebop_front_camera] Could not initialize the /dev/video1 V4L2 device.\n");
    return;
  }

  // Create the shot directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(BEBOP_FRONT_CAMERA_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[bebop_front_camera] Could not create shot directory %s.\n", STRINGIFY(BEBOP_FRONT_CAMERA_SHOT_PATH));
    return;
  }
}

/**
 * Start with streaming
 */
void bebop_front_camera_start(void)
{
  // Check if we are already running
  if (bebop_front_camera.is_streaming) {
    return;
  }

  // Start the streaming thread
  pthread_t tid;
  if (pthread_create(&tid, NULL, bebop_front_camera_thread, NULL) != 0) {
    printf("[vievideo] Could not create streaming thread.\n");
    return;
  }
}

/**
 * Stops the streaming
 * This could take some time, because the thread is stopped asynchronous.
 */
void bebop_front_camera_stop(void)
{
  // Check if not already stopped streaming
  if (!bebop_front_camera.is_streaming) {
    return;
  }

  // Stop the streaming thread
  bebop_front_camera.is_streaming = FALSE;

  // Stop the capturing
  if (!v4l2_stop_capture(bebop_front_camera.dev)) {
    printf("[bebop_front_camera] Could not stop capture of %s.\n", bebop_front_camera.dev->name);
    return;
  }

  // TODO: wait for the thread to finish to be able to start the thread again!
}

static void bebop_front_camera_save_shot(struct image_t *img, struct image_t *img_jpeg, struct image_t *raw_img)
{

  // Search for a file where we can write to
  char save_name[128];
  for (; bebop_front_camera.shot_number < 99999; bebop_front_camera.shot_number++) {
    sprintf(save_name, "%s/img_%05d.pgm", STRINGIFY(BEBOP_FRONT_CAMERA_SHOT_PATH), bebop_front_camera.shot_number);
    // Check if file exists or not
    if (access(save_name, F_OK) == -1) {
      FILE *fp = fopen(save_name, "w");
      if (fp == NULL) {
        printf("[bebop_front_camera-thread] Could not write shot %s.\n", save_name);
      } else {
        // Save it to the file and close it
        char pgm_header[] = "P5\n272 272\n255\n";
        fwrite(pgm_header, sizeof(char), 15, fp);
        fwrite(img->buf, sizeof(uint8_t), img->buf_size, fp);
        fclose(fp);

        // JPEG
        jpeg_encode_image(img, img_jpeg, 99, 1);
        sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(BEBOP_FRONT_CAMERA_SHOT_PATH), bebop_front_camera.shot_number);
        fp = fopen(save_name, "w");
        fwrite(img_jpeg->buf, sizeof(uint8_t), img_jpeg->buf_size, fp);
        fclose(fp);

        sprintf(save_name, "%s/img_%05d.raw", STRINGIFY(BEBOP_FRONT_CAMERA_SHOT_PATH), bebop_front_camera.shot_number);
        fp = fopen(save_name, "w");
        fwrite(raw_img->buf, sizeof(uint8_t), raw_img->buf_size, fp);
        fclose(fp);

      }

      // We don't need to seek for a next index anymore
      break;
    }
  }
}

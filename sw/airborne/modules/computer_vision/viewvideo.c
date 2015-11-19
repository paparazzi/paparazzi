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
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"
#include "modules/computer_vision/cv.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 4
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Default image folder
#ifndef VIEWVIDEO_SHOT_PATH
#ifdef VIDEO_THREAD_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH VIDEO_THREAD_SHOT_PATH
#else
#define VIEWVIDEO_SHOT_PATH /data/video/images
#endif
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif

#if !VIEWVIDEO_USE_NETCAT && !(defined VIEWVIDEO_USE_RTP)
#define VIEWVIDEO_USE_RTP TRUE
#endif

#if VIEWVIDEO_USE_NETCAT
#include <sys/wait.h>
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
struct UdpSocket video_sock;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
#if !VIEWVIDEO_USE_NETCAT
  .use_rtp = VIEWVIDEO_USE_RTP,
#endif
};

/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
bool_t viewvideo_function(struct image_t *img);
bool_t viewvideo_function(struct image_t *img)
{
  // Resize image if needed
  struct image_t img_small;
  image_create(&img_small,
               img->w / viewvideo.downsize_factor,
               img->h / viewvideo.downsize_factor,
               IMAGE_YUV422);

  // Create the JPEG encoded image
  struct image_t img_jpeg;
  image_create(&img_jpeg, img_small.w, img_small.h, IMAGE_JPEG);

#if VIEWVIDEO_USE_NETCAT
  char nc_cmd[64];
  sprintf(nc_cmd, "nc %s %d 2>/dev/null", STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT);
#endif

  if (viewvideo.is_streaming) {

    // Only resize when needed
    if (viewvideo.downsize_factor != 1) {
      image_yuv422_downsample(img, &img_small, viewvideo.downsize_factor);
      jpeg_encode_image(&img_small, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    } else {
      jpeg_encode_image(img, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    }

#if VIEWVIDEO_USE_NETCAT
    // Open process to send using netcat (in a fork because sometimes kills itself???)
    pid_t pid = fork();

    if (pid < 0) {
      printf("[viewvideo] Could not create netcat fork.\n");
    } else if (pid == 0) {
      // We are the child and want to send the image
      FILE *netcat = popen(nc_cmd, "w");
      if (netcat != NULL) {
        fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, netcat);
        pclose(netcat); // Ignore output, because it is too much when not connected
      } else {
        printf("[viewvideo] Failed to open netcat process.\n");
      }

      // Exit the program since we don't want to continue after transmitting
      exit(0);
    } else {
      // We want to wait until the child is finished
      wait(NULL);
    }
#else
    if (viewvideo.use_rtp) {

      // Send image with RTP
      rtp_frame_send(
        &video_sock,              // UDP socket
        &img_jpeg,
        0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        VIEWVIDEO_RTP_TIME_INC    // 90kHz time increment
      );
      // Extra note: when the time increment is set to 0,
      // it is automaticaly calculated by the send_rtp_frame function
      // based on gettimeofday value. This seems to introduce some lag or jitter.
      // An other way is to compute the time increment and set the correct value.
      // It seems that a lower value is also working (when the frame is received
      // the timestamp is always "late" so the frame is displayed immediately).
      // Here, we set the time increment to the lowest possible value
      // (1 = 1/90000 s) which is probably stupid but is actually working.
    }
#endif

  }

  // Free all buffers
  image_free(&img_jpeg);
  image_free(&img_small);
  return TRUE;
}

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  char save_name[512];

  cv_add(viewvideo_function);

  viewvideo.is_streaming = TRUE;

#if VIEWVIDEO_USE_NETCAT
  // Create an Netcat receiver file for the streaming
  sprintf(save_name, "%s/netcat-recv.sh", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "i=0\n");
    fprintf(fp, "while true\n");
    fprintf(fp, "do\n");
    fprintf(fp, "\tn=$(printf \"%%04d\" $i)\n");
    fprintf(fp, "\tnc -l 0.0.0.0 %d > img_${n}.jpg\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "\ti=$((i+1))\n");
    fprintf(fp, "done\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create netcat receiver file.\n");
  }
#else
  // Open udp socket
  udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);

  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "v=0\n");
    fprintf(fp, "m=video %d RTP/AVP 26\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "c=IN IP4 0.0.0.0\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create SDP file.\n");
  }
#endif
}


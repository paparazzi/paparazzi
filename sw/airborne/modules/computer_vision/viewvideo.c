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

#include BOARD_CONFIG

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

// Define stream framerate
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Define stream priority
#ifndef VIEWVIDEO_NICE_LEVEL
#define VIEWVIDEO_NICE_LEVEL 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_NICE_LEVEL)

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
struct UdpSocket video_sock1;
struct UdpSocket video_sock2;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT2_OUT)

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
 * This is a separate thread, so it needs to be thread safe!
 */
static struct image_t *viewvideo_function(struct UdpSocket *viewvideo_socket, struct image_t *img, uint16_t *rtp_packet_nr, uint32_t *rtp_frame_time,
    struct image_t *img_small, struct image_t *img_jpeg)
{
  // Resize small image if needed
  if(img_small->buf_size != img->buf_size/(viewvideo.downsize_factor*viewvideo.downsize_factor)){
    if(img_small->buf != NULL){
      image_free(img_small);
    }
    image_create(img_small,
                 img->w / viewvideo.downsize_factor,
                 img->h / viewvideo.downsize_factor,
                 IMAGE_YUV422);
  }

  // Resize JPEG encoded image if needed
  if(img_jpeg->w != img_small->w || img_jpeg->h != img_small->h)
  {
    if(img_jpeg->buf != NULL){
      image_free(img_jpeg);
    }
    image_create(img_jpeg, img_small->w, img_small->h, IMAGE_JPEG);
  }

#if VIEWVIDEO_USE_NETCAT
  char nc_cmd[64];
  sprintf(nc_cmd, "nc %s %d 2>/dev/null", STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT);
#endif

  if (viewvideo.is_streaming) {
    // Only resize when needed
    if (viewvideo.downsize_factor != 1) {
      image_yuv422_downsample(img, img_small, viewvideo.downsize_factor);
      jpeg_encode_image(img_small, img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    } else {
      jpeg_encode_image(img, img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
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
        fwrite(img_jpeg->buf, sizeof(uint8_t), img_jpeg->buf_size, netcat);
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
        viewvideo_socket,         // UDP socket
        img_jpeg,
        0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        VIEWVIDEO_FPS,
        //(img->ts.tv_sec * 1000000 + img->ts.tv_usec),
        rtp_packet_nr,
        rtp_frame_time
      );
    }
#endif
  }

  return NULL; // No new images were created
}

#ifdef VIEWVIDEO_CAMERA
static struct image_t *viewvideo_function1(struct image_t *img)
{
  static uint16_t rtp_packet_nr = 0;
  static uint32_t rtp_frame_time = 0;
  static struct image_t img_small = {.buf=NULL, .buf_size=0};
  static struct image_t img_jpeg = {.buf=NULL, .buf_size=0};
  return viewvideo_function(&video_sock1, img, &rtp_packet_nr, &rtp_frame_time, &img_small, &img_jpeg);
}
#endif

#ifdef VIEWVIDEO_CAMERA2
static struct image_t *viewvideo_function2(struct image_t *img)
{
  static uint16_t rtp_packet_nr = 0;
  static uint32_t rtp_frame_time = 0;
  static struct image_t img_small = {.buf=NULL, .buf_size=0};
  static struct image_t img_jpeg = {.buf=NULL, .buf_size=0};
  return viewvideo_function(&video_sock2, img, &rtp_packet_nr, &rtp_frame_time, &img_small, &img_jpeg);
}
#endif

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  viewvideo.is_streaming = true;

  // safety check
  if(viewvideo.downsize_factor < 1){
    viewvideo.downsize_factor = 1;
  }

  if(viewvideo.quality_factor > 99){
    viewvideo.quality_factor = 99;
  }

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
#ifdef VIEWVIDEO_CAMERA
  if (udp_socket_create(&video_sock1, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST)) {
    printf("[viewvideo]: failed to open view video socket, HOST=%s, port=%d\n", STRINGIFY(VIEWVIDEO_HOST),
           VIEWVIDEO_PORT_OUT);
  }
#endif

#ifdef VIEWVIDEO_CAMERA2
  if (udp_socket_create(&video_sock2, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT2_OUT, -1, VIEWVIDEO_BROADCAST)) {
    printf("[viewvideo]: failed to open view video socket, HOST=%s, port=%d\n", STRINGIFY(VIEWVIDEO_HOST),
           VIEWVIDEO_PORT2_OUT);
  }
#endif
#endif

#ifdef VIEWVIDEO_CAMERA
  cv_add_to_device_async(&VIEWVIDEO_CAMERA, viewvideo_function1,
                         VIEWVIDEO_NICE_LEVEL, VIEWVIDEO_FPS);
  fprintf(stderr, "[viewvideo] Added asynchronous video streamer listener for CAMERA1 at %u FPS \n", VIEWVIDEO_FPS);
#endif

#ifdef VIEWVIDEO_CAMERA2
  cv_add_to_device_async(&VIEWVIDEO_CAMERA2, viewvideo_function2,
                         VIEWVIDEO_NICE_LEVEL, VIEWVIDEO_FPS);
  fprintf(stderr, "[viewvideo] Added asynchronous video streamer listener for CAMERA2 at %u FPS \n", VIEWVIDEO_FPS);
#endif
}

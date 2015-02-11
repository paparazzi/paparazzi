/*
 * Copyright (C) 2012-2014 The Paparazzi Community
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

// UDP RTP Images
#include "modules/computer_vision/lib/udp/socket.h"
// Video
#include "modules/computer_vision/lib/v4l/video.h"
#include "modules/computer_vision/cv/resize.h"
#include "modules/computer_vision/cv/encoding/jpeg.h"
#include "modules/computer_vision/cv/encoding/rtp.h"

// Threaded computer vision
#include <pthread.h>

// Default broadcast IP
#ifndef VIDEO_SOCK_IP
#define VIDEO_SOCK_IP "192.168.1.255"
#endif

// Output socket can be defined from an offset
#ifdef VIDEO_SOCK_OUT_OFFSET
#define VIDEO_SOCK_OUT (5000+VIDEO_SOCK_OUT_OFFSET)
#endif

#ifndef VIDEO_SOCK_OUT
#define VIDEO_SOCK_OUT 5000
#endif

#ifndef VIDEO_SOCK_IN
#define VIDEO_SOCK_IN 4999
#endif

// Downsize factor for video stream
#ifndef VIDEO_DOWNSIZE_FACTOR
#define VIDEO_DOWNSIZE_FACTOR 4
#endif

// From 0 to 99 (99=high)
#ifndef VIDEO_QUALITY_FACTOR
#define VIDEO_QUALITY_FACTOR 50
#endif

// Frame Per Seconds
#ifndef VIDEO_FPS
#define VIDEO_FPS 4.
#endif

void viewvideo_run(void) {}

// take shot flag
int viewvideo_shot = 0;
volatile int viewvideo_save_shot_number = 0;


/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void *data);
void *computervision_thread_main(void *data)
{
  // Create a V4L2 device
#if USE_BOTTOM_CAMERA
  struct v4l2_device *dev = v4l2_init("/dev/video2", 320, 240, 10);
#else
  struct v4l2_device *dev = v4l2_init("/dev/video1", 1280, 720, 4);
#endif
  if (dev == NULL) {
    printf("Error initialising video\n");
    return 0;
  }


  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(dev)) {
    printf("Could not start capture\n");
    return 0;
  }

  // Video Resizing
  uint8_t quality_factor = VIDEO_QUALITY_FACTOR;
  uint8_t dri_jpeg_header = 0;
  int microsleep = (int)(1000000. / VIDEO_FPS);

  struct img_struct small;
  small.w = dev->w / VIDEO_DOWNSIZE_FACTOR;
  small.h = dev->h / VIDEO_DOWNSIZE_FACTOR;
  small.buf = (uint8_t *)malloc(small.w * small.h * 2);

  // Video Compression
  uint8_t *jpegbuf = (uint8_t *)malloc(dev->h * dev->w * 2);

  // Network Transmit
  struct UdpSocket *vsock;
  vsock = udp_socket(VIDEO_SOCK_IP, VIDEO_SOCK_OUT, VIDEO_SOCK_IN, FMS_BROADCAST);

  // Create SPD file and make folder if necessary
  FILE *sdp;
  if (system("mkdir -p /data/video/sdp") == 0) {
    sdp = fopen("/data/video/sdp/x86_config-mjpeg.sdp", "w");
    if (sdp != NULL) {
      fprintf(sdp, "v=0\n");
      fprintf(sdp, "m=video %d RTP/AVP 26\n", (int)(VIDEO_SOCK_OUT));
      fprintf(sdp, "c=IN IP4 0.0.0.0");
      fclose(sdp);
    }
  }

  // file index (search from 0)
  int file_index = 0;

  // time
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

  while (computer_vision_thread_command > 0) {
    // compute usleep to have a more stable frame rate
    struct timeval vision_thread_sleep_time;
    gettimeofday(&vision_thread_sleep_time, NULL);
    int dt = (int)(vision_thread_sleep_time.tv_sec - last_time.tv_sec) * 1000000 + (int)(vision_thread_sleep_time.tv_usec - last_time.tv_usec);
    if (dt < microsleep) { usleep(microsleep - dt); }
    last_time = vision_thread_sleep_time;

    // Wait for a new frame
    struct v4l2_img_buf *img = v4l2_image_get(dev);

    // Save picture on disk
    if (computer_vision_thread_command == 2) {
      uint8_t *end = encode_image(img->buf, jpegbuf, 99, FOUR_TWO_TWO, dev->w, dev->h, 1);
      uint32_t size = end - (jpegbuf);
      FILE *save;
      char save_name[128];
#if LOG_ON_USB
      if (system("mkdir -p /data/video/usb/images") == 0) {
#else
      if (system("mkdir -p /data/video/images") == 0) {
#endif
        // search available index (max is 99)
        for (; file_index < 99999; file_index++) {
          printf("search %d\n", file_index);
#if LOG_ON_USB
          sprintf(save_name, "/data/video/usb/images/img_%05d.jpg", file_index);
#else
          sprintf(save_name, "/data/video/images/img_%05d.jpg", file_index);
#endif
          // test if file exists or not
          if (access(save_name, F_OK) == -1) {
            printf("access\n");
            save = fopen(save_name, "w");
            if (save != NULL) {
              // Atomic copy
              viewvideo_save_shot_number = file_index;
              fwrite(jpegbuf, sizeof(uint8_t), size, save);
              fclose(save);
            } else {
              printf("Error when opening file %s\n", save_name);
            }
            // leave for loop
            break;
          } else {
            //printf("file exists\n");
          }
        }
      }
      if (computer_vision_thread_command == 2)
        computer_vision_thread_command = 1;
      viewvideo_shot = 0;
    }

    // Resize
    struct img_struct input;
    input.buf = img->buf;
    input.w = dev->w;
    input.h = dev->h;
    resize_uyuv(&input, &small, VIDEO_DOWNSIZE_FACTOR);

    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t *end = encode_image(small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end - (jpegbuf);

    // Send image with RTP
    printf("Sending an image ...%u\n", size);
    send_rtp_frame(
      vsock,            // UDP
      jpegbuf, size,    // JPEG
      small.w, small.h, // Img Size
      0,                // Format 422
      quality_factor,   // Jpeg-Quality
      dri_jpeg_header,  // DRI Header
      0                 // 90kHz time increment
    );
    // Extra note: when the time increment is set to 0,
    // it is automaticaly calculated by the send_rtp_frame function
    // based on gettimeofday value. This seems to introduce some lag or jitter.
    // An other way is to compute the time increment and set the correct value.
    // It seems that a lower value is also working (when the frame is received
    // the timestamp is always "late" so the frame is displayed immediately).
    // Here, we set the time increment to the lowest possible value
    // (1 = 1/90000 s) which is probably stupid but is actually working.

    // Free the image
    v4l2_image_free(dev, img);
  }
  printf("Thread Closed\n");
  v4l2_close(dev);
  computervision_thread_status = -100;
  return 0;
}

void viewvideo_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if (rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void viewvideo_stop(void)
{
  computer_vision_thread_command = 0;
}

int viewvideo_save_shot(void)
{
  if (computer_vision_thread_command > 0) {
    computer_vision_thread_command = 2;
  }
  return 0;
}



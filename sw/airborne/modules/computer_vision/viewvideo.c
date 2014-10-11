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

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  uint8_t quality_factor = VIDEO_QUALITY_FACTOR;
  uint8_t dri_jpeg_header = 0;
  int microsleep = (int)(1000000. / VIDEO_FPS);

  struct img_struct small;
  small.w = vid.w / VIDEO_DOWNSIZE_FACTOR;
  small.h = vid.h / VIDEO_DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  struct UdpSocket* vsock;
  vsock = udp_socket(VIDEO_SOCK_IP, VIDEO_SOCK_OUT, VIDEO_SOCK_IN, FMS_BROADCAST);

  // Create SPD file and make folder if necessary
  FILE* sdp;
  if (system("mkdir -p /data/video/sdp") == 0) {
    sdp = fopen("/data/video/sdp/x86_config-mjpeg.sdp","w");
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

  while (computer_vision_thread_command > 0)
  {
    // compute usleep to have a more stable frame rate
    struct timeval time;
    gettimeofday(&time, NULL);
    int dt = (int)(time.tv_sec - last_time.tv_sec)*1000000 + (int)(time.tv_usec - last_time.tv_usec);
    if (dt < microsleep) usleep(microsleep - dt);
    last_time = time;

    // Grab new frame
    video_grab_image(&vid, img_new);

    // Save picture on disk
    if (computer_vision_thread_command == 2)
    {
      uint8_t* end = encode_image (img_new->buf, jpegbuf, 99, FOUR_TWO_TWO, vid.w, vid.h, 1);
      uint32_t size = end-(jpegbuf);
      FILE* save;
      char save_name[128];
      if (system("mkdir -p /data/video/images") == 0) {
        // search available index (max is 99)
        for ( ; file_index < 99; file_index++) {
          printf("search %d\n",file_index);
          sprintf(save_name,"/data/video/images/img_%02d.jpg",file_index);
          // test if file exists or not
          if (access(save_name, F_OK) == -1) {
            printf("access\n");
            save = fopen(save_name, "w");
            if (save != NULL) {
              fwrite(jpegbuf, sizeof(uint8_t), size, save);
              fclose(save);
            }
            else {
              printf("Error when opening file %s\n", save_name);
            }
            // leave for loop
            break;
          } else {
            printf("file exists\n");
          }
        }
      }
      computer_vision_thread_command = 1;
      viewvideo_shot = 0;
    }

    // Resize
    resize_uyuv(img_new, &small, VIDEO_DOWNSIZE_FACTOR);

    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end - (jpegbuf);

    // Send image with RTP
    printf("Sending an image ...%u\n",size);
    send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        small.w, small.h, // Img Size
        0,                // Format 422
        quality_factor,   // Jpeg-Quality
        dri_jpeg_header,  // DRI Header
        1                 // 90kHz time increment
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
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void viewvideo_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
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



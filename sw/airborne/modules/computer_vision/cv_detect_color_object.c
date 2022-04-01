/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "opencv_color_edges.h"
#include "modules/computer_vision/obstacle_message.h"

#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"


#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif


// Filter Settings
bool GRAY_SCALE1 = true;
bool BLUR_IMAGE1 = true;
uint8_t  BLUR_SIZE_IMAGE1 = 3;
bool BLUR_EDGES1 = true;
uint8_t  BLUR_SIZE_EDGES1 = 3;
bool BORDERS1 = true;
uint8_t  BORDER_MARGIN1 = 3;
bool Y_UP_filter1 = true;
int  y_up_del1 = 40;
bool Y_DOWN_filter1 = true;
int  y_down_del1 = 200;
uint8_t  thresholdmin1 = 100;
uint8_t  thresholdmax1 = 200;
uint8_t  kernal_size1 = 3;
double min_obs_size1 = 0.04;
double max_obs_size1 = 0.4;

bool GRAY_SCALE2 = true;
bool BLUR_IMAGE2 = true;
uint8_t  BLUR_SIZE_IMAGE2 = 3;
bool BLUR_EDGES2 = true;
uint8_t  BLUR_SIZE_EDGES2 = 3;
bool BORDERS2 = true;
uint8_t  BORDER_MARGIN2 = 3;
bool Y_UP_filter2 = true;
int  y_up_del2 = 40;
bool Y_DOWN_filter2 = true;
int  y_down_del2 = 40;
uint8_t  thresholdmin2 = 100;
uint8_t  thresholdmax2 = 200;
uint8_t  kernal_size2 = 3;
double min_obs_size2 = 0.04;
double max_obs_size2 = 0.4;

bool cod_draw1 = false;
bool cod_draw2 = false;

const uint8_t  max_number_obsticals = 5;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

// Added global obstacle
struct obstacle global_obstacles[5]; // = max_number_obsticals = 5

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
	bool GRAY_SCALE;
	bool BLUR_IMAGE;
	uint8_t  BLUR_SIZE_IMAGE;
	bool BLUR_EDGES;
	uint8_t  BLUR_SIZE_EDGES;
	bool BORDERS;
	uint8_t  BORDER_MARGIN;
	bool Y_UP_filter;
	int  y_up_del;
	bool Y_DOWN_filter;
	int  y_down_del;
	uint8_t  thresholdmin;
	uint8_t  thresholdmax;
	uint8_t  kernal_size;
	double min_obs_size;
	double max_obs_size;
	bool draw;

  switch (filter){
  	  case 1:
		GRAY_SCALE = GRAY_SCALE1;
		BLUR_IMAGE = BLUR_IMAGE1;
		BLUR_SIZE_IMAGE = BLUR_SIZE_IMAGE1;
		BLUR_EDGES = BLUR_EDGES1;
		BLUR_SIZE_EDGES = BLUR_SIZE_EDGES1;
		BORDERS = BORDERS1;
		BORDER_MARGIN = BORDER_MARGIN1;
		Y_UP_filter = Y_UP_filter1;
		y_up_del = y_up_del1;
		Y_DOWN_filter = Y_DOWN_filter1;
		y_down_del = y_down_del1;
		thresholdmin = thresholdmin1;
		thresholdmax = thresholdmax1;
		kernal_size = kernal_size1;
		min_obs_size = min_obs_size1;
		max_obs_size = max_obs_size1;
      break;
      case 2:
    	GRAY_SCALE = GRAY_SCALE2;
		BLUR_IMAGE = BLUR_IMAGE2;
		BLUR_SIZE_IMAGE = BLUR_SIZE_IMAGE2;
		BLUR_EDGES = BLUR_EDGES2;
		BLUR_SIZE_EDGES = BLUR_SIZE_EDGES2;
		BORDERS = BORDERS2;
		BORDER_MARGIN = BORDER_MARGIN2;
		Y_UP_filter = Y_UP_filter2;
		y_up_del = y_up_del2;
		Y_DOWN_filter = Y_DOWN_filter2;
		y_down_del = y_down_del2;
		thresholdmin = thresholdmin2;
		thresholdmax = thresholdmax2;
		kernal_size1 = kernal_size2;
		min_obs_size = min_obs_size2;
		max_obs_size = max_obs_size2;
      break;
    default:
      return img;
  };

  int downsize = 1;

  struct obstacle new_obstacle[max_number_obsticals];

  opencv_color_edges(new_obstacle,img,GRAY_SCALE,
			BLUR_IMAGE,BLUR_SIZE_IMAGE,
			BLUR_EDGES,BLUR_SIZE_EDGES,
			BORDERS,BORDER_MARGIN,
			Y_UP_filter,y_up_del,
			Y_DOWN_filter,y_down_del,
			thresholdmin,thresholdmax,
			kernal_size,
			max_number_obsticals,
			draw,
			downsize,
			min_obs_size,max_obs_size);

  pthread_mutex_lock(&mutex);

  for (int i = 0; i < max_number_obsticals; i++) {
	  global_obstacles[i].pos_x = new_obstacle[i].pos_x;
	  global_obstacles[i].pos_y = new_obstacle[i].pos_y;
	  global_obstacles[i].width = new_obstacle[i].width;
	  global_obstacles[i].height = new_obstacle[i].height;
	  global_obstacles[i].updated = true;
  }
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_GRAY_SCALE1
  	GRAY_SCALE1 = COLOR_OBJECT_DETECTOR_GRAY_SCALE1;
	BLUR_IMAGE1 = COLOR_OBJECT_DETECTOR_BLUR_IMAGE1;
	BLUR_SIZE_IMAGE1 = COLOR_OBJECT_DETECTOR_BLUR_SIZE_IMAGE1;
	BLUR_EDGES1 = COLOR_OBJECT_DETECTOR_BLUR_EDGES1;
	BLUR_SIZE_EDGES1 = COLOR_OBJECT_DETECTOR_BLUR_SIZE_EDGES1;
	BORDERS1 = COLOR_OBJECT_DETECTOR_BORDERS1;
	BORDER_MARGIN1 = COLOR_OBJECT_DETECTOR_BORDER_MARGIN1;
	Y_UP_filter1 = COLOR_OBJECT_DETECTOR_Y_UP_filter1;
	y_up_del1 = COLOR_OBJECT_DETECTOR_y_up_del1;
	Y_DOWN_filter1 = COLOR_OBJECT_DETECTOR_Y_DOWN_filter1;
	y_down_del1 = COLOR_OBJECT_DETECTOR_y_down_del1;
	thresholdmin1 = COLOR_OBJECT_DETECTOR_thresholdmin1;
	thresholdmax1 = COLOR_OBJECT_DETECTOR_thresholdmax1;
	kernal_size1 = COLOR_OBJECT_DETECTOR_kernal_size1;
	min_obs_size1 = COLOR_OBJECT_DETECTOR_min_obs_size1;
	max_obs_size1 = COLOR_OBJECT_DETECTOR_max_obs_size1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_GRAY_SCALE2
  	GRAY_SCALE2 = COLOR_OBJECT_DETECTOR_GRAY_SCALE2;
  	BLUR_IMAGE2 = COLOR_OBJECT_DETECTOR_BLUR_IMAGE2;
  	BLUR_SIZE_IMAGE2 = COLOR_OBJECT_DETECTOR_BLUR_SIZE_IMAGE2;
  	BLUR_EDGES2 = COLOR_OBJECT_DETECTOR_BLUR_EDGES2;
  	BLUR_SIZE_EDGES2 = COLOR_OBJECT_DETECTOR_BLUR_SIZE_EDGES2;
  	BORDERS2 = COLOR_OBJECT_DETECTOR_BORDERS2;
  	BORDER_MARGIN2 = COLOR_OBJECT_DETECTOR_BORDER_MARGIN2;
  	Y_UP_filter2 = COLOR_OBJECT_DETECTOR_Y_UP_filter2;
  	y_up_del2 = COLOR_OBJECT_DETECTOR_y_up_del2;
  	Y_DOWN_filter2 = COLOR_OBJECT_DETECTOR_Y_DOWN_filter2;
  	y_down_del2 = COLOR_OBJECT_DETECTOR_y_down_del2;
  	thresholdmin2 = COLOR_OBJECT_DETECTOR_thresholdmin2;
  	thresholdmax2 = COLOR_OBJECT_DETECTOR_thresholdmax2;
  	kernal_size2 = COLOR_OBJECT_DETECTOR_kernal_size2;
  	min_obs_size2 = COLOR_OBJECT_DETECTOR_min_obs_size2;
  	max_obs_size2 = COLOR_OBJECT_DETECTOR_max_obs_size2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      } else {
    	  if (draw){
    	   *yp = 0;
    	  }
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  return cnt;
}

void color_object_detector_periodic(void)
{

  struct ObstacleMessage obstacleMessage[max_number_obsticals];

  static struct color_object_t local_filters[2];
  static struct obstacle local_obstacles[5]; // max_number_obsticals = 5;
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  memcpy(local_obstacles, global_obstacles, max_number_obsticals*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(global_filters[0].updated){
	  for (unsigned int i = 0; i < max_number_obsticals; i++) {
		  obstacleMessage[i].pos_x = local_obstacles[i].pos_x;
		  obstacleMessage[i].pos_y = local_obstacles[i].pos_y;
		  obstacleMessage[i].obs_width = local_obstacles[i].width;
		  obstacleMessage[i].obs_height = local_obstacles[i].height;
		  obstacleMessage[i].quality = local_obstacles[i].area;
		  obstacleMessage[i].time2impact = -1;
	  }
	  uint32_t stamp = get_sys_time_usec();
	  AbiSendMsgPAYLOAD_DATA(2, stamp, 1, sizeof(obstacleMessage), &obstacleMessage);
  }

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}

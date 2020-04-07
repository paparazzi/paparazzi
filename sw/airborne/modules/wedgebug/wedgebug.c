/*
 * Copyright (C) Ralph Rudi schmidt <ralph.r.schmidt@outlook.com>

 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.

 */
/** @file "modules/wedgebug/wedgebug.h"
 * @author Ralph Rudi schmidt <ralph.r.schmidt@outlook.com>
 * An integration of the WegdeBug algorithm (Laubach 1999) for path finding, for drones with stereo vision.
 */
#include "modules/wedgebug/wedgebug.h"
#include <stdio.h>
#include "modules/computer_vision/cv.h" // Required for the "cv_add_to_device" function
#include "pthread.h"


// New section ----------------------------------------------------------------------------------------------------------------
// Defines
#ifndef WEDGEBUG_CAMERA_RIGHT_FPS
#define WEDGEBUG_CAMERA_RIGHT_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef WEDGEBUG_CAMERA_LEFT_FPS
#define WEDGEBUG_CAMERA_LEFT_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif



// New section ----------------------------------------------------------------------------------------------------------------
// Define global variables
struct image_t img_left;
struct image_t img_right;
struct image_t img_combined;
uint8_t buf_left[WEDGEBUG_BUF_SIZE];
uint8_t buf_right[WEDGEBUG_BUF_SIZE];
//static pthread_mutex_t mutex;



// New section ----------------------------------------------------------------------------------------------------------------
// Functions - declaration
static struct image_t *copy_left_img_func(struct image_t *img); // Function X: Copies left image into a buffer (buf_left)
static struct image_t *copy_right_img_func(struct image_t *img); // Function X: Copies left image into a buffer (buf_right)
const char* get_img_type(enum image_type img_type); // Function X: Displays image type
void show_image_data(struct image_t *img); // Function X: Displays image data



// New section ----------------------------------------------------------------------------------------------------------------
// Function - definition
// Function x=
const char* get_img_type(enum image_type img_type)
{
	switch(img_type)
	{
	case IMAGE_YUV422: return "IMAGE_YUV422";
	case IMAGE_GRAYSCALE: return "IMAGE_GRAYSCALE";
	case IMAGE_JPEG: return "IMAGE_JPEG";
	case IMAGE_GRADIENT: return "IMAGE_GRADIENT";
	default: return "Image type not found";
	}
}

void show_image_data(struct image_t *img)
{
	printf("Image-Type: %s\n", get_img_type(img->type));
	printf("Image-Width: %d\n", img->w);
	printf("Image-Height: %d\n", img->h);
	printf("Image-Buf_Size: %d\n", img->buf_size);
	printf("Image-Buf_Memory_Occupied: %lu\n", sizeof(img->buf));
}



static struct image_t *copy_left_img_func(struct image_t *img)
{
	//show_image_data(img);
	//enum image_type t = img->type;
	//printf("copy_left_img_func was called: %s\n", get_img_type(t));
	image_copy(img, &img_left);
	//img_left.buf = img ->buf;
	//int *int_array = (int*)img_left.buf;

	uint8_t img_left_buf[img->buf_size];



	for (uint32_t i = 0; i < img->buf_size; i++)
	{
		//printf("Value of i: %d\n", i);

		img_left_buf[i] = ((uint8_t*) img->buf)[i];

		//printf("Value of img.buf[%d]: %d\n",i, ((uint8_t*)img->buf)[i]);
		//printf("Value of img_left_buf[%d]: %d\n",i, img_left_buf[i]);
	}

	//printf("Value of i: %d\n", i);

	int j = ((int)img->buf_size) - 1;
	printf("img.buffer_size: %d\n", img->buf_size);
	printf("img.buffer_size: %d\n", img_left.buf_size);
	printf("Pixel %d value - img: %d\n", j ,((uint8_t*)img->buf)[j]);
	printf("Pixel %d value - img_left: %d\n", j ,((uint8_t*)img_left.buf)[j]);
	printf("Pixel %d value - img_left: %d\n\n", j ,img_left_buf[j]); //img_left.buf_size
	//pthread_mutex_lock (& mutex );
	//pthread_mutex_unlock (& mutex );

	return img;
}


// Function x
static struct image_t *copy_right_img_func(struct image_t *img)
{
	//printf("copy_right_img_func was called: %d\n", WEDGEBUG_BUF_SIZE);
	//image_copy(img, &img_right);
	//image_copy(img, &img_right);
	image_copy(img, &img_right);
	return img;
}



// New section ----------------------------------------------------------------------------------------------------------------
void wedgebug_init(){
  printf("Wedgebug init function was called\n");
  image_create(&img_left, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_YUV422);
  image_create(&img_right, WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_YUV422);
  image_create(&img_combined, WEDGEBUG_CAMERA_COMBINED_WIDTH, WEDGEBUG_CAMERA_COMBINED_HEIGHT, IMAGE_YUV422);

  //show_image_data(&img_left);
  cv_add_to_device(&WEDGEBUG_CAMERA_LEFT, copy_left_img_func, WEDGEBUG_CAMERA_LEFT_FPS);
  cv_add_to_device(&WEDGEBUG_CAMERA_RIGHT, copy_right_img_func, WEDGEBUG_CAMERA_RIGHT_FPS);
}

void wedgebug_periodic(){
  // your periodic code here.
  // freq = 4.0 Hz
	//printf("Wedgebug periodic function was called\n");



}



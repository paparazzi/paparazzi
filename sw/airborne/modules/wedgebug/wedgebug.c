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
#include <stdio.h>
#include "modules/wedgebug/wedgebug.h"
#include "modules/wedgebug/wedgebug_opencv.h"
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
void show_image_entry(struct image_t *img, int entry_position, const char *img_name);



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

void show_image_entry(struct image_t *img, int entry_position, const char *img_name)
{
	printf("Pixel %d value - %s: %d\n", entry_position, img_name ,((uint8_t*)img->buf)[entry_position]);
}


static struct image_t *copy_left_img_func(struct image_t *img)
{
	image_copy(img, &img_left);
	//show_image_data(img);
	//show_image_entry(&img_left, 10, "img_left");
	return img;
}


// Function x
static struct image_t *copy_right_img_func(struct image_t *img)
{
	image_copy(img, &img_right);
	//show_image_data(img);
	//show_image_entry(&img_right, 10, "img_right");
	return img;
}



// New section ----------------------------------------------------------------------------------------------------------------
void wedgebug_init(){
	//printf("Wedgebug init function was called\n");

	// Creating empty images
	image_create(&img_left, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_YUV422);
	image_create(&img_right, WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_YUV422);
	image_create(&img_combined, WEDGEBUG_CAMERA_COMBINED_WIDTH, WEDGEBUG_CAMERA_COMBINED_HEIGHT, IMAGE_YUV422);
	//show_image_data(&img_left);

	// Adding callback functions
	cv_add_to_device(&WEDGEBUG_CAMERA_LEFT, copy_left_img_func, WEDGEBUG_CAMERA_LEFT_FPS);
	cv_add_to_device(&WEDGEBUG_CAMERA_RIGHT, copy_right_img_func, WEDGEBUG_CAMERA_RIGHT_FPS);
}

void wedgebug_periodic(){
  // your periodic code here.
  // freq = 4.0 Hz
	//printf("Wedgebug periodic function was called\n");

	// Creating YlYr image from left and right YUV422 image


	for (uint32_t i = 0; i < (img_combined.buf_size - 1); (i+=2))
	{
		((uint8_t*)img_combined.buf)[i] = ((uint8_t*)img_left.buf)[i + 1];
		((uint8_t*)img_combined.buf)[i+1] = ((uint8_t*)img_right.buf)[i + 1];
	}

	//save_image_gray(img_combined.buf, 480, 240);
	//save_image_color(img_combined.buf, img_combined.w, img_combined.h);

	uint8_t buf_merged[240*240];

	uint8_t max;
	uint8_t min;
	uint32_t j = 0;
	for (uint32_t i = 1; i < ((240*240) - 1); (i+=1))
	{
		buf_merged[i] = ((uint8_t*)img_combined.buf)[j];
		j +=2;

		if (i ==1){max = buf_merged[i]; min = buf_merged[i];}
		else {
			if(buf_merged[i] > max){max = buf_merged[i];}
			if(buf_merged[i] < min){min = buf_merged[i];}
		}


		//printf("buf_merged[%d]: %d\n", i, buf_merged[i]);

	}
	printf("Before C++ function: Min=%d; Max=%d\n", min, max);

	save_image_gray(buf_merged, 240, 240);
	save_image_color(img_combined.buf, img_combined.w, img_combined.h);
	printf("\n");


	/*

	for (uint32_t i = 0; i < (img_combined.buf_size - 1); (i+=2))
	{
		((uint8_t*)img_combined.buf)[i] = ((uint8_t*)img_left.buf)[i + 1];
		((uint8_t*)img_combined.buf)[i+1] = ((uint8_t*)img_right.buf)[i + 1];
	}


	int gray_v_left = 0;
	int gray_v_right = gray_v_left + 1; //11
	int gray_f_images = gray_v_left + 1; //11


	printf("Compare left gray value (position %d) from left image (position %d)\n", gray_v_left, gray_f_images);
	show_image_entry(&img_combined, gray_v_left, "img_combined_from_left");
	show_image_entry(&img_left, gray_f_images, "img_left");
	printf("\n");
	printf("Compare left gray value (position %d) from right image (position %d)\n", gray_v_right, gray_f_images);
	show_image_entry(&img_combined, gray_v_right, "img_combined_from_right");
	show_image_entry(&img_right, gray_f_images, "img_right");
	printf("\n\n");
	*/

}



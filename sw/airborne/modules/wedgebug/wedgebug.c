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

struct image_t img_YY;
struct image_t img_YY_left;
struct image_t img_YY_right;

uint8_t buf_left[WEDGEBUG_CAMERA_LEFT_HEIGHT * WEDGEBUG_CAMERA_LEFT_WIDTH];
uint8_t buf_right[WEDGEBUG_CAMERA_RIGHT_HEIGHT * WEDGEBUG_CAMERA_RIGHT_WIDTH];
//static pthread_mutex_t mutex;



// New section ----------------------------------------------------------------------------------------------------------------
// Functions - declaration
static struct image_t *copy_left_img_func(struct image_t *img); // Function X: Copies left image into a buffer (buf_left)
static struct image_t *copy_right_img_func(struct image_t *img); // Function X: Copies left image into a buffer (buf_right)
const char* get_img_type(enum image_type img_type); // Function 1: Displays image type
void show_image_data(struct image_t *img); // Function 2: Displays image data
void show_image_entry(struct image_t *img, int entry_position, const char *img_name);
void split_YY_image(struct image_t *img_YY, struct image_t *img_gray_left_empty, struct image_t *img_gray_right_empty);



// New section ----------------------------------------------------------------------------------------------------------------
// Function - definition
// Function 1
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


// Function x
void show_image_data(struct image_t *img)
{
	printf("Image-Type: %s\n", get_img_type(img->type));
	printf("Image-Width: %d\n", img->w);
	printf("Image-Height: %d\n", img->h);
	printf("Image-Buf_Size: %d\n", img->buf_size);
	printf("Image-Buf_Memory_Occupied: %lu\n", sizeof(img->buf));
}


// Function x
void show_image_entry(struct image_t *img, int entry_position, const char *img_name)
{
	printf("Pixel %d value - %s: %d\n", entry_position, img_name ,((uint8_t*)img->buf)[entry_position]);
}

// Function x
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

// Function x
void split_YY_image(struct image_t *img_YY, struct image_t *img_gray_left_empty, struct image_t *img_gray_right_empty)
{
	if (img_gray_left_empty->w != (img_YY->w / 2) || img_gray_right_empty->w != (img_YY->w / 2)) {
		printf("New images cannot be used to split YY image!\n");
	    return;
	  }

	uint32_t j = 0;
	for (uint32_t i = 0; i <  img_YY->buf_size; i +=2)
	{
		((uint8_t*) img_gray_left_empty->buf)[j] = ((uint8_t*) img_YY->buf)[i];
		((uint8_t*) img_gray_right_empty->buf)[j+1] = ((uint8_t*) img_YY->buf)[i];


		j++;
	}
}



// New section ----------------------------------------------------------------------------------------------------------------
void wedgebug_init(){
	//printf("Wedgebug init function was called\n");

	// Creating empty images
	image_create(&img_left, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_YUV422);
	image_create(&img_right, WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_YUV422);
	image_create(&img_YY, WEDGEBUG_CAMERA_COMBINED_WIDTH, WEDGEBUG_CAMERA_COMBINED_HEIGHT, IMAGE_YUV422);

	// Empty images below are created for tests
	image_create(&img_YY_left, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_GRAYSCALE);
	image_create(&img_YY_right, WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_GRAYSCALE);


	// Adding callback functions
	cv_add_to_device(&WEDGEBUG_CAMERA_LEFT, copy_left_img_func, WEDGEBUG_CAMERA_LEFT_FPS);
	cv_add_to_device(&WEDGEBUG_CAMERA_RIGHT, copy_right_img_func, WEDGEBUG_CAMERA_RIGHT_FPS);
}

void wedgebug_periodic(){
  // your periodic code here.
  // freq = 4.0 Hz
	//printf("Wedgebug periodic function was called\n");

	// Creating YlYr image from left and right YUV422 image

	uint32_t j = 0;
	for (uint32_t i = 0; i < (img_YY.buf_size - 1); (i+=2))
	{
		((uint8_t*)img_YY.buf)[i] = ((uint8_t*)img_left.buf)[i + 1];
		((uint8_t*)img_YY.buf)[i+1] = ((uint8_t*)img_right.buf)[i + 1];

		//((uint8_t*)img_YY_left.buf)[j] = ((uint8_t*)img_left.buf)[i + 1];
		//((uint8_t*)img_YY_right.buf)[j] = ((uint8_t*)img_right.buf)[i + 1];
		j++;
	}


	image_to_grayscale(&img_left, &img_YY_left);
	image_to_grayscale(&img_right, &img_YY_right);

	save_image_gray(img_YY.buf, 480, 240, "/home/dureade/Documents/paparazzi_images/YY_stereo_image.bmp");
	save_image_color(img_right.buf, img_right.w, img_right.h, "/home/dureade/Documents/paparazzi_images/color_image.bmp");

	//split_YY_image(&img_YY, &img_YY_left, &img_YY_right);

	save_image_gray(img_YY_left.buf, img_YY_left.w, img_YY_left.h, "/home/dureade/Documents/paparazzi_images/YY_stereo_left_image.bmp");
	save_image_gray(img_YY_right.buf, img_YY_right.w, img_YY_right.h, "/home/dureade/Documents/paparazzi_images/YY_stereo_right_image.bmp");

	printf("img_left.buf_size: %d\n", img_left.buf_size);
	printf("img_YY_left.buf_size: %d", img_YY_left.buf_size);

	//show_image_entry(&img_YY_right, 0 , "img_YY_right");
	//show_image_entry(&img_right, 1 , "img_right");
	//show_image_entry(&img_YY_left, 0 , "img_YY_left");
	//show_image_entry(&img_left, 1 , "img_left");


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



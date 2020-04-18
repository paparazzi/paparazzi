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
struct image_t img_depth;
//static pthread_mutex_t mutex;



// New section ----------------------------------------------------------------------------------------------------------------
//Function - Declaration
//Supporting
const char* get_img_type(enum image_type img_type); // Function 1: Displays image type
void show_image_data(struct image_t *img); // Function 2: Displays image data
void show_image_entry(struct image_t *img, int entry_position, const char *img_name); // Function 3: Displays pixel value of image
//Core
static struct image_t *copy_left_img_func(struct image_t *img); // Function 1: Copies left image into a buffer (buf_left)
static struct image_t *copy_right_img_func(struct image_t *img); // Function 2: Copies left image into a buffer (buf_right)
void UYVYs_interlacing_V(struct image_t *YY, struct image_t *left, struct image_t *right); // Function 3: Copies gray pixel values of left and right UYVY images into merged YY image
void UYVYs_interlacing_H(struct image_t *merged, struct image_t *left, struct image_t *right);



// New section ----------------------------------------------------------------------------------------------------------------
// Function - Definition
// Supporting:
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


// Function 2
void show_image_data(struct image_t *img)
{
	printf("Image-Type: %s\n", get_img_type(img->type));
	printf("Image-Width: %d\n", img->w);
	printf("Image-Height: %d\n", img->h);
	printf("Image-Buf_Size: %d\n", img->buf_size);
	printf("Image-Buf_Memory_Occupied: %lu\n", sizeof(img->buf));
}


// Function 3
void show_image_entry(struct image_t *img, int entry_position, const char *img_name)
{
	printf("Pixel %d value - %s: %d\n", entry_position, img_name ,((uint8_t*)img->buf)[entry_position]);
}


// Core:
// Function 1
static struct image_t *copy_left_img_func(struct image_t *img)
{
	image_copy(img, &img_left);
	//show_image_data(img);
	//show_image_entry(&img_left, 10, "img_left");
	return img;
}


// Function 2
static struct image_t *copy_right_img_func(struct image_t *img)
{
	image_copy(img, &img_right);
	//show_image_data(img);
	//show_image_entry(&img_right, 10, "img_right");
	return img;
}


// Function 3
void UYVYs_interlacing_V(struct image_t *merged, struct image_t *left, struct image_t *right)
{
	// Error messages
	if (left->w != right->w || left->h != right->h)
	{
		printf("The dimensions of the left and right image to not match!");
		return;
	}
	if ((merged->w * merged->h) != (2 * right->w) * right->w)
	{
		printf("The dimensions of the empty image template for merger are not sufficient to merge gray left and right pixel values.");
		return;
	}

	uint8_t *UYVY_left = left->buf;
	uint8_t *UYVY_right = right->buf;
	uint8_t *YY = merged->buf;
	uint32_t loop_length = left->w * right->h;

	// Incrementing pointers to get to first gray value of the UYVY images
	UYVY_left++;
	UYVY_right++;


	for (uint32_t i = 0; i < loop_length; i++)
	{
		*YY = *UYVY_left; // Copies left gray pixel (Y) to the merged image YY, in first position
		YY++; // Moving to second position of merged image YY
		*YY = *UYVY_right; // Copies right gray pixel (Y) to the merged image YY, in second position
		YY++; // Moving to the next position, in preparation to copy left gray pixel (Y) to the merged image YY
		UYVY_left+=2; // Moving pointer to next gray pixel (Y), in the left image
		UYVY_right+=2; // Moving pointer to next gray pixel (Y), in the right image
		/*
		 * Note: Since the loop lenth is based on the UYVY image the size of the data should be (2 x w) x h.
		 * This is also the same size as for the new merged YY image.
		 * Thus incrementing the pointer for UYVY_left and right, in each iteration, does not lead to problems (same for YY image)
		 */
	}
}



// Function 3
void UYVYs_interlacing_H(struct image_t *merged, struct image_t *left, struct image_t *right)
{
	// Error messages
	if (left->w != right->w || left->h != right->h)
	{
		printf("The dimensions of the left and right image to not match!");
		return;
	}
	if ((merged->w * merged->h) != (2 * right->w) * right->w)
	{
		printf("The dimensions of the empty image template for merger are not sufficient to merge gray left and right pixel values.");
		return;
	}

	uint8_t *UYVY_left = left->buf;
	uint8_t *UYVY_right = right->buf;
	uint8_t *YY1 = merged->buf; // points to first row for pixels of left image
	uint8_t *YY2 = YY1 + merged->w; // points to second row for pixels of right image

	// Incrementing pointers to get to first gray value of the UYVY images
	UYVY_left++;
	UYVY_right++;

	for (uint32_t i = 0; i < left->h; i++)
	{
		//printf("Loop 1: %d\n", i);
		for (uint32_t j = 0; j < left->w; j++)
		{
			//printf("Loop 1: %d\n", j);
			*YY1 = *UYVY_left;
			*YY2 = *UYVY_right;
			YY1++;
			YY2++;
			UYVY_left+=2;
			UYVY_right+=2;
		}
		YY1 += merged->w; // Jumping pointer to second next row (i.e. over row with pixels from right image)
		YY2 += merged->w; // Jumping pointer to second next row (i.e. over row with pixels from left image)
	}
}




// New section ----------------------------------------------------------------------------------------------------------------
void wedgebug_init(){
	//printf("Wedgebug init function was called\n");

	// Creating empty images
	image_create(&img_left, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_YUV422); // To store left camera image
	image_create(&img_right,WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_YUV422);// To store right camera image
	image_create(&img_YY,WEDGEBUG_CAMERA_INTERLACED_WIDTH, WEDGEBUG_CAMERA_INTERLACED_HEIGHT, IMAGE_GRAYSCALE);// To store interlaced image
	image_create(&img_depth,240, 240, IMAGE_GRAYSCALE);// To store depth

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

	//UYVYs_interlacing_V(&img_YY, &img_left, &img_right); // Creating YlYr image from left and right YUV422 image
	UYVYs_interlacing_H(&img_YY, &img_left, &img_right);
	image_to_grayscale(&img_left, &img_YY_left); // Converting left image from UYVY to gray scale for saving function
	image_to_grayscale(&img_right, &img_YY_right); // Converting right image from UYVY to gray scale for saving function

	save_image_gray(img_YY.buf, img_YY.w, img_YY.h, "/home/dureade/Documents/paparazzi_images/YY_stereo_image.bmp");
	save_image_gray(img_YY_left.buf, img_YY_left.w, img_YY_left.h, "/home/dureade/Documents/paparazzi_images/YY_stereo_left_image.bmp");
	save_image_gray(img_YY_right.buf, img_YY_right.w, img_YY_right.h, "/home/dureade/Documents/paparazzi_images/YY_stereo_right_image.bmp");

	SBM(&img_YY_left, &img_YY_right ,&img_depth , 16, 25);

	save_image_gray(img_depth.buf, img_depth.w, img_depth.h, "/home/dureade/Documents/paparazzi_images/image_disparity.bmp");

	//printf("img_YY.buf_size: %d\n", img_YY.buf_size);
	//printf("img_YY.buf_size: %d\n", img_YY.h);
	//printf("img_YY.buf_size: %d\n", img_YY.w);
	//printf("img_YY.buf_size: %d\n", img_left.buf_size);
	//printf("img_YY_left.buf_size: %d", img_YY_left.buf_size);
	//show_image_entry(&img_YY_right, 0 , "img_YY_right");
	//show_image_entry(&img_right, 1 , "img_right");
	//show_image_entry(&img_YY_left, 0 , "img_YY_left");
	//show_image_entry(&img_left, 1 , "img_left");
	//printf("\n");


}



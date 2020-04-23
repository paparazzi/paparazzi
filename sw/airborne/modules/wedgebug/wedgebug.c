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
struct image_t img_left_int8;
struct image_t img_right_int8;
struct image_t img_depth_int8;
struct image_t img_depth_int16;
struct image_t img_depth_int8_cropped;
struct image_t img_depth_int16_cropped;
//static pthread_mutex_t mutex;
int N_disparities = 64;
int block_size_disparities = 25;
int min_disparity = 0;
uint16_t crop_y;
uint16_t crop_height;
uint16_t crop_x;
uint16_t crop_width;


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
void post_disparity_crop_rect(uint16_t* height_start, uint16_t* height_offset, uint16_t* width_start, uint16_t* width_offset, const uint16_t height_old,const uint16_t width_old, const int disp_n, const int block_size);



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

// Function 4


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




// Function 4 - Return the upper left coordinates of a square (x and y coordinates) and the offset in terms of width and height,
// given the number of disparity levels and the block size used by the block matching algorithm. This is need to crop an image
void post_disparity_crop_rect(
		uint16_t* height_start,
		uint16_t* height_offset,
		uint16_t* width_start,
		uint16_t* width_offset,
		const uint16_t height_old,
		const uint16_t width_old,
		const int disp_n,
		const int block_size)
{

	uint16_t block_size_black = block_size / 2;
	uint16_t left_black = disp_n + block_size_black;


	*height_start = block_size_black;
	*height_offset = height_old - block_size_black;
	*height_offset = *height_offset - *height_start;

	*width_start = left_black - 1;
	*width_offset = width_old - block_size_black;
	*width_offset = *width_offset - *width_start;
}




// New section ----------------------------------------------------------------------------------------------------------------
void wedgebug_init(){
	//printf("Wedgebug init function was called\n");

	// Creating empty images
	image_create(&img_left, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_YUV422); // To store left camera image
	image_create(&img_right,WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_YUV422);// To store right camera image
	image_create(&img_YY,WEDGEBUG_CAMERA_INTERLACED_WIDTH, WEDGEBUG_CAMERA_INTERLACED_HEIGHT, IMAGE_GRAYSCALE);// To store interlaced image

	image_create(&img_depth_int8,WEDGEBUG_CAMERA_DISPARITY_WIDTH, WEDGEBUG_CAMERA_DISPARITY_HEIGHT, IMAGE_GRAYSCALE);// To store depth
	image_create(&img_depth_int16,WEDGEBUG_CAMERA_DISPARITY_WIDTH, WEDGEBUG_CAMERA_DISPARITY_HEIGHT, IMAGE_OPENCV_DISP);// To store depth

	post_disparity_crop_rect(&crop_y, &crop_height, &crop_x, &crop_width, img_right.h , img_right.w, N_disparities, block_size_disparities);
	image_create(&img_depth_int8_cropped,crop_width, crop_height, IMAGE_GRAYSCALE);// To store cropped depth
	image_create(&img_depth_int16_cropped, crop_width, crop_height, IMAGE_OPENCV_DISP);


	// Empty images below are created for tests
	image_create(&img_left_int8, WEDGEBUG_CAMERA_LEFT_WIDTH, WEDGEBUG_CAMERA_LEFT_HEIGHT, IMAGE_GRAYSCALE);
	image_create(&img_right_int8, WEDGEBUG_CAMERA_RIGHT_WIDTH, WEDGEBUG_CAMERA_RIGHT_HEIGHT, IMAGE_GRAYSCALE);


	// Adding callback functions
	cv_add_to_device(&WEDGEBUG_CAMERA_LEFT, copy_left_img_func, WEDGEBUG_CAMERA_LEFT_FPS);
	cv_add_to_device(&WEDGEBUG_CAMERA_RIGHT, copy_right_img_func, WEDGEBUG_CAMERA_RIGHT_FPS);
}

void wedgebug_periodic(){
  // your periodic code here.
  // freq = 4.0 Hz
	//printf("Wedgebug periodic function was called\n");

	//UYVYs_interlacing_V(&img_YY, &img_left, &img_right); // Creating YlYr image from left and right YUV422 image
	//UYVYs_interlacing_H(&img_YY, &img_left, &img_right);
	image_to_grayscale(&img_left, &img_left_int8); // Converting left image from UYVY to gray scale for saving function
	image_to_grayscale(&img_right, &img_right_int8); // Converting right image from UYVY to gray scale for saving function


	SBM(&img_depth_int8, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 0);// Creating cropped disparity map image
	SBM(&img_depth_int16, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 0);// Creating cropped disparity map image

	SBM(&img_depth_int8_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 1);// Creating cropped disparity map image
	SBM(&img_depth_int16_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 1);// Creating cropped disparity map image




	//save_image_gray(&img_YY, "/home/dureade/Documents/paparazzi_images/img_YY.bmp");
	save_image_gray(&img_left_int8, "/home/dureade/Documents/paparazzi_images/img_left_int8.bmp");
	save_image_gray(&img_right_int8, "/home/dureade/Documents/paparazzi_images/img_right_int8.bmp");

	save_image_gray(&img_depth_int8, "/home/dureade/Documents/paparazzi_images/img_depth_int8.bmp");
	save_image_gray(&img_depth_int16, "/home/dureade/Documents/paparazzi_images/img_depth_int16.bmp");

	save_image_gray(&img_depth_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int8_cropped.bmp");
	save_image_gray(&img_depth_int16_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int16_cropped.bmp");




	/*
	int i = 20000;

	printf("index 0 value of img_depth_int16: %f\n", ((int16_t*)img_depth_int16.buf)[0]);
	printf("index %d value of img_depth_int16: %d\n",i ,((int16_t*)img_depth_int16.buf)[i]);
	printf("index %d value of img_depth_int16: %f\n",i ,((int16_t*)img_depth_int16.buf)[i]);


	for (int i = 0; i < (img_depth2.w * img_depth2.h); i++)
	{
		if (i % 20000 == 0)
		{
			printf("New image (after C++) position %d: %d\n",i, ((uint8_t*)img_depth2.buf)[i]);

		}
	}*/


	//printf("Buf size  (bytes) of img_depth_uint16 (should be 66096): %d\n", img_depth_uint16.buf_size);
	//post_disparity_dims(&new_height_min, &new_height_max, &new_width_min, &new_width_max, 240 , 240, N_disparities, block_size_disparities);
	//post_disparity_dims2(&new_height, &new_width, 240 , 240, N_disparities, block_size_disparities);
	//post_disparity_crop_rect(&new_height_min, &new_height_max, &new_width_min, &new_width_max, 240 , 240, N_disparities, block_size_disparities);
	//printf("img_depth2 h:w = %d:%d\n", img_depth2.h, img_depth2.w);
	//printf("\n\n\n\n\n");
	//printf("height min:max = %d:%d\n width min:max = %d:%d\n", new_height_min, new_height_max, new_width_min, new_width_max);
	//printf("cropped image dims: [%d, %d]\n", new_height_max, new_width_max);
	//printf("cropped image dims2: [%d, %d]", new_height, new_width);
	//printf("height min:offset = %d:%d\n width min:offset = %d:%d\n", new_height_min, new_height_max, new_width_min, new_width_max);
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



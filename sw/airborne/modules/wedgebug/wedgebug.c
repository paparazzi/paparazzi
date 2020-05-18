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
#include "modules/computer_vision/lib/vision/image.h"// For image-related structures
#include "pthread.h"
#include <stdint.h> // Needed for common types like uint8_t
#include "state.h"
#include "math/pprz_algebra_float.h"// Needed for vector operations, Euclidean distance, FloatVect3 and FloatRMat
#include "math/pprz_algebra.h"// Needed for vector operations (simple subtraction)
#include "math/pprz_geodetic_float.h"// Needed for NedCoor_f
#include "generated/flight_plan.h" // Needed for WP (waypoint) functions and WP IDs (as defined in "ralphthesis2020_stereo_cyberzoo.xml")
#include "firmwares/rotorcraft/autopilot_guided.h" // Needed for guidance functions such as "autopilot_guided_goto_ned" and "guidance_h_set_guided_heading"


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
struct image_t img_middle_int8_cropped;
struct image_t img_edges_int8_cropped;

struct crop_t img_cropped_info;
struct img_size_t img_dims;
struct img_size_t img_cropped_dims;

struct img_size_t kernel_median_dims;
struct kernel_C1 median_kernel;


//static pthread_mutex_t mutex;
int N_disparities = 64;
int block_size_disparities = 25;
int min_disparity = 0;
uint8_t cycle_counter = 0;
struct FloatVect3 VGOALwenu;
struct FloatVect3 VGOALwned;
uint8_t is_goal_setpoint_set;

uint8_t median_disparity_in_front; // Variable to hold the median disparity in front of the drone. Needed to see if obstacle is there.

// Thresholds
uint8_t threshold_median_disparity; // Above this median disparity, an obstacle is considered to block the way (i.e. the blocking obstacle need to be close)
uint8_t threshold_disparity_of_edges; // Above this disparity edges are legible for WedgeBug algorithm (i.e. edges cannot be very far away)

// Principal points
struct point_t c_img;
struct point_t c_img_cropped;


// Declaring rotation matrices and transition vectors for frame to frame transformations
// 1) Rotation matrix and transition vector to transform from world ENU frame to world NED frame
struct FloatRMat Rwnedwenu;
struct FloatVect3 VNEDwenu;
// 2) Rotation matrix and transition vector to transform from world ned frame to robot frame
struct FloatRMat Rrwned;
struct FloatVect3 VRwned;
// 3) Rotation matrix and transition vector to transform from robot frame to camera frame
struct FloatRMat Rcr;
struct FloatVect3 VCr;

// Declaration and initialization of camera parameters
float b = WEDGEBUG_CAMERA_BASELINE / 1000.00;
uint16_t f = WEDGEBUG_CAMERA_FOCAL_LENGTH;



// Define new structures + enums

enum navigation_state {
  POSITION_INITIAL = 1,
  MOVE_TO_START = 2,
  POSITION_START = 3,
  MOVE_TO_GOAL = 4,
  POSITION_GOAL = 5,
  WEDGEBUG_START = 6,
  MOVE_TO_EDGE = 7,
  SCAN_EDGE = 8,

};

enum navigation_state current_state ;// Default state is 0 i.e. nothing


// New section: Functions - Declaration ----------------------------------------------------------------------------------------------------------------

// Supporting
const char* get_img_type(enum image_type img_type); // Function 1: Displays image type
void show_image_data(struct image_t *img); // Function 2: Displays image data
void show_image_entry(struct image_t *img, int entry_position, const char *img_name); // Function 3: Displays pixel value of image

// External
void post_disparity_crop_rect(struct crop_t *img_cropped_info,struct img_size_t *original_img_dims, const int disp_n, const int block_size);
void set_state(uint8_t state);
void kernel_create(struct kernel_C1 *kernel, uint16_t width, uint16_t height);
void kernel_free(struct kernel_C1 *kernel);
uint8_t getMedian(uint8_t *a, uint32_t n);

//Core
static struct image_t *copy_left_img_func(struct image_t *img); // Function 1: Copies left image into a buffer (buf_left)
static struct image_t *copy_right_img_func(struct image_t *img); // Function 2: Copies left image into a buffer (buf_right)
void UYVYs_interlacing_V(struct image_t *YY, struct image_t *left, struct image_t *right); // Function 3: Copies gray pixel values of left and right UYVY images into merged YY image
void UYVYs_interlacing_H(struct image_t *merged, struct image_t *left, struct image_t *right);

uint32_t maximum_intensity(struct image_t *img);
void thresholding_img(struct image_t *img, uint8_t threshold);
void principal_points(struct point_t *c_output ,const struct point_t *c_old_input, struct crop_t *img_cropped_info);
float disp_to_depth(const uint8_t d, const float b, const uint16_t f);
void Vi_to_Vc(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x , const uint8_t d, const float b, const uint16_t f);
int32_t indx1d_a(const int32_t y, const int32_t x, const struct image_t *img);
int32_t indx1d_b(const int32_t y, const int32_t x, const struct img_size_t *img_dims);
int32_t indx1d_c(const int32_t y, const int32_t x, const uint16_t img_height, const uint16_t img_width);

void Va_to_Vb(struct FloatVect3 *Vb, struct FloatVect3 *Va, struct FloatRMat *Rba, struct FloatVect3 *VOa);
void Vb_to_Va(struct FloatVect3 *Va, struct FloatVect3 *Vb, struct FloatRMat *Rba, struct FloatVect3 *VOa);
void Vw_to_Vc(struct FloatVect3 *Vc, struct FloatVect3 *Vw, struct FloatRMat *Rrw, struct FloatVect3 *VRw, struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose);
void Vc_to_Vw(struct FloatVect3 *Vw, struct FloatVect3 *Vc, struct FloatRMat *Rrw, struct FloatVect3 *VRw, struct FloatRMat *Rcr, struct FloatVect3 *VCr ,const uint8_t verbose);

float float_vect3_norm_two_points(struct FloatVect3 *V1, struct FloatVect3 *V2);
float heading_towards_waypoint(uint8_t wp);
uint8_t median_disparity_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median);






// New section: Functions - Definition ----------------------------------------------------------------------------------------------------------------

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



// External:

// Function 1 - Returns the upper left coordinates of a square (x and y coordinates) and the offset in terms of width and height,
// given the number of disparity levels and the block size used by the block matching algorithm. This is need to crop an image
void post_disparity_crop_rect(struct crop_t *img_cropped_info,struct img_size_t *original_img_dims,const int disp_n,const int block_size)
{

	uint16_t block_size_black = block_size / 2;
	uint16_t left_black = disp_n + block_size_black;


	img_cropped_info->y = block_size_black;
	img_cropped_info->h = original_img_dims->h - block_size_black;
	img_cropped_info->h = img_cropped_info->h - img_cropped_info->y;

	img_cropped_info->x = left_black - 1;
	img_cropped_info->w = original_img_dims->w - block_size_black;
	img_cropped_info->w  = img_cropped_info->w - img_cropped_info->x;
}


// Function 2 - Sets finite state machine state (useful for the flight path blocks)
extern void set_state(uint8_t state)
{
	current_state = state;
}


// Function 3 - Creates empty 8bit kernel
void kernel_create(struct kernel_C1 *kernel, uint16_t width, uint16_t height)
{
	kernel->h = height;
	kernel->w = width;
	kernel->buf_size = height * width;
	kernel->buf_weights = malloc(kernel->buf_size);
	kernel->buf_values = malloc(kernel->buf_size);
}


// Function 4 - Deletes 8bit kernel
void kernel_free(struct kernel_C1 *kernel)
{
  if (kernel->buf_weights != NULL) {
    free(kernel->buf_weights);
    kernel->buf_weights = NULL;
  }
  if (kernel->buf_values != NULL) {
    free(kernel->buf_values);
    kernel->buf_values = NULL;
  }
}


// Function 5 - Calculates median of a 8bit image
uint8_t getMedian(uint8_t *a, uint32_t n)
{
  // Allocate an array of the same size and sort it.
  uint32_t i, j;

  uint8_t dpSorted[n];
  for (i = 0; i < n; ++i) {
    dpSorted[i] = a[i];
  }
  for (i = n - 1; i > 0; --i) {
    for (j = 0; j < i; ++j) {
      if (dpSorted[j] > dpSorted[j + 1]) {
        uint8_t dTemp = dpSorted[j];
        dpSorted[j] = dpSorted[j + 1];
        dpSorted[j + 1] = dTemp;
      }
    }
  }

  // Middle or average of middle values in the sorted array.
  uint8_t dMedian = 0;
  if ((n % 2) == 0) {
    dMedian = (dpSorted[n / 2] + dpSorted[(n / 2) - 1]) / 2.0;
  } else {
    dMedian = dpSorted[n / 2];
  }
  return dMedian;
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


// Function 4
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


// Function 5 - Returns the maximum value in a uint8_t image
uint32_t maximum_intensity(struct image_t *img)
{
	uint32_t max = 0;
	for (uint32_t i = 0; i < img->buf_size; i++)
	{
		uint8_t* intensity = &((uint8_t*)img->buf)[i];

		if (*intensity > max)
		{
			max = *intensity;
		}
	}
	return max;
}


// Function 6 - Thresholds 8bit images given and turns all values >= threshold to 255
void thresholding_img(struct image_t *img, uint8_t threshold)
{
	for (uint32_t i = 0; i < img->buf_size; i++)
	{
		uint8_t* intensity = &((uint8_t*)img->buf)[i];

		if (*intensity >= threshold)
		{
			*intensity = 127;
		}
		else
			*intensity = 0;
	}
}


// Function 7 - Calculates principal point coordinates for a cropped image, based on the x
// and y coordinates of the cropped area (upper left-hand side: crop_y and crop_x).
void principal_points(struct point_t *c_output ,const struct point_t *c_old_input, struct crop_t *img_cropped_info)
{
	c_output->y = c_old_input->y - img_cropped_info->y;
	c_output->x = c_old_input->x - img_cropped_info->x;
}


// Function 8 - Converts disparity to depth using focal length (in pixels) and baseline distance (in meters)
float disp_to_depth(const uint8_t d, const float b, const uint16_t f)
{
	return b * f / d;
}


// Function 9 - Calculates 3d points in a scene based on the 2d coordinates of the point in the
// image plane and the depth. d in in pixels, b is in meters and f is in pixels
void Vi_to_Vc(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x , const uint8_t d, const float b, const uint16_t f)
{
	// Calculating Z
	// In case disparity is 0 Z will be very very small to avoid detection of algorithm that
	// calculates closest edge point to target goal
	//printf("y=%d\n", image_point_y);
	//printf("x=%d\n", image_point_x);
	//printf("d=%d\n", d);


	if (d==0)
	{
		scene_point->z = 0.0001;
	}
	else
	{
		scene_point->z = disp_to_depth(d, b, f);
	}


	//printf("Z=%f\n", scene_point->Z);

	// Calculating Y
	scene_point->y = image_point_y * scene_point -> z / f;

	// Calculating X
	scene_point->x = image_point_x * scene_point -> z / f;
	//printf("Y (y=%d) =  %f\n", image_point->y, scene_point->Y);
	//printf("X (x=%d) =  %f\n", image_point->x, scene_point->X);
	//printf("Z (d=%d) =  %f\n", d, scene_point->Z);

}


// Function 10a - Converts 2d coordinates into 1d coordinates (for 1d arrays) - using struct image_t for dimensions
int32_t indx1d_a(const int32_t y, const int32_t x, const struct image_t *img)
{
	if (x >= (img->w) || x < 0)
	{
		printf("Error: index x=%d is out of bounds for axis 0 with size %d. Returning -1\n", x, img->w);
		return -1;
	}
	else if (y >= (img->h) || y < 0)
	{
		printf("Error: index y=%d is out of bounds for axis 0 with size %d. Returning -1\n", y, img->h);
		return -1;
	}
	else
	{
		return x + img->w * y;
	}
}


// Function 10b - Converts 2d coordinates into 1d coordinates (for 1d arrays) - using struct img_size_t for dimensions
int32_t indx1d_b(const int32_t y, const int32_t x, const struct img_size_t *img_dims)
{
	if (x >= (img_dims->w) || x < 0)
	{
		printf("Error: index %d is out of bounds for axis 0 with size %d. Returning -1\n", x, img_dims->w);
		return -1;
	}
	else if (y >= (img_dims->h) || y < 0)
	{
		printf("Error: index %d is out of bounds for axis 0 with size %d. Returning -1\n", y, img_dims->h);
		return -1;
	}
	else
	{
		return x + img_dims->w * y;
	}
}

// Function 10c - Converts 2d coordinates into 1d coordinates (for 1d arrays) - using two uint16_t values for dimensions
int32_t indx1d_c(const int32_t y, const int32_t x, const uint16_t img_height, const uint16_t img_width)
{
	if (x >= (img_width) || x < 0)
	{
		printf("Error: index x=%d is out of bounds for axis 0 with size %d. Returning -1\n", x, img_width);
		return -1;
	}
	else if (y >= (img_height) || y < 0)
	{
		printf("Error: index y=%d is out of bounds for axis 0 with size %d. Returning -1\n", y, img_height);
		return -1;
	}
	else
	{
		return x + img_width * y;
	}
}


// Function 11 - Function to convert point in coordinate system a to a point in the coordinate system b
void Va_to_Vb(struct FloatVect3 *Vb, struct FloatVect3 *Va, struct FloatRMat *Rba, struct FloatVect3 *VOa)
{
	// The following translates world vector coordinates into the agent coordinate system
	Va->x = Va->x - VOa->x;
	Va->y = Va->y - VOa->y;
	Va->z = Va->z - VOa->z;

	// In case the axes of the world coordinate system (w) and the agent coordinate system (a) do not
	// coincide, they are adjusted with the rotation matrix R
	float_rmat_vmult(Vb, Rba, Va);
}


// Function 12 - Function to convert point in coordinate system b back to a point in the coordinate system a
void Vb_to_Va(struct FloatVect3 *Va, struct FloatVect3 *Vb, struct FloatRMat *Rba, struct FloatVect3 *VOa)
{
	// In case the axes of the agent coordinate system (a) and the world coordinate system (w) do not
	// coincide, they are adjusted with the inverse rotation matrix R
	float_rmat_transp_vmult(Va, Rba, Vb);

	// The following translates agent vector coordinates into the world coordinate system
	Va->x = Va->x + VOa->x;
	Va->y = Va->y + VOa->y;
	Va->z = Va->z + VOa->z;
}


// Function 13 - Function wrapper to convert a point in the world coordinate system to a point in the camera coordinate system
void Vw_to_Vc(struct FloatVect3 *Vc, struct FloatVect3 *Vw, struct FloatRMat *Rrw, struct FloatVect3 *VRw, struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose)
{
	struct FloatVect3 Vr;

	// Print log only if enabled
	if (verbose != 0)
	{
		// Rotation matrix - World coordinate system expressed in the robot coordinate system
		printf("Rrw\n");
		printf("%f, %f, %f,\n", Rrw->m[0],Rrw->m[1], Rrw->m[2]);
		printf("%f, %f, %f,\n", Rrw->m[3],Rrw->m[4], Rrw->m[5]);
		printf("%f, %f, %f\n\n", Rrw->m[6],Rrw->m[7], Rrw->m[8]);

		// Vector coordinates - Robot in the world frame system
		printf("VRw (drone location)\n");
		printf(" %f\n %f\n %f\n\n", VRw->x, VRw->y, VRw->z);

		// World coordinates
		printf("Vw\n");
		printf(" %f\n %f\n %f\n\n", Vw->x, Vw->y, Vw->z);
	}


	// Robot coordinates from world coordinates
	Va_to_Vb(&Vr, Vw, Rrw, VRw);

	// Print log only if enabled
	if (verbose != 0)
	{
		// Robot coordinates
		printf("Vr\n");
		printf(" %f\n %f\n %f\n\n", Vr.x, Vr.y, Vr.z);

		// Rotation matrix - Robot coordinate system expressed in the camera coordinate system
		printf("Rcr\n");
		printf("%f, %f, %f,\n", Rcr->m[0],Rcr->m[1], Rcr->m[2]);
		printf("%f, %f, %f,\n", Rcr->m[3],Rcr->m[4], Rcr->m[5]);
		printf("%f, %f, %f\n\n", Rcr->m[6],Rcr->m[7], Rcr->m[8]);

		// Vector coordinates - Camera in the robot frame system
		printf("VCa (camera location)\n");
		printf(" %f\n %f\n %f\n\n", VCr->x, VCr->y, VCr->z);
	}

	// Camera coordinates from robot coordinates
	Va_to_Vb(Vc, &Vr, Rcr, VCr);

	// Print log only if enabled
	if (verbose != 0)
	{
		// Camera coordinates
		printf("Vc\n");
		printf(" %f\n %f\n %f\n\n", Vc->x, Vc->y, Vc->z);
	}
}


// Function 14 - Function wrapper to convert a point in the camera coordinate system back to a point in the world coordinate system
void Vc_to_Vw(struct FloatVect3 *Vw, struct FloatVect3 *Vc, struct FloatRMat *Rrw, struct FloatVect3 *VRw, struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose)
{
	struct FloatVect3 Vr;

	// Agent coordinates from camera coordinates
	Vb_to_Va(&Vr, Vc, Rcr, VCr);

	// Print log only if enabled
	if (verbose != 0)
	{
		// Back to robot coordinates
		printf("Vr - back calculated\n");\
		printf(" %f\n %f\n %f\n\n", Vr.x, Vr.y, Vr.z);
	}


	// World coordinates from a coordinates
	Vb_to_Va(Vw, &Vr, Rrw, VRw);

	// Print log only if enabled
	if (verbose != 0)
	{
		// Back to world coordinates
		printf("Vw - back calculated\n");
		printf(" %f\n %f\n %f\n\n", Vw->x, Vw->y, Vw->z);
	}
}

// Function 15 - Function to obtain the Euclidean distance (in meters) between two 3d points
float float_vect3_norm_two_points(struct FloatVect3 *V1, struct FloatVect3 *V2)
{
	struct FloatVect3 V_V1V2_diff;
	VECT3_DIFF(V_V1V2_diff, *V1, *V2);
	return float_vect3_norm(&V_V1V2_diff);
}

// Function 16 - function to calculate angle between robot and specific waypoint
float heading_towards_waypoint(uint8_t wp)
{
  struct FloatVect2 VWPwt = {WaypointX(wp), WaypointY(wp)};
  struct FloatVect2 VRwt_VWPwtVWRwt_diff;
  float angle;

  VECT2_DIFF(VRwt_VWPwtVWRwt_diff, VWPwt, *stateGetPositionEnu_f());
  angle = atan2f(VRwt_VWPwtVWRwt_diff.x, VRwt_VWPwtVWRwt_diff.y);
  return angle;
}

// Function 17 - Function to calculate median disparity to a point (Vi) in an image (img), using a kernel structure (kernel_median)
uint8_t median_disparity_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median)
{
	// Creating Start and stop coordinates of in the image coordinate system, based on kernel size
	uint8_t VSTARTi_y = Vi->y  - (kernel_median->h / 2);
	uint8_t VSTARTi_x = Vi->x - (kernel_median->w/ 2);
	uint8_t VSTOPi_y = Vi->y + (kernel_median->h / 2);
	uint8_t VSTOPi_x = Vi->x + (kernel_median->w / 2);

	// Declaring kernel coordinates
	uint16_t Vk_y;
	uint16_t Vk_x;

	// Declaring 1d indices (for result of transforming 2d coordinates into 1d coordinate)
	int32_t index_img;
	int32_t index_kernel;

	// Declaring variable to store median in
	uint8_t median;


	// Here we get the median value of a block in the middle of an image using a kernel structure
	for (uint8_t Vi_y = VSTARTi_y; Vi_y < (VSTOPi_y+1); Vi_y++)
	{
		for (uint8_t Vi_x = VSTARTi_x; Vi_x < (VSTOPi_x+1); Vi_x++)
		{
			// Calculating kernel coordinates
			Vk_y = Vi_y - VSTARTi_y;
			Vk_x = Vi_x - VSTARTi_x;

			// Converting 2d indices to 1d indices
			index_img = indx1d_a(Vi_y , Vi_x, img);
			index_kernel = indx1d_c(Vk_y, Vk_x, median_kernel.h, median_kernel.w);

			// Saving disparity values of image underneath the kernel, into the kernel buffer
			((uint8_t*) median_kernel.buf_values)[index_kernel] = ((uint8_t*) img->buf)[index_img];
		}
	}

	// Calculating median disparity value of values recoded by the kernel
	median = getMedian(((uint8_t*) median_kernel.buf_values), (median_kernel.h * median_kernel.w)); //

	return median;
}


/*
 * a: Coordinate system a (i.e. the coordinate frame a)
 * b: Coordinate system b (i.e. the coordinate frame b)
 * w: World coordinate system (i.e. the world coordinate frame = x is depth, y is left and right, and z is altitude))
 * r: Robot coordinate system (i.e. the robot coordinate frame = x is depth, y is left and right, and z is altitude))
 * c: Camera Coordinate system (i.e. the camera coordinate frame = x is left and right, y is altitude and z is depth)
 * i: Image coordinate system  (i.e. the image (plane) coordinate frame)
 * k: Kernel coordinate system (i.e. the kernel coordinate frame)
 *
 * Va: Vector coordinates, in the coordinate system a (i.e. a point in the coordinate system a)
 * Vb: Vector coordinates, in the coordinate system b (i.e. a point in the coordinate system b)
 * Vwned: Vector coordinates, in the world coordinate system (i.e. a point in the world coordinate system) -NED
 * Vwenu: Vector coordinates, in the world coordinate system two (i.e. a point in the world coordinate system two) - ENU
 * Vr: Vector coordinates, in the robot coordinate system (i.e. a point in the world coordinate system)
 * Vc: Vector coordinates, in the camera coordinate system (i.e. a point in the world coordinate system)
 * Vi: Vector coordinates, in the image coordinate system (i.e. a point in the image [plane] coordinates system)
 * Vk: Vector coordinates, in the kernel coordinate system (i.e. a point in the kernel coordinates system)
 *
 * R: Rotation matrix
 *
 * Rba: Rotation matrix of coordinate system a expressed in the coordinate system b
 * Rrw: Rotation matrix of world coordinate system expressed in the robot coordinate system
 * Rcr: Rotation matrix of robot coordinate system expressed in the camera coordinate system
 *
 * VRw: Vector coordinates of robot in the world coordinate system
 * VCr: Vector coordinates of camera in the robot coordinate system
 * VOa: Vector coordinates of the object (has its own object frame) in the the coordinate system a *
 */




// New section: Init and periodic functions ----------------------------------------------------------------------------------------------------------------
void wedgebug_init(){
	//printf("Wedgebug init function was called\n");


	// Images creation process:
	// Creation of images
	// Creating structure to hold dimensions of normal camera images
	img_dims.w = WEDGEBUG_CAMERA_LEFT_WIDTH;img_dims.h = WEDGEBUG_CAMERA_LEFT_HEIGHT;
	image_create(&img_left, img_dims.w , img_dims.h, IMAGE_YUV422); // To store left camera image
	image_create(&img_right,img_dims.w , img_dims.h, IMAGE_YUV422);// To store right camera image
	//image_create(&img_YY,WEDGEBUG_CAMERA_INTERLACED_WIDTH, WEDGEBUG_CAMERA_INTERLACED_HEIGHT, IMAGE_GRAYSCALE);// To store interlaced image
	image_create(&img_left_int8, img_dims.w , img_dims.h, IMAGE_GRAYSCALE); // To store gray scale version of left image
	image_create(&img_right_int8, img_dims.w , img_dims.h, IMAGE_GRAYSCALE); // To store gray scale version of left image


	// Creation of images - Cropped:
	// Calculating cropped image details (x, y, width and height)
	post_disparity_crop_rect(&img_cropped_info, &img_dims, N_disparities, block_size_disparities);
	// Creating structure to hold dimensions of cropped camera images
	img_cropped_dims.w = img_cropped_info.w; img_cropped_dims.h = img_cropped_info.h;
	// Creating empty images - cropped
	image_create(&img_depth_int8_cropped, img_cropped_dims.w, img_cropped_dims.h, IMAGE_GRAYSCALE);// To store cropped depth - 8 bit
	//image_create(&img_depth_int16_cropped, crop_width, crop_height, IMAGE_OPENCV_DISP);// To store cropped depth - 16 bit
	image_create(&img_middle_int8_cropped,img_cropped_dims.w, img_cropped_dims.h, IMAGE_GRAYSCALE);// To store intermediate image data from processing - 8 bit
	image_create(&img_edges_int8_cropped,img_cropped_dims.w, img_cropped_dims.h, IMAGE_GRAYSCALE);// To store edges image data from processing - 8 bit


	// Creation of kernels:
	// Creating structure to hold dimensions of kernels
	kernel_median_dims.w = 5; kernel_median_dims.h = 5;
	// Creating empty kernel:
	kernel_create(&median_kernel, kernel_median_dims.w, kernel_median_dims.h);


	// Adding callback functions
	cv_add_to_device(&WEDGEBUG_CAMERA_LEFT, copy_left_img_func, WEDGEBUG_CAMERA_LEFT_FPS);
	cv_add_to_device(&WEDGEBUG_CAMERA_RIGHT, copy_right_img_func, WEDGEBUG_CAMERA_RIGHT_FPS);


	//Initialization of constant rotation matrices and transition vectors for frame to frame transformations
	// 1) Rotation matrix and transition vector to transform from world ENU frame to world NED frame
	Rwnedwenu.m[0] = 0; Rwnedwenu.m[1] = 1;	Rwnedwenu.m[2] = 0;
	Rwnedwenu.m[3] = 1; Rwnedwenu.m[4] = 0; Rwnedwenu.m[5] = 0;
	Rwnedwenu.m[6] = 0; Rwnedwenu.m[7] = 0; Rwnedwenu.m[8] = -1;
	VNEDwenu.x = 0;
	VNEDwenu.y = 0;
	VNEDwenu.z = 0;
	// 3) Rotation matrix and transition vector to transform from robot frame to camera frame
	Rcr.m[0] = 0; Rcr.m[1] = 1;	Rcr.m[2] = 0;
	Rcr.m[3] = 0; Rcr.m[4] = 0; Rcr.m[5] = 1;
	Rcr.m[6] = 1; Rcr.m[7] = 0; Rcr.m[8] = 0;
	VCr.x = 0;
	VCr.y = 0;
	VCr.z = 0;

	// Initializing goal vector in the NEW coordinate system
	VGOALwenu.x = WaypointX(WP_GOAL1);
	VGOALwenu.y = WaypointY(WP_GOAL1);
	VGOALwenu.z = WaypointAlt(WP_GOAL1);
	Va_to_Vb(&VGOALwned, &VGOALwenu, &Rwnedwenu, &VNEDwenu);

	// Calculating principal points of normal image and cropped image
	c_img.y = img_dims.h / 2;
	c_img.x = img_dims.w / 2;
	principal_points(&c_img_cropped,&c_img, &img_cropped_info); // Calculates principal points for cropped image, considering the original dimensions

	// Setting thresholds
	threshold_median_disparity = 55; // Above this median disparity, an obstacle is considered to block the way. >60 = close than 35cm
	threshold_disparity_of_edges= 10;



}



void wedgebug_periodic(){
  // your periodic code here.
  // freq = 4.0 Hz
	//printf("Wedgebug periodic function was called\n");


	// No imaes are capturin during the first call of the periodic function
	// So all processing must happen after the first cycle


	//set_state(MOVE_TO_GOAL);
	printf("Current state %d\n", current_state);

	//printf("Angle %f\n", heading_towards_waypoint(WP_GOAL1)); // Figure out how to deal with this waypoint


	//Initialization of dynamic rotation matrices and transition vectors for frame to frame transformations
	// 2) Rotation matrix and transition vector to transform from world ned frame to robot frame
	float_vect_copy(Rrwned.m, stateGetNedToBodyRMat_f()->m, 9);
	VRwned.x = stateGetPositionNed_f()->x;
	VRwned.y = stateGetPositionNed_f()->y;
	VRwned.z = stateGetPositionNed_f()->z;


	/*
	enum navigation_state {
	  POSITION_INITIAL = 1,
	  MOVE_TO_START = 2,
	  POSITION_START = 3,
	  MOVE_TO_GOAL = 4,
	  POSITION_GOAL = 5,
	  WEDGEBUG_START = 6,
	  MOVE_TO_EDGE = 7,
	  SCAN_EDGE = 8,

	};*/



	switch(current_state)
	{
	case POSITION_INITIAL: // 1
	{
		printf("POSITION_INITIAL = %d\n", POSITION_INITIAL);
		// Nothing happens here
	}break;

	case MOVE_TO_START: // 2
	{
		printf("MOVE_TO_START = %d\n", MOVE_TO_START);
		// Nothing happens here
	}break;

	case POSITION_START: // 3
	{
		printf("POSITION_START = %d\n", POSITION_START);
		// Nothing happens here
	}break;

	case MOVE_TO_GOAL: // 4
	{
		printf("MOVE_TO_GOAL = %d\n", MOVE_TO_GOAL);


		// 1. Converting left and right image to 8bit grayscale for further processing
		image_to_grayscale(&img_left, &img_left_int8); // Converting left image from UYVY to gray scale for saving function
		image_to_grayscale(&img_right, &img_right_int8); // Converting right image from UYVY to gray scale for saving function

		// 2. Deriving disparity map from block matching (left image is reference image)
		SBM_OCV(&img_depth_int8_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 1);// Creating cropped disparity map image

		// 3. Morphological operations 1
		// Needed to smoove object boundaries and to remove noise removing noise
		opening_OCV(&img_depth_int8_cropped, &img_middle_int8_cropped,13, 1);
		closing_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,13, 1);
		dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,11, 1);

		// Checking threshold
		// Calculating median disparity
		median_disparity_in_front = median_disparity_to_point(&c_img_cropped, &img_depth_int8_cropped, &median_kernel);

		// THis code is to see what depth the disparity value is
		float depth;
		//In case disparity is 0 (infinite distance or error we set it to one disparity
		// above the threshold as the likelyhood that the object is too close is large (as opposed to it being infinitely far away)
		if(median_disparity_in_front == 0 )
		{
			median_disparity_in_front = (threshold_median_disparity + 1);
		}

		depth = disp_to_depth(median_disparity_in_front, b, f);

		printf("median_disparity_in_front = %d\n", median_disparity_in_front);
		if (median_disparity_in_front > threshold_median_disparity)
		{
			printf("Object detected!!!!!!!!\n");
		}
		printf("depth function = %f\n", depth);

		printf("\n");



		// This statement is needed in order to make sure that the setpoint is only set if current_state is 4
		// AND the current mode is guided mode (otherwise the setpoint might be set before guidance mode
		// is activated and that simply leads to nothing)
		if ((autopilot_get_mode() == AP_MODE_GUIDED))// && (is_goal_setpoint_set == 0))
		{
			// Sets setpoint to goal position and orientates drones towards the goal as well
			autopilot_guided_goto_ned(VGOALwned.x, VGOALwned.y, VGOALwned.z, heading_towards_waypoint(WP_GOAL1));
			is_goal_setpoint_set = 1; // Now the setpoint has been set
		}

		// The following ensures the drone will always face the goal
		//guidance_h_set_guided_heading(heading_towards_waypoint(WP_GOAL1));

		// If the drone is very close to the goal the setpoint is released (=0) and the state
		// is changed to "POSITION_GOAL (5)"
		if (float_vect3_norm_two_points(&VGOALwned, &VRwned) < 0.25)
		{
			is_goal_setpoint_set = 0;
			set_state(POSITION_GOAL);
		}
	}break;

	case POSITION_GOAL: // 5
	{
		printf("POSITION_GOAL = %d\n", POSITION_GOAL);
		// Since the drone is at the goal we will swithc bach to the NAV mode
		autopilot_mode_auto2 = AP_MODE_NAV;
		autopilot_static_set_mode(AP_MODE_NAV);
	}break;

	case WEDGEBUG_START: // 6
	{
		printf("WEDGEBUG_START = %d\n", WEDGEBUG_START);
	}break;

	case MOVE_TO_EDGE: // 7
	{
		printf("MOVE_TO_EDGE = %d\n", MOVE_TO_EDGE);
	}break;

	case SCAN_EDGE: // 8
	{
		printf("SCAN_EDGE = %d\n", SCAN_EDGE);
	}break;

    default: // 0
    {
    	printf("default = %d\n", 0);
    }
    break;

	}




	if (cycle_counter != 0)
	{
		//image_to_grayscale(&img_left, &img_left_int8); // Converting left image from UYVY to gray scale for saving function
		//image_to_grayscale(&img_right, &img_right_int8); // Converting right image from UYVY to gray scale for saving function

		//SBM_OCV(&img_depth_int8_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 1);// Creating cropped disparity map image

		// Morphological operations 1
		// Needed to smoove object boundaries and to remove noise removing noise
		//opening_OCV(&img_depth_int8_cropped, &img_middle_int8_cropped,13, 1);
		//closing_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,13, 1);
		//dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,11, 1);

		// Edge detection
		sobel_OCV(&img_middle_int8_cropped, &img_edges_int8_cropped, 5, 300);

		// Morphological  operations 2
		// This is needed so that when using the edges as filters (to work on disparity values
		// only found on edges) the underlying disparity values are those of the foreground
		// and not the background
		dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,11, 1);






		// Current work--------------------------------------------------------------------------------------------------------------------------------------------------------------------
		// ###################### NEXT --> Set depth threshold and see if medium is below it, if it is return true. Then put this in a function and include it in the "MOVE_TO_GOAL" state







		// Example for finding target point in camera coordinate system - start
		// Creating target point in the camera coordinate system
		struct FloatVect3 target_point;
		target_point.y = -0.251803;
		target_point.x = 0.0671475;
		target_point.z = 6;


		// Loop to calculate position (in image) of point closes to hypothetical target - Start
		uint8_t threshold_disp = 10; //In the loop to determine the cl
		struct crop_t edge_search_area;
		edge_search_area.y = 0;
		edge_search_area.h = img_depth_int8_cropped.h;
		edge_search_area.x = 0;
		edge_search_area.w = img_depth_int8_cropped.w;

		uint32_t n_edges_found = 0;
		uint32_t max_edge_intensity = maximum_intensity(&img_edges_int8_cropped);


		float distance = 255; // This stores distance from edge to goal. Its initialized with 255 as basically any edge found will be closer than that and will replace 255 meters
		struct FloatVect3 Vc2; // Just a vector to save the the current 3d point in the camera coordinate system.
		struct FloatVect2 closest_edge;// A vector to save the closest edge to the goal identified
		float f_distance_to_goal; // Saves distance from edge to goal NOTE: probably should add a vector to store distance from edge point to drone


		for (uint16_t y = edge_search_area.y; y < (edge_search_area.y + edge_search_area.h ); y++)//for (uint32_t y = edge_search_area.y; y < (edge_search_area.y + edge_search_area.h); y++)
		{
			for (uint16_t x = edge_search_area.x; x < (edge_search_area.x + edge_search_area.w ); x++)
			{
				int32_t indx = indx1d_a(y, x, &img_depth_int8_cropped);
				uint8_t edge_value = ((uint8_t*) img_edges_int8_cropped.buf)[indx];
				// We save the disparity of the current point
				uint8_t disparity = ((uint8_t*) img_middle_int8_cropped.buf)[indx];

				// Three conditions must be met for an edge to be considered a viable route for the drone:
				// 1) At least one edge pixels must have been found (maximum_intensity(&img_edges_int8_cropped) != 0)
				//   a) This disparity of the current coordinate (x,y) must coincide with an edge pixel
				//      (as all non-edge pixels have been set to 0) - (edge_value != 0)
				//   b) The disparity of the current coordinate (x, y) must be above a certain threshold - (disparity > threshold_disp)
				if ((max_edge_intensity != 0) && (edge_value != 0) && (disparity > threshold_disp))
				{

					// We increase the edge counter for every edge found
					n_edges_found++;

					// We determine the offset from the principle point
					int32_t y_from_c = y - c_img_cropped.y;
					int32_t x_from_c = x - c_img_cropped.x;
					// We derive the 3d scene point using from the disparity saved earlier
					//Ci_to_Cc(&Vc, y_from_c, x_from_c, disparity, b, f);
					Vi_to_Vc(&Vc2, y_from_c, x_from_c, disparity, b, f);

					// Calculating Euclidean distance (N2)
					f_distance_to_goal =  float_vect3_norm_two_points(&target_point, &Vc2);

					// If current distance (using distance vector) is smaller than the previous minimum distance
					// measure then save new distance and point coordinates associated with it
					if (f_distance_to_goal < distance)
					{
						distance = f_distance_to_goal;
						closest_edge.y = (float)y;
						closest_edge.x = (float)x;
						//y_saved=y;
						//x_saved=x;
						//printf("saved: %d, %d\n", y_saved, x_saved);
						//printf("y,x: %d, %d\n", y, x);
						//printf("saved: %f, %f\n", closest_edge.y, closest_edge.x);

					}

					//printf("x image = %d\n", x);
					//printf("y image = %d\n", y);
					//printf("x image from c = %d\n", x_from_c);
					//printf("y image from c = %d\n", y_from_c);
					//printf("d  = %d\n", disparity);
					//printf("X scene from c = %f\n", scene_point.X);
					//printf("Y scene from c = %f\n", scene_point.Y);
					//printf("Z scene from c = %f\n", scene_point.Z);
					//printf("Closest edge [y,x] = [%d, %d]\n", (int)closest_edge.y, (int)closest_edge.x);
					//printf("Distance to goal (m) = %f\n\n", distance);
				}
			}
		}

		if (n_edges_found > 0)
		{
			((uint8_t*) img_edges_int8_cropped.buf)[indx1d_a(closest_edge.y, closest_edge.x, &img_depth_int8_cropped)] = 255;
			//printf("Viable edge found\n");
		}
		else
		{
			//printf("No viable edges were found\n");
			1+1;
		}


		//printf("\n");



		//printf("median / disparity / depth = %d / %d / %f\n",median, disparity2 ,Vc3.z);

		//printf("Closest edge [y,x] = [%d, %d]\n", closest_edge.y, closest_edge.x);
	    //printf("Distance to goal (m) = %f\n\n", distance);
	    // Loop to calculate position (in image) of point closes to hypothetical target - End
		// Example for finding target point in camera coordinate system - End
		//printf("Width/height = %d / %d\n", img_depth_int8_cropped.w, img_depth_int8_cropped.h);
		//printf("Position of y coordinate %d and x coordinate %d in 1d array: %d\n", position_2d.y, position_2d.x, position_1d);
		/*
		uint16_t x = 0;
		float depth;
		if (x==0)
		{
			depth = 0.0001;
		}
		else
		{
			depth = b * f / x;
		}
		printf("depth of 64 disparity: %f\n", depth);


		*/

		//save_image_gray(&img_YY, "/home/dureade/Documents/paparazzi_images/img_YY.bmp");
		save_image_gray(&img_left_int8, "/home/dureade/Documents/paparazzi_images/img_left_int8.bmp");
		save_image_gray(&img_right_int8, "/home/dureade/Documents/paparazzi_images/img_right_int8.bmp");

		//save_image_gray(&img_depth_int8, "/home/dureade/Documents/paparazzi_images/img_depth_int8.bmp");
		//save_image_gray(&img_depth_int16, "/home/dureade/Documents/paparazzi_images/img_depth_int16.bmp");

		save_image_gray(&img_depth_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int8_cropped.bmp");
		//save_image_gray(&img_depth_int16_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int16_cropped.bmp");

		save_image_gray(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_middle_int8_cropped.bmp");

		save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped.bmp");


		/*
		//save_image_gray(&img_YY, "/home/dureade/Documents/paparazzi_images/img_YY.bmp");
		save_image_gray(&img_left_int8, "/home/dureade/Documents/paparazzi_images/img_left_int8.bmp");
		save_image_gray(&img_right_int8, "/home/dureade/Documents/paparazzi_images/img_right_int8.bmp");

		//save_image_gray(&img_depth_int8, "/home/dureade/Documents/paparazzi_images/img_depth_int8.bmp");
		//save_image_gray(&img_depth_int16, "/home/dureade/Documents/paparazzi_images/img_depth_int16.bmp");

		save_image_gray(&img_depth_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int8_cropped.bmp");
		//save_image_gray(&img_depth_int16_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int16_cropped.bmp");

		save_image_gray(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_middle_int8_cropped.bmp");

		save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped.bmp");

		*/





		/*

			uint8_t min_8, max_8;
		int16_t min_16, max_16;

		for (int i = 0; i < (img_depth_int8.h * img_depth_int16.h); i++)
		{
			if  (i==0)
			{
				 min_8 = ((uint8_t*)img_depth_int8.buf)[i];
				 max_8 = ((uint8_t*)img_depth_int8.buf)[i];
				 min_16 = ((int16_t*)img_depth_int16.buf)[i];
				 max_16 = ((int16_t*)img_depth_int16.buf)[i];



				//max_8 = ((uint8_t)img_depth_int8).buf)[i];
				//min_16 = ((int16_t)img_depth_int8.buf)[i];
				//max_16 = ((int16_t)img_depth_int8.buf)[i];
				printf("Stuff called: %d\n", img_depth_int8.buf_size);
			}
			else
			{
				if (min_8 > ((uint8_t*)img_depth_int8.buf)[i]){min_8 = ((uint8_t*)img_depth_int8.buf)[i];;}
				if (max_8 < ((uint8_t*)img_depth_int8.buf)[i]){max_8 = ((uint8_t*)img_depth_int8.buf)[i];;}

				if (min_16 > ((int16_t*)img_depth_int16.buf)[i]){min_16 = ((int16_t*)img_depth_int16.buf)[i];;}
				if (max_16 < ((int16_t*)img_depth_int16.buf)[i]){max_16 = ((int16_t*)img_depth_int16.buf)[i];;}
			}

		}
		printf("8bit depth - min:max: %d:%d\n", min_8, max_8);
		printf("16bit depth - min:max: %d:%d\n", min_16, max_16);


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

	// Since the counter is not infinite it is only increased in the first 10 cycles
	if (cycle_counter < 10){cycle_counter++;}
}



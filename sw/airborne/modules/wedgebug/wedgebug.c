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
 *
 *Note.
 *1. Information on different flight modes (such as AP_MODE_GUIDED) can be found here paparazzi/sw/airborne/firmwares/rotorcraft/autopilot_static.h
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
#include <math.h> // needed for basic math functions
#include "autopilot.h" // Needed to set states (GUIDED vs NAV)
#include <time.h> // Needed to measure time


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

// Declaring images
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

// Declaring crop_t structure for information about the cropped image (after BM)
struct crop_t img_cropped_info;

// Declaring dimensions of images and kernels used
struct img_size_t img_dims;
struct img_size_t img_cropped_dims;
struct img_size_t kernel_median_dims;

// Declaring empty kernel for obtaining median
struct kernel_C1 median_kernel;

// Delcaring structuring element sizes
int SE_opening_OCV; // SE size for the opening operation
int SE_closing_OCV; // SE size for the closing operation
int SE_dilation_OCV_1; // SE size for the first dilation operation
int SE_dilation_OCV_2; // SE size for the second dilation operation (see state 6 "WEDGEBUG_START" )
int SE_sobel_OCV; // SE size for the sobel operation, to detect edges


// Declaring vectors to hold global 3d points
struct FloatVect3 VSTARTwenu; 			// Declared vector of coordinates of start position in ENU world coordinate system
struct FloatVect3 VSTARTwned; 			// Declared vector of coordinates of start position in NED world coordinate system
struct FloatVect3 VGOALwenu; 			// Declared vector of coordinates of goal in ENU world coordinate system
struct FloatVect3 VGOALwned; 			// Declared vector of coordinates of goal in NED world coordinate system
struct FloatVect3 VGOALr;    			// Declared vector of coordinates of goal in robot coordinate system
struct FloatVect3 VGOALc;    			// Declared vector of coordinates of goal in camera coordinate system
struct FloatVect3 VEDGECOORDINATESc; 	// Declared vector of coordinates of "best" edge detected in camera coordinate system
struct FloatVect3 VEDGECOORDINATESr; 	// Declared vector of coordinates of "best" edge detected in robot coordinate system
struct FloatVect3 VEDGECOORDINATESwned; // Declared vector of coordinates of "best" edge detected in NED world coordinate system
struct FloatVect3 VEDGECOORDINATESwenu;	// Declared vector of coordinates of "best" edge detected in ENU world coordinate system
struct FloatVect3 VDISTANCEPOSITIONwned;// Declared a vector to hold the current position, which is needed for calculating the distance traveled



// Thresholds
int threshold_edge_magnitude;			// Edges with a magnitude above this value are detected. Above this value, edges are given the value 127, otherwise they are given the value zero.
uint8_t threshold_median_disparity; 	// Above this median disparity, an obstacle is considered to block the way (i.e. the blocking obstacle need to be close)
uint8_t threshold_disparity_of_edges; 	// Above this disparity edges are eligible for WedgeBug algorithm (i.e. edges cannot be very far away)
float threshold_distance_to_goal; 		// Below this distance (in meters) it is considered that the robot has reached the goal
float threshold_distance_to_angle;		// Below this distance (in radians) it is considered that the robot has reached the target angle

// Declaring confidence parameters
int16_t obstacle_confidence;		// This is the confidence that an obstacle was spotted
int16_t free_path_confidence;		// This is the confidence that no obstacle was spotted
int16_t position_confidence;		// This is the confidence that the desired position was reached
int16_t heading_confidence;			// This is the confidence that the desired heading is reached
int16_t edge_found_confidence;		// This is the confidence that an edge was found
int16_t max_obstacle_confidence;	// This is the max confidence that an obstacle was spotted
int16_t max_free_path_confidence;	// This is the max confidence that an obstacle was not spotted
int16_t max_position_confidence;	// This is the max confidence that a specific position was reached
int16_t max_heading_confidence;		// This is the max confidence that a specific heading was reached
int16_t max_edge_found_confidence; 	// This is the max confidence that edges were found

// Declaring boolean flags
uint8_t is_start_reached_flag;		// Set to 1 if start position is reached, 0 otherwise.
uint8_t is_setpoint_reached_flag;	// Set to 1 if setpoint is reached, 0 otherwise.
uint8_t is_obstacle_detected_flag;	// Set to 1 if obstacle is detected, 0 otherwise.
uint8_t is_path_free_flag;			// Set to 1 if no obstacle is detected, 0 otherwise.
uint8_t is_heading_reached_flag;	// Set to 1 if heading is reached, 0 otherwise.
uint8_t is_best_edge_found_flag;	// Set to 1 if best edge was identified, 0 otherwise
uint8_t is_state_changed_flag; 		// Set to 1 if state was changed, 0 otherwise

// Declaring principal points
struct point_t c_img;				// Principal point of normal camera images
struct point_t c_img_cropped;		// Principal point of cropped camera images

// Declaring edge search area
struct crop_t edge_search_area; // This structure holds information about the window in which edges are searched in


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
  POSITION_EDGE = 8,
  EDGE_SCAN = 9
};
enum navigation_state current_state ;// Default state is 0 i.e. nothing

// This is a structure to save angles that are needed for the "Edge Scan" state
struct ES_angles {
	float heading_initial;  // This is the initial heading of the robot when the "Edge Scan" state is entered
	float heading_max_left; // This is the maximum left turn angle (determines how much to the left the robot can look to search for edges), to search for edges
	float heading_max_right;// This is the maximum right turn angle (determines how much to the right the robot can look to search for edges), to search for edges
	uint8_t initiated;		// This is a flag that can be set to check whether the structure is allowed to be overwritten (0=allowed, 1=forbidden)
	uint8_t is_left_reached_flag;	// This is a flag to check whether the left was scanned for an edge already
	uint8_t is_right_reached_flag;	// This is a flag to check whether the right was scanned for an edge already
};

// This is a structure to save the initial position a drone is in at the beginning of a state
struct initial_position{
	float x;
	float y;
	float z;
	uint8_t initiated; // This is a flag that can be set to check whether the structure is allowed to be overwritten (0=allowed, 1=forbidden)
};


struct initial_position IPwned_state_1; // Structure to save initial position of state 1 (POSITION_INITIAL)
struct initial_position IPwned_state_2; // Structure to save initial position of state 2 (MOVE_TO_START)
struct initial_position IPwned_state_3; // Structure to save initial position of state 3 (POSITION_START)
struct initial_position IPwned_state_4; // Structure to save initial position of state 4 (MOVE_TO_GOAL)
struct initial_position IPwned_state_5; // Structure to save initial position of state 5 (POSITION_GOAL)
struct initial_position IPwned_state_6; // Structure to save initial position of state 6 (WEDGEBUG_START)
struct initial_position IPwned_state_7; // Structure to save initial position of state 7 (MOVE_TO_EDGE)
struct initial_position IPwned_state_8; // Structure to save initial position of state 8 (POSITION_EDGE)
struct initial_position IPwned_state_9; // Structure to save initial position of state 9 (EDGE_SCAN)
struct ES_angles initial_state;


// Declaring variables for time measurement
double time_state[NUMER_OF_STATES];	 	// Double array for saving total time (clock cycles) spent in the states (position 0 = state 0 and so on)
double counter_state[NUMER_OF_STATES];//  A counter to measure the total cycles that each state in the FSM (within the periodic function) went through
double counter_cycles; 				// A counter to measure the total cycles that the periodic function went through
clock_t clock_total_time; 				// Clock to measure total time (clock cycles)) it took for the robot to fly from start to goal
clock_t clock_total_time_current; 		// Clock to hold time measured at start of the current cycle of the periodic function
clock_t clock_total_time_previous; 		// Clock to hold time measured at start of the previous cycle of the periodic function

// Other declarations
uint8_t previous_state; // Variable that saves previous state the state machine was in, for some memory
//static pthread_mutex_t mutex;
int N_disparities = 64;
int block_size_disparities = 25;
int min_disparity = 0;
float heading; // Varibale for storing the heading of the drone (psi in radians)
float max_edge_search_angle = M_PI/2;
uint8_t median_disparity_in_front; // Variable to hold the median disparity in front of the drone. Needed to see if obstacle is there.
float distance_traveled; // Variable to hold the distance traveled of the robot (since start and up to the goal)
uint8_t number_of_states; // Variable to save the total number of states used in the finite state machine

// For debugging purpose. Allows for changing of state in simulation if 1. If 0, does not allow for state change. Useful if you want to execute only one state repeatedly
uint8_t allow_state_change_1; // From within state "POSITION_INITIAL"
uint8_t allow_state_change_2; // From within state "MOVE_TO_START"
uint8_t allow_state_change_3; // From within state "POSITION_START "
uint8_t allow_state_change_4; // From within state "MOVE_TO_GOAL"
uint8_t allow_state_change_5; // From within state "POSITION_GOAL"
uint8_t allow_state_change_6; // From within state "WEDGEBUG_START"
uint8_t allow_state_change_7; // From within state "MOVE_TO_EDGE"
uint8_t allow_state_change_8; // From within state "POSITION_EDGE"
uint8_t allow_state_change_9; // From within state "EDGE_SCAN"


// New section: Functions - Declaration ----------------------------------------------------------------------------------------------------------------
// Supporting
const char* get_img_type(enum image_type img_type); // Function 1: Displays image type
void show_image_data(struct image_t *img); // Function 2: Displays image data
void show_image_entry(struct image_t *img, int entry_position, const char *img_name); // Function 3: Displays pixel value of image

// External
void post_disparity_crop_rect(struct crop_t *img_cropped_info,struct img_size_t *original_img_dims, const int disp_n, const int block_size);
void set_state(uint8_t state, uint8_t change_allowed);
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
float heading_towards_setpoint_WNED(struct FloatVect3 *VSETPOINTwned);
uint8_t median_disparity_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median);

uint8_t find_best_edge_coordinates(struct FloatVect3 *VEDGECOORDINATESc, struct FloatVect3 *VTARGETc, struct image_t *img_edges, struct image_t *img_disparity, struct crop_t *edge_search_area, uint8_t threshold);
uint8_t is_setpoint_reached(struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold);

float float_norm_two_angles(float target_angle, float current_angle);
uint8_t is_heading_reached(float target_angle, float current_angle, float threshold);
uint8_t are_setpoint_and_angle_reached(struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold_setpoint, float target_angle, float current_angle, float threshold_angle);





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
void set_state(uint8_t state, uint8_t change_allowed)
{
	if (change_allowed == 1){current_state = state;}
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

	struct FloatVect3 Va_temp;
	Va_temp.x = Va->x;
	Va_temp.y = Va->y;
	Va_temp.z = Va->z;


	// The following translates world vector coordinates into the agent coordinate system
	Va_temp.x = Va->x - VOa->x;
	Va_temp.y = Va->y - VOa->y;
	Va_temp.z = Va->z - VOa->z;

	// In case the axes of the world coordinate system (w) and the agent coordinate system (a) do not
	// coincide, they are adjusted with the rotation matrix R
	float_rmat_vmult(Vb, Rba, &Va_temp);
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

// Function 16a - function to calculate angle between robot and specific waypoint
float heading_towards_waypoint(uint8_t wp)
{
  struct FloatVect2 VWPwt = {WaypointX(wp), WaypointY(wp)};
  struct FloatVect2 difference;
  float angle;

  VECT2_DIFF(difference, VWPwt, *stateGetPositionEnu_f());
  angle = atan2f(difference.x, difference.y);
  return angle;
}


// Function 16b - function to calculate angle between robot and specific setpoint
float heading_towards_setpoint_WNED(struct FloatVect3 *VSETPOINTwned)
{
  struct FloatVect2 VSETPOINTwned2d = {VSETPOINTwned->x, VSETPOINTwned->y};
  struct FloatVect2 difference;
  float angle;

  VECT2_DIFF(difference, VSETPOINTwned2d, *stateGetPositionNed_f());
  angle = atan2f(difference.x, difference.y);
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

	// In case the upper bounds of the kernel are outside of the image area
	// (lower bound not important because of uint8_t type converting everything below 0 to 0):
	if (VSTOPi_y > img->h)
	{
		VSTOPi_y = (img->h - 1);
	}
	if (VSTOPi_x > img->w)
	{
		VSTOPi_x = (img->w - 1);
	}



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

// Function 18 - Function to find "best" (vlosest ideal pathwat to goal from robot to edge to goal) edgef
// Returns a 3d Vector to the best "edge" and 1 if any edge is found and 0 if no edge is found.
uint8_t find_best_edge_coordinates(
		struct FloatVect3 *VEDGECOORDINATESc,
		struct FloatVect3 *VTARGETc,
		struct image_t *img_edges,
		struct image_t *img_disparity,
		struct crop_t *edge_search_area,
		uint8_t threshold)
{

	// Loop to calculate position (in image) of point closes to hypothetical target - Start


	uint32_t n_edges_found = 0;

	float distance = 255; // This stores distance from edge to goal. Its initialized with 255 as basically any edge found will be closer than that and will replace 255 meters
	struct FloatVect3 VEDGEc; // A vector to save the point of an eligible detected edge point in the camera coordinate system.
	struct FloatVect3 VROBOTCENTERc;// Declaring camera center coordinates vector
	VROBOTCENTERc.x = 0.0;VROBOTCENTERc.y = 0.0;VROBOTCENTERc.z = 0.0; // Initializing camera center coordinates vector
	struct point_t VCLOSESTEDGEi; // A vector to save the point of the "best" eligible detected edge point in the image coordinate system.
	float f_distance_edge_to_goal;  // Saves distance from edge to goal
	float f_distance_robot_to_edge; // Saves distance from robot to goal
	int32_t indx; // Variable to store 1d index calculated from 2d index
	uint8_t edge_value; // Variable to store the intensity value of a pixel in the img_edge
	uint8_t disparity; // variable to store the disparity level of a pixel in the img_disparity
	uint8_t disparity_best;


	for (uint16_t y = edge_search_area->y; y < (edge_search_area->y + edge_search_area->h ); y++)//for (uint32_t y = edge_search_area.y; y < (edge_search_area.y + edge_search_area.h); y++)
	{
		for (uint16_t x = edge_search_area->x; x < (edge_search_area->x + edge_search_area->w ); x++)
		{
			indx = indx1d_a(y, x, img_edges); // We convert the 2d index [x,y] into a 1d index
			edge_value = ((uint8_t*) img_edges->buf)[indx]; // We save the intensity of the current point
			disparity = ((uint8_t*) img_disparity->buf)[indx]; // We save the disparity of the current point

			// Two conditions must be met for an edge to be considered a viable route for the drone:
			// 1) This disparity of the current coordinate (x,y) must coincide with an edge pixel
			//    (as all non-edge pixels have been set to 0) - (edge_value != 0)
			// 2) The disparity of the current coordinate (x, y) must be above a certain threshold. This simulates vision cone - (disparity > threshold_disparity_of_edges)
			if ((edge_value != 0) && (disparity > threshold))
			{
				// We increase the edge counter for every edge found
				n_edges_found++;
				// We determine the offset from the principle point
				int32_t y_from_c = y - c_img_cropped.y; // NOTE. The variable "c_img_cropped" is a global variable
				int32_t x_from_c = x - c_img_cropped.x; // NOTE. The variable "c_img_cropped" is a global variable
				// We derive the 3d scene point using from the disparity saved earlier
				Vi_to_Vc(&VEDGEc, y_from_c, x_from_c, disparity, b, f); // NOTE. The variables "b" and "f" are a global variables
				// Calculating Euclidean distance (N2) - Edge to goal
				f_distance_edge_to_goal =  float_vect3_norm_two_points(VTARGETc, &VEDGEc);
				// Calculating Euclidean distance (N2) - robot to edge
				f_distance_robot_to_edge =  float_vect3_norm_two_points(&VEDGEc, &VROBOTCENTERc);


				// If current distance (using distance vector) is smaller than the previous minimum distance
				// measure then save new distance and point coordinates associated with it
				if ((f_distance_robot_to_edge + f_distance_edge_to_goal) < distance)
				{
					// Saving closest edge point, in camera coordinate system
					VEDGECOORDINATESc->x = VEDGEc.x;
					VEDGECOORDINATESc->y = VEDGEc.y;
					VEDGECOORDINATESc->z = VEDGEc.z;
					// Saving disparity at point
					disparity_best = disparity;
					// Saving smallest distance
					distance = (f_distance_robot_to_edge + f_distance_edge_to_goal);
					// Saving closest edge point, in image coordinate system
					VCLOSESTEDGEi.y = y;
					VCLOSESTEDGEi.x = x;

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
				//printf("Robot center coordinates = [%f, %f, %f]\n", VROBOTCENTERc.x, VROBOTCENTERc.y, VROBOTCENTERc.z);
				//printf("Edge coordinates = [%f, %f, %f]\n", VEDGEc.x, VEDGEc.y, VEDGEc.z);
				//printf("Distance to goal (m) = %f\n", distance);
				//printf("Distance to goal2 (m) = %f + %f\n\n", f_distance_robot_to_edge, f_distance_edge_to_goal);
			}
		}
	}

	if (n_edges_found > 0)
	{
		((uint8_t*) img_edges->buf)[indx1d_a(VCLOSESTEDGEi.y, VCLOSESTEDGEi.x, img_edges)] = 255;
		printf("Viable closest edge found: [%d, %d] (disparity = %d)\n", VCLOSESTEDGEi.y , VCLOSESTEDGEi.x, disparity_best);

		printf("At distance: %f\n", distance);
		return 1;
	}
	else
	{
		printf("No viable edge found\n");
		return 0;
	}
}


// Function 19 - Function to determine if setpoint was reached (it was reached if distance is below a set threshold)
uint8_t is_setpoint_reached(struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold)
{
	if (float_vect3_norm_two_points(VGOAL, VCURRENTPOSITION) < threshold)	{return 1;}
	else 																	{return 0;}
}


// Function 20  - function to obtain the Euclidian distance between two angles (in radians)
float float_norm_two_angles(float target_angle, float current_angle)
{
	float target_current_diff;
	target_current_diff = target_angle - current_angle;
	//printf("Target angle = %f\n", target_angle);
	//printf("Current angle = %f\n", current_angle);
	//printf("difference in function = %f\n", sqrt(pow(target_current_diff, 2)));
	return sqrt(pow(target_current_diff, 2));
}

// Function 21 - Function to determine if set heading was reached (it was reached if the Euclidean distance is below a set threshold)
uint8_t is_heading_reached(float target_angle, float current_angle, float threshold)
{
	if (float_norm_two_angles(target_angle, current_angle) < threshold)	{return 1;}
	else 																{return 0;}
}

// Function 22
uint8_t are_setpoint_and_angle_reached(
		struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold_setpoint,
		float target_angle, float current_angle, float threshold_angle)
{
	if ((float_vect3_norm_two_points(VGOAL, VCURRENTPOSITION) < threshold_setpoint) &&(float_norm_two_angles(target_angle, current_angle) < threshold_angle))	{return 1;}
	else 																																						{return 0;}
}


// Function 23
void initial_state_create(struct initial_position *IPwned, struct FloatVect3 *Vwned)
{
	IPwned->x = Vwned->x;
	IPwned->y = Vwned->y;
	IPwned->z = Vwned->z;
	IPwned->initiated = 1; // Initiated flag
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
	// 2) Rotation matrix and transition vector to transform from robot frame to camera frame
	Rcr.m[0] = 0; Rcr.m[1] = 1;	Rcr.m[2] = 0;
	Rcr.m[3] = 0; Rcr.m[4] = 0; Rcr.m[5] = 1;
	Rcr.m[6] = 1; Rcr.m[7] = 0; Rcr.m[8] = 0;
	VCr.x = 0;
	VCr.y = 0;
	VCr.z = 0;

	// Initializing goal vector in the NED world coordinate system
	VGOALwenu.x = WaypointX(WP_GOAL1);
	VGOALwenu.y = WaypointY(WP_GOAL1);
	VGOALwenu.z = WaypointAlt(WP_GOAL1);
	Va_to_Vb(&VGOALwned, &VGOALwenu, &Rwnedwenu, &VNEDwenu);


	// Initializing start vector in the NED world coordinate system
	VSTARTwenu.x = WaypointX(WP_START1);
	VSTARTwenu.y = WaypointY(WP_START1);
	VSTARTwenu.z = WaypointAlt(WP_START1);
	Va_to_Vb(&VSTARTwned, &VSTARTwenu, &Rwnedwenu, &VNEDwenu);


	// Calculating principal points of normal image and cropped image
	c_img.y = img_dims.h / 2;
	c_img.x = img_dims.w / 2;
	principal_points(&c_img_cropped,&c_img, &img_cropped_info); // Calculates principal points for cropped image, considering the original dimensions


	// Initializing structuring element sizes
	SE_opening_OCV = 13; 	// SE size for the opening operation
	SE_closing_OCV = 13; 	// SE size for the closing operation
	SE_dilation_OCV_1 = 201;// SE size for the first dilation operation (Decides where edges are detected, increase to increase drone safety zone NOTE. This functionality should be replaced with c space expansion)
	SE_dilation_OCV_2 = 11; // SE size for the second dilation operation (see state 6 "WEDGEBUG_START" )
	SE_sobel_OCV = 5; 		// SE size for the sobel operation, to detect edges


	// Setting thresholds
	threshold_median_disparity = 11; 		// Above this median disparity, an obstacle is considered to block the way. >60 = close than 35cm
	threshold_edge_magnitude = 300;  		// Edges with a magnitude above this value are detected. Above this value, edges are given the value 127, otherwise they are given the value zero.
	threshold_disparity_of_edges = 5; 		// Above this underlying disparity value, edges are considers eligible for detection
	threshold_distance_to_goal = 0.25; 		// Above this threshold, the goal is considered reached
	threshold_distance_to_angle = 0.0004;	// Above this threshold, the angle/heading is considered reached

	// Initializing confidence parameters
	obstacle_confidence = 0;		// This is the confidence that an obstacle was spotted
	free_path_confidence = 0;		// This is the confidence that no obstacle was spotted
	position_confidence = 0;		// This is the confidence that the desired position was reached
	heading_confidence = 0;			// This is the confidence that the desired heading is reached
	edge_found_confidence = 0;		// This is the confidence that an edge was found
	max_obstacle_confidence = 3;	// This is the max confidence that an obstacle was spotted
	max_free_path_confidence = 10;		// This is the max confidence that an obstacle was not spotted
	max_position_confidence = 30;	// This is the max confidence that a specific position was reached
	max_heading_confidence = 5;		// This is the max confidence that a specific heading was reached
	max_edge_found_confidence = 10;	// This is the max confidence that edges were found

	// Initializing boolean flags
	is_start_reached_flag = 0;		// Set to 1 if start position is reached, 0 otherwise
	is_setpoint_reached_flag = 0;	// Set to 1 if setpoint is reached, 0 otherwise
	is_obstacle_detected_flag = 0;	// Set to 1 if obstacle is detected, 0 otherwise
	is_path_free_flag = 0;			// Set to 1 if no obstacle is detected, 0 otherwise
	is_heading_reached_flag = 0;	// Set to 1 if heading is reached, 0 otherwise
	is_best_edge_found_flag = 0; 	// Set to 1 if best edge was found, 0 otherwise
	is_state_changed_flag = 0; 	// Set to 1 if state was changed, 0 otherwise


	// Initializing area over which edges are searched in
	edge_search_area.y = 0;
	edge_search_area.h = img_depth_int8_cropped.h;
	edge_search_area.x = 0;
	edge_search_area.w = img_depth_int8_cropped.w;

	// Initializing Edge scan structure
	initial_state.initiated = 0; 			// 0 = it can be overwritten
	initial_state.is_left_reached_flag = 0;	// The scan has not reached the left maximum angle yet
	initial_state.is_right_reached_flag = 0;// The scan has not reached the right maximum angle yet


	// Initializing  initial_position structures of states
	IPwned_state_1.initiated = 0; // Structure to save initial position of state 1 (POSITION_INITIAL)
	IPwned_state_2.initiated = 0; // Structure to save initial position of state 2 (MOVE_TO_START)
	IPwned_state_3.initiated = 0; // Structure to save initial position of state 3 (POSITION_START)
	IPwned_state_4.initiated = 0; // Structure to save initial position of state 4 (MOVE_TO_GOAL)
	IPwned_state_5.initiated = 0; // Structure to save initial position of state 5 (POSITION_GOAL)
	IPwned_state_6.initiated = 0; // Structure to save initial position of state 6 (WEDGEBUG_START)
	IPwned_state_7.initiated = 0; // Structure to save initial position of state 7 (MOVE_TO_EDGE)
	IPwned_state_8.initiated = 0; // Structure to save initial position of state 8 (POSITION_EDGE)
	IPwned_state_9.initiated = 0; // Structure to save initial position of state 9 (EDGE_SCAN)


	// Initializing debugging options
	allow_state_change_1 = 1; // Allows state change from within state "POSITION_INITIAL"
	allow_state_change_2 = 1; // Allows state change from within state "MOVE_TO_START"
	allow_state_change_3 = 1; // Allows state change from within state "POSITION_START "
	allow_state_change_4 = 1; // Allows state change from within state "MOVE_TO_GOAL"
	allow_state_change_5 = 1; // Allows state change from within state "POSITION_GOAL"
	allow_state_change_6 = 1; // Allows state change from within state "WEDGEBUG_START"
	allow_state_change_7 = 1; // Allows state change from within state "MOVE_TO_EDGE"
	allow_state_change_8 = 1; // Allows state change from within state "POSITION_EDGE"
	allow_state_change_9 = 1; // Allows state change from within state "EDGE_SCAN"

	// Other initializations
	previous_state = 0;						// Variable for state machine memory
	VDISTANCEPOSITIONwned.x = VSTARTwned.x;	// Initializing a vector to hold the current position, which is needed for calculating the distance traveled
	VDISTANCEPOSITIONwned.y = VSTARTwned.y;
	VDISTANCEPOSITIONwned.z = VSTARTwned.z;
	clock_total_time_previous = 0;
	distance_traveled = 0; 					// Variable to hold the distance traveled of the robot (since start and up to the goal)
	number_of_states = NUMER_OF_STATES;     // Variable to save the total number of states used in the finite state machine

	/*
	enum navigation_state {
	  POSITION_INITIAL = 1,
	  MOVE_TO_START = 2,
	  POSITION_START = 3,
	  MOVE_TO_GOAL = 4,
	  POSITION_GOAL = 5,
	  WEDGEBUG_START = 6,
	  MOVE_TO_EDGE = 7,
	  POSITION_EDGE = 8,
  	  EDGE_SCAN = 9

	};*/


	//set_state(0 ,1);


}








void wedgebug_periodic(){
  // your periodic code here.
  // freq = 15.0 Hz
	//printf("Wedgebug periodic function was called\n");

	// Debugging - setting default state
	//set_state(MOVE_TO_GOAL ,1);
	printf("Current state %d\n", current_state);



	// Cycle-dependent initializations
	//Initialization of dynamic rotation matrices and transition vectors for frame to frame transformations
	// Rotation matrix and transition vector to transform from world ned frame to robot frame
	float_vect_copy(Rrwned.m, stateGetNedToBodyRMat_f()->m, 9);
	VRwned.x = stateGetPositionNed_f()->x;
	VRwned.y = stateGetPositionNed_f()->y;
	VRwned.z = stateGetPositionNed_f()->z;

	// Initialization of robot heading
	heading = stateGetNedToBodyEulers_f()->psi;

	// Initialization of robot location/orientation-dependent variables
	// Converting goal world NED coordinates into Camera coordinates
	Va_to_Vb(&VGOALr, &VGOALwned, &Rrwned, &VRwned);
	Va_to_Vb(&VGOALc, &VGOALr, &Rcr, &VCr);



	// Checking is state was changed, if yes then all glas are reset and the is_state_changed_flag
	// is set to 1 for this cycle only. Else, the  is_state_changed_flag is set to 0 again;
	if (current_state != previous_state)
	{
		// Setting flag signifying that the state was changed
		is_state_changed_flag = 1;
		// Reset flags
		is_start_reached_flag = 0;				// Set to 1 if start position is reached, 0 otherwise
		is_setpoint_reached_flag = 0;			// Set to 1 if setpoint is reached, 0 otherwise
		is_obstacle_detected_flag = 0;			// Set to 1 if obstacle is detected, 0 otherwise
		is_path_free_flag = 0;					// Set to 1 if no obstacle is detected, 0 otherwise
		is_heading_reached_flag = 0;			// Set to 1 if heading is reached, 0 otherwise
		is_best_edge_found_flag = 0; 			// Set to 1 if best edge was found, 0 otherwise
		initial_state.initiated = 0; 			// 0 = it can be overwritten
		initial_state.is_left_reached_flag = 0;	// The scan has not reached the left maximum angle yet
		initial_state.is_right_reached_flag = 0;// The scan has not reached the right maximum angle yet
	}
	else if(is_state_changed_flag != 0)
	{
		is_state_changed_flag = 0;
	}




	// ############ Metric 1 - Runtime total
	// If the current state is not 0 (default) and position initial and move to start and position goal, record time elapsed
	if ((current_state != 0) && (current_state != POSITION_INITIAL) && (current_state != MOVE_TO_START) && (current_state != POSITION_START)&& (current_state != POSITION_GOAL))
	{
		// Recording current time
		clock_total_time_current = clock();
		// In case we are in the position start state and the state has changed, initialize clock_total_time_previous
		if ((current_state == MOVE_TO_GOAL) && is_state_changed_flag && (previous_state == POSITION_START))
		{
			clock_total_time_previous = clock_total_time_current;
		}
		//Else check time difference to previous cycle and add to clock_total_time
		else
		{
			clock_total_time = clock_total_time + (clock_total_time_current - clock_total_time_previous);
			clock_total_time_previous = clock_total_time_current;
		}
	}

	// Initializing previous_state variable for next cycle
	// This happens here and not above as the metric above depends on the previous state
	previous_state = current_state;


	// ############ Metric 2 - Distance traveled (total)
	// If the current state is not 0 (default) and position initial and move to start and position goal, record distance traveled
	if ((current_state != 0) && (current_state != POSITION_INITIAL) && (current_state != MOVE_TO_START) && (current_state != POSITION_START) && (current_state != POSITION_GOAL))
	{
		//printf("\n\nSate change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
		distance_traveled = distance_traveled + float_vect3_norm_two_points(&VDISTANCEPOSITIONwned, &VRwned);

		VDISTANCEPOSITIONwned.x = VRwned.x;
		VDISTANCEPOSITIONwned.y = VRwned.y;
		VDISTANCEPOSITIONwned.z = VRwned.z;
	}

	// ############ Metric 3 - Runtime average of background processes (see below) - Start:
	clock_t clock_background_processes; // Creating variable to hold time (number of cycles)
	clock_background_processes = clock(); // Saving current time, way below it is used to calculate time spent in a cycle
	counter_cycles++; // Counting how many times a state was activated (needed for average calculation)


	//Background processes
	// 1. Converting left and right image to 8bit grayscale for further processing
	image_to_grayscale(&img_left, &img_left_int8); // Converting left image from UYVY to gray scale for saving function
	image_to_grayscale(&img_right, &img_right_int8); // Converting right image from UYVY to gray scale for saving function

	// 2. Deriving disparity map from block matching (left image is reference image)
	SBM_OCV(&img_depth_int8_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities, 1);// Creating cropped disparity map image

	/*
	// Optional thresholding of disparity map
	uint8_t thresh = 1;
	for (int32_t i = 0; i < (img_depth_int8_cropped.h*img_depth_int8_cropped.w); i++)
	{
		uint8_t disparity = ((uint8_t*)img_depth_int8_cropped)[i];
		if(disparity < thresh)
		{
			((uint8_t*)img_depth_int8_cropped)[i] = 0; // if below disparity assume object is indefinately away
		}
	}*/


	// 3. Morphological operations 1
	// Needed to smoove object boundaries and to remove noise removing noise
	opening_OCV(&img_depth_int8_cropped, &img_middle_int8_cropped, SE_opening_OCV, 1);
	closing_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped, SE_closing_OCV, 1);
	dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,SE_dilation_OCV_1, 1);

	// 4. Sobel edge detection
	sobel_OCV(&img_middle_int8_cropped, &img_edges_int8_cropped, SE_sobel_OCV, threshold_edge_magnitude);





	// ############ Metric 3 - Runtime average of background processes (see below) - End:
	clock_background_processes = clock_background_processes + (clock() - clock_background_processes);;


	// ############ Metric 4 - Runtime average per state - Start:
	clock_t clock_FSM; // Creating variable to hold time (number of cycles)
	clock_FSM = clock(); // Saving current time, way below it is used to calculate time spent in a cycle
    counter_state[current_state]++; // Counting how many times a state was activated (needed for average calculation). This is done here as state may change in FSM

	/*
	enum navigation_state {
	  POSITION_INITIAL = 1,
	  MOVE_TO_START = 2,
	  POSITION_START = 3,
	  MOVE_TO_GOAL = 4,
	  POSITION_GOAL = 5,
	  WEDGEBUG_START = 6,
	  MOVE_TO_EDGE = 7,
	  POSITION_EDGE = 8,
  	  EDGE_SCAN = 9

	};*/
	// Finite state machine
	switch(current_state)
	{
	case POSITION_INITIAL: // 1 ----------------------------------------------
	{
		printf("POSITION_INITIAL = %d\n", POSITION_INITIAL);
		// Nothing happens here
	}break;


	case MOVE_TO_START: // 2 ----------------------------------------------
	{
		printf("MOVE_TO_START = %d\n", MOVE_TO_START);
		// Nothing happens here
	}break;


	case POSITION_START: // 3 ----------------------------------------------
	{
		printf("POSITION_START = %d\n", POSITION_START);
		// Nothing happens here
	}break;


	case MOVE_TO_GOAL: // 4 ----------------------------------------------
	{
		printf("MOVE_TO_GOAL = %d\n", MOVE_TO_GOAL);


		// Checking if goal is reached, if not continue to move
		if (is_setpoint_reached_flag)
		{
			printf("Goal is reached\n");
			set_state(POSITION_GOAL , allow_state_change_4);

		}
		// Else, move to goal and check confidence that goal has been reached.
		else // This happens continuously, as long as the state is active. It stops when the flag has been set below.
		{
			// 1. Set setpoint to goal
			// This statement is needed in order to make sure that the setpoint is only set if
			// the current mode is guided mode (otherwise the setpoint might be set before guidance mode
			// is activated and that simply leads to nothing)
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				// Sets setpoint to goal position and orientates drones towards the goal as well
				autopilot_guided_goto_ned(VGOALwned.x, VGOALwned.y, VGOALwned.z, heading_towards_waypoint(WP_GOAL1));
			}

			// If start appears to be reached increase confidence
			if (is_setpoint_reached(&VGOALwned, &VRwned, threshold_distance_to_goal))
			{
				position_confidence++;
				Bound(position_confidence, 0, max_position_confidence);
			}

			// If the position_confidence is high enough, set is_start_reached_flag to 1 and reset position_confidence
			if (position_confidence == max_position_confidence)
			{
				is_setpoint_reached_flag = 1;
				position_confidence = 0;
			}
		}



		// Checking if obstacle is in way, if not continue checking for it
		if (is_obstacle_detected_flag && !is_setpoint_reached_flag)
		{
			printf("Object detected!!!!!!!!\n");
			// 1. Seting setpoint to current location
			guidance_h_hover_enter();
			set_state(WEDGEBUG_START , allow_state_change_4);

		}
		// Else, check confidence that obstacle is there
		else // This happens continuously, as long as the state is active. It stops when the flag has been set below.
		{
			// Calculate median disparity in front if guided mode is active
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				median_disparity_in_front = median_disparity_to_point(&c_img_cropped, &img_depth_int8_cropped, &median_kernel);
			}

			//In case disparity is 0 (infinite distance or error we set it to one disparity
			// above the threshold as the likelyhood that the object is too close is large (as opposed to it being infinitely far away)
			if(median_disparity_in_front == 0 )
			{
				median_disparity_in_front = (threshold_median_disparity + 1);
			}
			printf("median_disparity_in_front = %d\n", median_disparity_in_front);
			// If obstacle appears to be detected, increase confidence
			if ((median_disparity_in_front > threshold_median_disparity) && (float_vect3_norm_two_points(&VGOALwned, &VRwned) > 3)) // NOTE. The second logical operator was added for testing. Delete it after reducing object distance range and integrating the look for edge function
			{
				obstacle_confidence++;
				Bound(obstacle_confidence, 0, max_obstacle_confidence);
			}
			// If the obstacle_confidence is high enough, set is_obstacle_detected_flag to 1 and reset obstacle_confidence
			if (obstacle_confidence == max_obstacle_confidence)
			{
				is_obstacle_detected_flag = 1;
				obstacle_confidence = 0;
			}
		}


		// THis code is to see what depth the disparity value is
		float depth;
		depth = disp_to_depth(median_disparity_in_front, b, f);
		printf("depth function = %f\n", depth);

		printf("position_confidence = %d\n", position_confidence);
		printf("obstacle_confidence = %d\n", obstacle_confidence);


	}break;


	case POSITION_GOAL: // 5 ----------------------------------------------
	{
		printf("POSITION_GOAL = %d\n", POSITION_GOAL);
		// Since the drone is at the goal we will swithc bach to the NAV mode
		if ((autopilot_get_mode() == AP_MODE_GUIDED))
		{
			autopilot_mode_auto2 = AP_MODE_NAV;
			autopilot_static_set_mode(AP_MODE_NAV);
		}
		printf("Total time to reach goal = %f\n", ((double)clock_total_time) / CLOCKS_PER_SEC);
		printf("Total distance_traveled = %f\n", distance_traveled);
		printf("Average runtime of background processes = %f\n", (clock_background_processes / CLOCKS_PER_SEC / counter_cycles));

		for (uint8_t i = 0; i < number_of_states; i++)
		{
			printf("Average runtime of state %d = %f\n", i, (time_state[i] / CLOCKS_PER_SEC / counter_state[i]));
		}




	}break;

	case WEDGEBUG_START: // 6 ----------------------------------------------
	{
		printf("WEDGEBUG_START = %d\n", WEDGEBUG_START);
		// 1. Stopping drone movement (a second time, just to make sure)
		guidance_h_hover_enter();

		// 2. Morphological  operations 2
		// This is needed so that when using the edges as filters (to work on disparity values
		// only found on edges) the underlying disparity values are those of the foreground
		// and not the background
		dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,SE_dilation_OCV_2, 1);

		// 3. Checking if edges are located
		// Declaring (and possible initialization) of sate 6-dependent parameters
		uint8_t was_edge_found = 0; // A variable to store whether a "best" edge was found or not

		// Running function to detect and save edge
		was_edge_found = find_best_edge_coordinates(
				&VEDGECOORDINATESc,
				&VGOALc,//target_point,
				&img_edges_int8_cropped,
				&img_middle_int8_cropped,
				&edge_search_area,
				threshold_disparity_of_edges);
		//printf("was_edge_found?: %d\n", was_edge_found);
		// If edge was found
		if(was_edge_found)
		{
			Vb_to_Va(&VEDGECOORDINATESr, &VEDGECOORDINATESc, &Rcr, &VCr);
			Vb_to_Va(&VEDGECOORDINATESwned, &VEDGECOORDINATESr, &Rrwned, &VRwned);
			Va_to_Vb(&VEDGECOORDINATESwenu, &VEDGECOORDINATESwned, &Rwnedwenu, &VNEDwenu);
			//printf("Optimal edge found\n");
			//printf("Camera coordinates of edge: [%f, %f, %f]\n", VEDGECOORDINATESc.x, VEDGECOORDINATESc.y, VEDGECOORDINATESc.z);
			set_state(MOVE_TO_EDGE , allow_state_change_6);


		}
		// If no edge was found
		else
		{
			printf("Edge not found!!!!!!!!!!!!!!!!!\n");
			set_state(EDGE_SCAN , allow_state_change_6);

		}
	}break;

	case MOVE_TO_EDGE: // 7 ----------------------------------------------
	{
		printf("MOVE_TO_EDGE = %d\n", MOVE_TO_EDGE);

		//printf("Camera coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESc.x, VEDGECOORDINATESc.y, VEDGECOORDINATESc.z);
		//printf("Robot coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESr.x, VEDGECOORDINATESr.y, VEDGECOORDINATESr.z);
		printf("World NED coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESwned.x, VEDGECOORDINATESwned.y, VEDGECOORDINATESwned.z);
		//printf("World ENU coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESwenu.x, VEDGECOORDINATESwenu.y, VEDGECOORDINATESwenu.z);



		// If correct angle is reached, nothing happens
		if (is_heading_reached_flag)
		{
			printf("Heading is reached\n");
		}
		// Else, adjust heading and check confidence that heading is reached
		else // This happens continuously, as long as the state is active. It stops when the flag has been set below
		{
			// Set desired heading (if guided mode is on)
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				guidance_h_set_guided_heading(heading_towards_setpoint_WNED(&VEDGECOORDINATESwned));//   heading_towards_waypoint(WP_GOAL1));
			}
			// If heading appears to be reached increase confidence
			if (is_heading_reached(heading_towards_setpoint_WNED(&VEDGECOORDINATESwned), heading, threshold_distance_to_angle))
			{
				heading_confidence++;
				Bound(heading_confidence, 0, max_heading_confidence);
			}

			// If the heading_confidence is high enough, set is_heading_reached_flag to 1 and reset heading_confidence
			if (heading_confidence == max_heading_confidence)
			{
				is_heading_reached_flag = 1;
				heading_confidence = 0;
			}

		}




		// If edge setpoint is reached, just hover
		if (is_setpoint_reached_flag && is_heading_reached_flag)
		{
			printf("Edge is reached\n");
			//guidance_h_hover_enter();
			autopilot_guided_goto_ned(VEDGECOORDINATESwned.x, VEDGECOORDINATESwned.y, VEDGECOORDINATESwned.z, heading_towards_waypoint(WP_GOAL1));
		}
		// Else, move to edge setpoint and check confidence that edge setpoint is reached
		else if (is_heading_reached_flag) // This happens continuously, as long as the state is active. It stops when the flag has been set below.
		{
			// Set setpoint to edge
			// This statement is needed in order to make sure that the setpoint is only set if
			// the current mode is guided mode (otherwise the setpoint might be set before guidance mode
			// is activated and that simply leads to nothing)
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				// Sets setpoint to goal position and orientates robot towards the goal as well
				//autopilot_guided_goto_ned(VEDGECOORDINATESwned.x, VEDGECOORDINATESwned.y, VEDGECOORDINATESwned.z, heading_towards_setpoint_WNED(&VRwned));
				guidance_h_set_guided_pos(VEDGECOORDINATESwned.x, VEDGECOORDINATESwned.y); guidance_v_set_guided_z(VEDGECOORDINATESwned.z);

			}

			// If edge appears to be reached increase confidence
			if (is_setpoint_reached(&VEDGECOORDINATESwned, &VRwned, threshold_distance_to_goal))// && !is_setpoint_reached_flag)
			{
				position_confidence++;
				Bound(position_confidence, 0, max_position_confidence);
			}

			// If the position_confidence is high enough, set is_setpoint_reached_flag to 1 and reset position_confidence
			if (position_confidence == max_position_confidence)
			{
				is_setpoint_reached_flag = 1;
				position_confidence = 0;
			}
		}






		// If both edge is reached and the robot faces the correct angle, the state is changed to "MOVE_TO_GOAL"
		if (is_heading_reached_flag && is_setpoint_reached_flag)
		{
			printf("Position and Heading are reached\n");
			set_state(POSITION_EDGE , allow_state_change_7);

		}
		printf("position_confidence = %d\n", position_confidence);
		printf("heading_confidence = %d\n", heading_confidence);



	}break;


	case POSITION_EDGE: // 8 ----------------------------------------------
	{

		printf("POSITION_EDGE = %d\n", POSITION_EDGE);

		// THis ensures that the robot stay on edge a all times
		guidance_h_set_guided_pos(VEDGECOORDINATESwned.x, VEDGECOORDINATESwned.y); guidance_v_set_guided_z(VEDGECOORDINATESwned.z);


		// 1. We make the robot face the goal
		// If robot faces goal, robot stay at current edge
		if (is_heading_reached_flag)
		{
			printf("Heading is reached\n");
			guidance_h_set_guided_heading(heading_towards_waypoint(WP_GOAL1));
		}
		// Else, adjust heading and check confidence that heading is reached
		else // This happens continuously, as long as the state is active. It stops when the flag has been set.
		{
			// Set desired heading (if guided mode is on)
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				guidance_h_set_guided_heading(heading_towards_waypoint(WP_GOAL1));
			}
			// If heading appears to be reached increase confidence
			if (is_heading_reached(heading_towards_waypoint(WP_GOAL1), heading, threshold_distance_to_angle))
			{
				heading_confidence++;
				Bound(heading_confidence, 0, max_heading_confidence);
			}

			// If the heading_confidence is high enough, set is_heading_reached_flag to 1 and reset heading_confidence
			if (heading_confidence == max_heading_confidence)
			{
				is_heading_reached_flag = 1;
				heading_confidence = 0;
			}
		}

		// 2. In this loop we check if an obstacle is blocking the path or not. If no obstacle is blocking the path the free_path_confidence is
		// increased, whereas if an obstacle is blocking the path the obstacle_confidence is increased. If one of the confidences reaches the
		// maximum the respective flag is turned on. These statements only execute if the robot faces the goal (see above)
		// If obstacle is in way AND heading is reached
		if (is_obstacle_detected_flag && is_heading_reached_flag)
		{
			printf("Object detected!!!!!!!!\n");
			set_state(WEDGEBUG_START, allow_state_change_7);
		}
		// Else if the path is clear AND heading is reached
		else if (is_path_free_flag && is_heading_reached_flag)
		{
			printf("Path is free\n");
			set_state(MOVE_TO_GOAL, allow_state_change_7);
		}
		// Else (if heading is reached) check for obstacles and free path and check confidence that an obstacle is there or that the path is free
		else if(is_heading_reached_flag) // This only runs if the robot faces the goal and neither an obstacle or a free path has been found
			// Calculate median disparity in front if guided mode is active
			{
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				median_disparity_in_front = median_disparity_to_point(&c_img_cropped, &img_depth_int8_cropped, &median_kernel);
			}

			//In case disparity is 0 (infinite distance or error we set it to one disparity
			// above the threshold as the likelihood that the object is too close is large (as opposed to it being infinitely far away)
			if(median_disparity_in_front == 0 )
			{
				median_disparity_in_front = (threshold_median_disparity + 1);
			}
			printf("median_disparity_in_front = %d\n", median_disparity_in_front);

			// This if statement increase the confidence
			// If obstacle is detected, increase obstacle_confidence
			if ((median_disparity_in_front > threshold_median_disparity)) // NOTE. The second logical operator was added for testing. Delete it after reducing object distance range and integrating the look for edge function
			{
				obstacle_confidence++;
				Bound(obstacle_confidence, 0, max_obstacle_confidence);
			}
			// If obstacle is not detected, increase free_path_confidence
			else
			{
				free_path_confidence++;
				Bound(free_path_confidence, 0, max_free_path_confidence);
			}

			// If the obstacle_confidence is high enough, set is_obstacle_detected_flag to 1 and reset obstacle_confidence and free_path_confidence
			if (obstacle_confidence == max_obstacle_confidence)
			{
				is_obstacle_detected_flag = 1;
				obstacle_confidence = 0;
				free_path_confidence = 0;
			}
			// If the free_path_confidence is high enough, set is_path_free_flag to 1 and reset free_path_confidence and obstacle_confidence
			if (free_path_confidence == max_obstacle_confidence)
			{
				is_path_free_flag = 1;
				free_path_confidence = 0;
				obstacle_confidence = 0;
			}
			}
		printf("Obstacle confidence = %d\n", obstacle_confidence);
		printf("Free path confidence = %d\n", free_path_confidence);
	}break;

	case EDGE_SCAN: // 9 ----------------------------------------------
	{
		printf("EDGE_SCAN = %d\n", EDGE_SCAN);

		//guidance_h_set_guided_heading_rate(0.0349066);


		// Making drone hover, so that it does not drift from its current position
		guidance_h_hover_enter();


		// Continuous scanning of edges (until this state is left)
		// Morphological  operations 2 - To ensure maximum disparity is underneath edge
		dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped,SE_dilation_OCV_2, 1);

		// 3. Checking if edges are located
		// Running function to detect and save edge
		is_best_edge_found_flag = find_best_edge_coordinates(
				&VEDGECOORDINATESc,
				&VGOALc,//target_point,
				&img_edges_int8_cropped,
				&img_middle_int8_cropped,
				&edge_search_area,
				threshold_disparity_of_edges);


		// If edge was found
		if(is_best_edge_found_flag)
		{
			Vb_to_Va(&VEDGECOORDINATESr, &VEDGECOORDINATESc, &Rcr, &VCr);
			Vb_to_Va(&VEDGECOORDINATESwned, &VEDGECOORDINATESr, &Rrwned, &VRwned);
			Va_to_Vb(&VEDGECOORDINATESwenu, &VEDGECOORDINATESwned, &Rwnedwenu, &VNEDwenu);
			edge_found_confidence++;
			//Bound(edge_found_confidence, 0, max_heading_confidence);
		}


		// Initializing current positional parameters (if they have not been initialized before)
		if (initial_state.initiated == 0)
		{
			initial_state.heading_initial = heading;
			initial_state.heading_max_left = (heading - max_edge_search_angle) + (WEDGEBUG_HFOV / 2); // this way, when the the center of the drone is facing left, its fov does not exceed the maximum angle to the left
			FLOAT_ANGLE_NORMALIZE(initial_state.heading_max_left);
			initial_state.heading_max_right = (heading + max_edge_search_angle) - (WEDGEBUG_HFOV / 2);  // this way, when the the center of the drone is facing right, its fov does not exceed the maximum angle to the right
			FLOAT_ANGLE_NORMALIZE(initial_state.heading_max_right);
			initial_state.initiated = 1;
		}


		// Code for looking left
		// If left heading has been reached (i.e. if the flag is activated)
		if (initial_state.is_left_reached_flag)
		{
			printf("Left heading is reached\n");
		}
		// Else,  adjust angle to to face more left and check confidence that left heading has been reached
		else // This happens continuously, as long as the state is active. It stops when the flag has been set below
		{
			// Set heading to maximum left heading (if guided mode is on)
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				guidance_h_set_guided_heading(initial_state.heading_max_left);
			}
			// If heading appears to be reached increase confidence
			if (is_heading_reached(initial_state.heading_max_left, heading, threshold_distance_to_angle))
			{
				heading_confidence++;
				Bound(heading_confidence, 0, max_heading_confidence);
			}
			// If the heading_confidence is high enough, set initial_state.is_left_reached_flag to 1 and reset heading_confidence
			if (heading_confidence == max_heading_confidence)
			{
				initial_state.is_left_reached_flag = 1;
				heading_confidence = 0;
			}
		}



		// Code for looking right - Only runs if robot has previously looked left (initial_state.heading_max_left = 1)
		// If right heading has been reached (i.e. if the flag is activated)
		if (initial_state.is_right_reached_flag)
		{
			printf("Right heading is reached\n");
		}
		// Else,  adjust angle to to face more right and check confidence that right heading has been reached
		else if(initial_state.is_left_reached_flag)
		{
			// Set heading to maximum left heading (if guided mode is on)
			if ((autopilot_get_mode() == AP_MODE_GUIDED))
			{
				guidance_h_set_guided_heading(initial_state.heading_max_right);
			}
			// If heading appears to be reached increase confidence
			if (is_heading_reached(initial_state.heading_max_right, heading, threshold_distance_to_angle))
			{
				heading_confidence++;
				Bound(heading_confidence, 0, max_heading_confidence);
			}
			// If the heading_confidence is high enough, set initial_state.is_right_reached_flag to 1 and resets heading_confidence
			if (heading_confidence == max_heading_confidence)
			{
				initial_state.is_right_reached_flag = 1;
				heading_confidence = 0;
			}
		}


		// Code for leaving state - Only runs if robot has previously looked left and then right
		// If the robot has looked left and right and found an edge, change state to "MOVE_TO_EDGE"
		if (initial_state.is_left_reached_flag && initial_state.is_right_reached_flag)
		{
			printf("Scan completed\n");

			// Setting is_edge_found_flag
			if (edge_found_confidence > 0) 	{is_best_edge_found_flag = 1;}
			else 							{is_best_edge_found_flag = 0;}

			// If an edge has been found during the scan
			if(is_best_edge_found_flag)
			{
				printf("Edge has been found\n");
				set_state(MOVE_TO_EDGE, allow_state_change_9);

			}
			// If no edge has been found during the scan
			else
			{
				printf("Minimum has been encountered\n");
			}
		}else
		{
			printf("Scan not complete\n");
		}
		printf("heading_confidence = %d\n", heading_confidence);
		printf("edge_found_confidence = %d\n", edge_found_confidence);
	}break;

    default: // 0 ----------------------------------------------
    {
    	printf("default = %d\n", 0);
    }
    break;

	}
	// ############ Metric 4 - Runtime average per state - End:
	clock_FSM = clock() - clock_FSM; // Calculating time it took for the FSM to finish running
    time_state[current_state] += ((double)clock_FSM); // Adding calculated time to total time of current state



	printf("Time elapsed since start = %f\n", ((double)clock_total_time) / CLOCKS_PER_SEC);
	printf("distance_traveled = %f\n", distance_traveled);

	/*
    // printing flags
	printf("is_start_reached_flag = %d\n", is_start_reached_flag);
	printf("is_setpoint_reached_flag = %d\n", is_setpoint_reached_flag);
	printf("is_obstacle_detected_flag = %d\n", is_obstacle_detected_flag);
	printf("is_path_free_flag = %d\n", is_path_free_flag);
	printf("is_heading_reached_flag = %d\n", is_heading_reached_flag);
	printf("is_best_edge_found_flag = %d\n", is_best_edge_found_flag);
	printf("is_state_changed_flag = %d\n", is_state_changed_flag);
	printf("initial_state.initiated = %d\n", initial_state.initiated);
	printf("initial_state.is_left_reached_flag = %d\n", initial_state.is_left_reached_flag);
	printf("initial_state.is_right_reached_flag = %d\n", initial_state.is_right_reached_flag);

	*/
    printf("\n");






	save_image_gray(&img_left_int8, "/home/dureade/Documents/paparazzi_images/img_left_int8.bmp");
	save_image_gray(&img_right_int8, "/home/dureade/Documents/paparazzi_images/img_right_int8.bmp");
	save_image_gray(&img_depth_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_depth_int8_cropped.bmp");
	save_image_gray(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_middle_int8_cropped.bmp");
	save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped.bmp");


}



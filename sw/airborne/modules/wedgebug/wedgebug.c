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
 *2. When starting paparazzi and joystick is selected, the autopilot will start in the mode AP_MODE_ATTITUDE_DIRECT   4 (but engines are off)
 *3. When pressing circle on the dual shock 4 controller you activate the manual control mode AP_MODE_ATTITUDE_Z_HOLD   9
 *4. When pressing x on the dual shock 4 controller you activate the manual control mode AP_MODE_ATTITUDE_DIRECT   4
 */

/*
 *Note. Most variables are initialized in the initialization function. Variables
 *marked with an "!" were taken into account when calculating the total memory consumption
 */


// New section: Importing headers ----------------------------------------------------------------------------------------------------------------

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


// New section --------------------------------------------------------------------------------------------------------------------------------------
// Defines
#ifndef WEDGEBUG_CAMERA_RIGHT_FPS
#define WEDGEBUG_CAMERA_RIGHT_FPS 0 //< Default FPS (zero means run at camera fps)
#endif
#ifndef WEDGEBUG_CAMERA_LEFT_FPS
#define WEDGEBUG_CAMERA_LEFT_FPS 0 //< Default FPS (zero means run at camera fps)
#endif



// New section ----------------------------------------------------------------------------------------------------------------
// Define global variables

// Declaring images
struct image_t img_left;        //! Image obtained from left camera (UYVY format)
struct image_t img_right;       //! Image obtained from right camera (UYVY format)
struct image_t img_left_int8;     //! Image obtained from left camera, converted into 8bit gray image
struct image_t img_left_int8_cropped; // Image obtained from left camera, converted into 8bit gray image
struct image_t img_right_int8;      //! Image obtained from right camera, converted into 8bit gray image
struct image_t img_disparity_int8_cropped;  //! Image obtained after simple block matching
struct image_t img_depth_int16_cropped; //! Image holding depth values (cm) obtained from the disparity image
struct image_t img_middle_int8_cropped; // Image obtained after processing (morphological operations) of previous image
struct image_t img_edges_int8_cropped;  //! Image obtained from the external sobel edge detection function = sobel_OCV

struct image_t img_disparity_int16_cropped;
struct image_t img_middle_int16_cropped;

// New section: Global variables - Declarations ----------------------------------------------------------------------------------------------------------------



// Declaring crop_t structure for information about the cropped image (after BM)
struct crop_t img_cropped_info; //!

// Declaring dimensions of images and kernels used
struct img_size_t img_dims;       //! Dimensions of images captured by the camera (left and right have same dimension)
struct img_size_t
  img_cropped_dims;   //! Dimension of image after it has been cropped to remove pixel error due to block matching limitations
struct img_size_t
  kernel_median_dims; //! Dimensions of the kernel that detect median disparity in front of drone (for obstacle detection)

// Declaring empty kernel for obtaining median
struct kernel_C1 median_kernel; // !
struct kernel_C1 median_kernel16bit; // !

// Delcaring structuring element sizes
int SE_opening_OCV;   //! SE size for the opening operation
int SE_closing_OCV;   //! SE size for the closing operation
int SE_dilation_OCV_1;  //! SE size for the first dilation operation
int SE_dilation_OCV_2;  //! SE size for the second dilation operation (see state 3 "WEDGEBUG_START" and state 6 "POSITION_EDGE" )
int SE_erosion_OCV;   //! SE size for the erosion operation (see state 3 "WEDGEBUG_START" and state 6 "POSITION_EDGE", its needed to "drag" the depth of the foreground objects over the edges detected)
int SE_sobel_OCV;     //! SE size for the Sobel operation, to detect edges
uint16_t K_median_w;  //! Width of kernel for the median kernel
uint16_t K_median_h;  //! Height of kernel for the median kernel


// Declaring vectors to hold global 3d points
struct FloatVect3
  VSTARTwenu;         //! Declared vector of coordinates of start position in ENU world coordinate system
struct FloatVect3
  VSTARTwned;         //! Declared vector of coordinates of start position in NED world coordinate system
struct FloatVect3 VGOALwenu;        //! Declared vector of coordinates of goal in ENU world coordinate system
struct FloatVect3 VGOALwned;        //! Declared vector of coordinates of goal in NED world coordinate system
struct FloatVect3 VGOALr;           //! Declared vector of coordinates of goal in robot coordinate system
struct FloatVect3 VGOALc;           //! Declared vector of coordinates of goal in camera coordinate system
struct FloatVect3
  VEDGECOORDINATESc;    //! Declared vector of coordinates of "best" edge detected in camera coordinate system
struct FloatVect3
  VEDGECOORDINATESr;    //! Declared vector of coordinates of "best" edge detected in robot coordinate system
struct FloatVect3
  VEDGECOORDINATESwned;   //! Declared vector of coordinates of "best" edge detected in NED world coordinate system
struct FloatVect3
  VEDGECOORDINATESwenu;   //! Declared vector of coordinates of "best" edge detected in ENU world coordinate system
struct FloatVect3
  VDISTANCEPOSITIONwned;  //! Declared a vector to hold the current position, which is needed for calculating the distance traveled
struct FloatVect3
  VHOLDINGPOINTwned;    //! Declared a vector to hold the position of a holding point (offten used to make sure drone stays still before stuff happens)
struct FloatVect3
  VPBESTEDGECOORDINATESwned;//! Declared vector of coordinates of previous "best" edge detected in NED world coordinate system


// Declaring thresholds
int threshold_edge_magnitude;     //! Edges with a magnitude above this value are detected. Above this value, edges are given the value 127, otherwise they are given the value zero.
uint8_t threshold_median_disparity;   //! Above this median disparity, an obstacle is considered to block the way (i.e. the blocking obstacle need to be close)
uint8_t threshold_disparity_of_edges;   //! Above this disparity edges are eligible for WedgeBug algorithm (i.e. edges cannot be very far away)
float threshold_distance_to_goal;     //! Below this distance (in meters) it is considered that the robot has reached the goal
float threshold_distance_to_angle;    //! Below this distance (in radians) it is considered that the robot has reached the target angle
float threshold_distance_to_goal_direct;//! Below this distance (in meters) it is considered that the robot has reached the goal, in DIRECT_CONTROL mode

uint16_t threshold_median_depth;      //! Below this median disparity, an obstacle is considered to block the way (i.e. the blocking obstacle need to be close)
uint16_t threshold_depth_of_edges;    //! Below this depth (m) edges are eligible for the WedgeBug algorithm (i.e. edges cannot be very far away)

// Declaring confidence parameters
int16_t obstacle_confidence;    //! This is the confidence that an obstacle was spotted
int16_t free_path_confidence;   //! This is the confidence that no obstacle was spotted
int16_t position_confidence;    //! This is the confidence that the desired position was reached
int16_t heading_confidence;     //! This is the confidence that the desired heading is reached

//int16_t edge_found_micro_confidence;    //! This is the confidence that an edge was found - inside of the find_best_edge_coordinates function
int16_t edge_found_macro_confidence;    //! This is the confidence that an edge was found - outside of the find_best_edge_coordinates function
int16_t no_edge_found_confidence;     //! This is the confidence that no edge was found
int16_t max_obstacle_confidence;      //! This is the max confidence that an obstacle was spotted
int16_t max_free_path_confidence;     //! This is the max confidence that an obstacle was not spotted
int16_t max_position_confidence;      //! This is the max confidence that a specific position was reached
int16_t max_heading_confidence;       //! This is the max confidence that a specific heading was reached
int16_t max_edge_found_micro_confidence;  //! This is the max confidence that edges (micro-see above) were found
int16_t max_edge_found_macro_confidence;  //! This is the max confidence that edges (macro-see above were found
int16_t max_no_edge_found_confidence;   //! This is the max confidence that no edges were found

// Declaring boolean flags
uint8_t is_start_reached_flag;    // Set to 1 if start position is reached, 0 otherwise.
uint8_t is_setpoint_reached_flag; //! Set to 1 if setpoint is reached, 0 otherwise.
uint8_t is_obstacle_detected_flag;  //! Set to 1 if obstacle is detected, 0 otherwise.
uint8_t is_path_free_flag;      //! Set to 1 if no obstacle is detected, 0 otherwise.
uint8_t is_heading_reached_flag;  //! Set to 1 if heading is reached, 0 otherwise.
uint8_t is_edge_found_macro_flag; //! Set to 1 if best edge (according to macro confidence) was found, 0 otherwise
uint8_t is_edge_found_micro_flag;   //! Set to 1 if best edge (according to micro confidence) was found, 0 otherwise
uint8_t is_no_edge_found_flag;    //! Set to 1 if no edge was identified, 0 otherwise
uint8_t is_state_changed_flag;    //! Set to 1 if state was changed, 0 otherwise
uint8_t is_mode_changed_flag;   //! Set to 1 if control mode of drone is changed, 0 otherwise
uint8_t save_images_flag;       // For report: Flag to indicate if images should be saved

// Declaring principal points
struct point_t c_img;       //! Principal point of normal camera images
struct point_t c_img_cropped;   //! Principal point of cropped camera images

// Declaring edge search area
struct crop_t edge_search_area; //! This structure holds information about the window in which edges are searched in


// Declaring rotation matrices and transition vectors for frame to frame transformations
// 1) Rotation matrix and transition vector to transform from world ENU frame to world NED frame
struct FloatRMat Rwnedwenu; //!
struct FloatVect3 VNEDwenu; //!
// 2) Rotation matrix and transition vector to transform from world ned frame to robot frame
struct FloatRMat Rrwned;  //!
struct FloatVect3 VRwned;   //!
// 3) Rotation matrix and transition vector to transform from robot frame to camera frame
struct FloatRMat Rcr;   //!
struct FloatVect3 VCr;    //!

// Declaration and initialization of camera parameters
float b = WEDGEBUG_CAMERA_BASELINE /
          1000.00; //! Camera baseline, in meters (i.e. horizontal distance between the two cameras of the stereo setup)
uint16_t f = WEDGEBUG_CAMERA_FOCAL_LENGTH;    //! Camera focal length, in pixels (i.e. distance between camera



// Define new structures + enums
enum navigation_state {
  MOVE_TO_GOAL = 1,
  POSITION_GOAL = 2,
  WEDGEBUG_START = 3,
  MOVE_TO_EDGE = 4,
  POSITION_EDGE = 5,
  EDGE_SCAN = 6
};
enum navigation_state current_state ;// Default state is 0 i.e. nothing


enum control_mode_state {
  DIRECT_CONTROL = 1,
  AUTONOMOUS_NAV = 2,
  AUTONOMOUS_GUIDED = 3
};

enum control_mode_state current_mode;// Default state is 0 i.e. nothing



// This is a structure to save angles that are needed for the "Edge Scan" state
struct ES_angles {
  float heading_initial;  // This is the initial heading of the robot when the "Edge Scan" state is entered
  float heading_max_left; // This is the maximum left turn angle (determines how much to the left the robot can look to search for edges), to search for edges
  float heading_max_right;// This is the maximum right turn angle (determines how much to the right the robot can look to search for edges), to search for edges
  uint8_t initiated;    // This is a flag that can be set to check whether the structure is allowed to be overwritten (0=allowed, 1=forbidden)
  uint8_t is_left_reached_flag; // This is a flag to check whether the left was scanned for an edge already
  uint8_t is_right_reached_flag;  // This is a flag to check whether the right was scanned for an edge already
};
struct ES_angles initial_heading; // !


// Declaring variables for time measurement
double time_state[NUMBER_OF_STATES];   // Double array for saving total time (clock cycles) spent in the states (position 0 = state 0 and so on)
double counter_state[NUMBER_OF_STATES]; // A counter to measure the total cycles that each state in the FSM (within the periodic function) went through
double counter_cycles;          // A counter to measure the total cycles that the periodic function went through
clock_t clock_total_time;         // Clock to measure total time (clock cycles)) it took for the robot to fly from start to goal
clock_t clock_total_time_current;     // Clock to hold time measured at start of the current cycle of the periodic function
clock_t clock_total_time_previous;    // Clock to hold time measured at start of the previous cycle of the periodic function
clock_t clock_background_processes;
clock_t clock_FSM;

// Other declarations
uint8_t previous_state;         //! Variable that saves previous state the state machine was in, for some memory
uint8_t previous_mode;          //! Variable that saves previous mode to control the drone, for some memory
int N_disparities = 64;         //! Number of disparity levels (0-this number)
int block_size_disparities = 25;    //! Block size used for the block matching (SBM) function
int min_disparity = 0;//
float heading;              //! Variable for storing the heading of the drone (psi in radians)
float max_edge_search_angle = M_PI /
                              2; //! The maximum angle (in adians) to the left and right of the drone, that edges can be detected in. Edges outside of this area are considered to be in a minimum
uint8_t median_disparity_in_front;    //! Variable to hold the median disparity in front of the drone. Needed to see if obstacle is there.
uint16_t median_depth_in_front;     //! Variable to hold the median depth in front of the drone. Needed to see if obstacle is there
float distance_traveled;        // Variable to hold the distance traveled of the robot (since start and up to the goal)
uint8_t number_of_states;       // Variable to save the total number of states used in the finite state machine
float distance_robot_edge_goal;     //! Variable to hold distance from robot to edge to goal (used in EDGE_SCAN (9) state)
int heat_map_type;


// For debugging purpose. Allows for changing of state in simulation if 1. If 0, does not allow for state change. Useful if you want to execute only one state repeatedly

uint8_t allow_state_change_MOVE_TO_GOAL; // From within state "MOVE_TO_GOAL"
uint8_t allow_state_change_POSITION_GOAL; // From within state "POSITION_GOAL"
uint8_t allow_state_change_WEDGEBUG_START; // From within state "WEDGEBUG_START"
uint8_t allow_state_change_MOVE_TO_EDGE; // From within state "MOVE_TO_EDGE"
uint8_t allow_state_change_POSITION_EDGE; // From within state "POSITION_EDGE"
uint8_t allow_state_change_EDGE_SCAN; // From within state "EDGE_SCAN"


uint8_t is_total_timer_on_flag;
float threshold_distance_to_goal_manual;


// New section: Functions - Declaration ----------------------------------------------------------------------------------------------------------------

// Supporting
const char *get_img_type(enum image_type img_type); // Function 1: Displays image type
void show_image_data(struct image_t *img); // Function 2: Displays image data
void show_image_entry(struct image_t *img, int entry_position,
                      const char *img_name); // Function 3: Displays pixel value of image
void show_rotation_matrix(struct FloatRMat *R);

// External
void post_disparity_crop_rect(struct crop_t *img_cropped_info, struct img_size_t *original_img_dims, const int disp_n,
                              const int block_size);
void set_state(uint8_t state, uint8_t change_allowed);
void kernel_create(struct kernel_C1 *kernel, uint16_t width, uint16_t height, enum image_type type);
void kernel_free(struct kernel_C1 *kernel);
uint8_t getMedian(uint8_t *a, uint32_t n);

//Core
static struct image_t *copy_left_img_func(struct image_t
    *img); // Function 1: Copies left image into a buffer (buf_left)
static struct image_t *copy_right_img_func(struct image_t
    *img); // Function 2: Copies left image into a buffer (buf_right)
void UYVYs_interlacing_V(struct image_t *YY, struct image_t *left,
                         struct image_t *right); // Function 3: Copies gray pixel values of left and right UYVY images into merged YY image
void UYVYs_interlacing_H(struct image_t *merged, struct image_t *left, struct image_t *right);

uint32_t maximum_intensity(struct image_t *img);
void thresholding_img(struct image_t *img, uint8_t threshold);
void principal_points(struct point_t *c_output, const struct point_t *c_old_input, struct crop_t *img_cropped_info);
float disp_to_depth(const uint8_t d, const float b, const uint16_t f);
uint8_t depth_to_disp(const float depth, const float b, const uint16_t f);
void Vi_to_Vc(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x, const uint8_t d,
              const float b, const uint16_t f);
int32_t indx1d_a(const int32_t y, const int32_t x, const struct image_t *img);
int32_t indx1d_b(const int32_t y, const int32_t x, const struct img_size_t *img_dims);
int32_t indx1d_c(const int32_t y, const int32_t x, const uint16_t img_height, const uint16_t img_width);

void Va_to_Vb(struct FloatVect3 *Vb, struct FloatVect3 *Va, struct FloatRMat *Rba, struct FloatVect3 *VOa);
void Vb_to_Va(struct FloatVect3 *Va, struct FloatVect3 *Vb, struct FloatRMat *Rba, struct FloatVect3 *VOa);
void Vw_to_Vc(struct FloatVect3 *Vc, struct FloatVect3 *Vw, struct FloatRMat *Rrw, struct FloatVect3 *VRw,
              struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose);
void Vc_to_Vw(struct FloatVect3 *Vw, struct FloatVect3 *Vc, struct FloatRMat *Rrw, struct FloatVect3 *VRw,
              struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose);

float float_vect3_norm_two_points(struct FloatVect3 *V1, struct FloatVect3 *V2);
float heading_towards_waypoint(uint8_t wp);
float heading_towards_setpoint_WNED(struct FloatVect3 *VSETPOINTwned);
uint8_t median_disparity_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median);

uint8_t find_best_edge_coordinates(struct FloatVect3 *VEDGECOORDINATESc, struct FloatVect3 *VTARGETc,
                                   struct image_t *img_edges, struct image_t *img_disparity, struct crop_t *edge_search_area, uint8_t threshold,
                                   int16_t max_confidence);
uint8_t is_setpoint_reached(struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold);

float float_norm_two_angles(float target_angle, float current_angle);
uint8_t is_heading_reached(float target_angle, float current_angle, float threshold);
uint8_t are_setpoint_and_angle_reached(struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION,
                                       float threshold_setpoint, float target_angle, float current_angle, float threshold_angle);
void disp_to_depth_img(struct image_t *img8bit_input, struct image_t *img16bit_output);
void background_processes(uint8_t save_images_flag);

uint16_t getMedian16bit(uint16_t *a, uint32_t n);
float dispfixed_to_disp(const int16_t d);
float disp_to_depth_16bit(const int16_t d, const float b, const uint16_t f);
void Vi_to_Vc_depth(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x,
                    const float depth /*depth in m*/, const uint16_t f);
void Vi_to_Vc16bit(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x, const uint16_t d,
                   const float b, const uint16_t f);
uint16_t median_depth_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median);
uint8_t find_best_edge_coordinates2(struct FloatVect3 *VEDGECOORDINATESc, struct FloatVect3 *VTARGETc,
                                    struct image_t *img_edges, struct image_t *img_depth, struct crop_t *edge_search_area, uint16_t threshold,
                                    int16_t max_confidence);
void background_processes_16bit(uint8_t save_images_flag);




// New section: Functions - Definition ----------------------------------------------------------------------------------------------------------------

// Supporting:

// Function 1
const char *get_img_type(enum image_type img_type)
{
  switch (img_type) {
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
  printf("Pixel %d value - %s: %d\n", entry_position, img_name, ((uint8_t *)img->buf)[entry_position]);
}

// Function 4
void show_rotation_matrix(struct FloatRMat *R)
{
  printf("[[%f, %f, %f]\n", R->m[0], R->m[1], R->m[2]);
  printf(" [%f, %f, %f]\n", R->m[3], R->m[4], R->m[5]);
  printf(" [%f, %f, %f]]\n", R->m[6], R->m[7], R->m[8]);

}



// External:

// Function 1 - Returns the upper left coordinates of a square (x and y coordinates) and the offset in terms of width and height,
// given the number of disparity levels and the block size used by the block matching algorithm. This is need to crop an image
void post_disparity_crop_rect(struct crop_t *img_cropped_info, struct img_size_t *original_img_dims, const int disp_n,
                              const int block_size)
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
  if (change_allowed == 1) {current_state = state;}
}


// Function 3 - Creates empty 8bit kernel
void kernel_create(struct kernel_C1 *kernel, uint16_t width, uint16_t height, enum image_type type)
{

  // Set the variables
  kernel->type = type;
  kernel->h = height;
  kernel->w = width;


  // Depending on the type the size differs
  if (type == IMAGE_YUV422) {
    kernel->buf_size = sizeof(uint8_t) * 2 * width * height;
  } else if (type == IMAGE_JPEG) {
    kernel->buf_size = sizeof(uint8_t) * 2 * width * height;  // At maximum quality this is enough
  } else if (type == IMAGE_GRADIENT) {
    kernel->buf_size = sizeof(int16_t) * width * height;
  } else if (type == IMAGE_INT16) {
    kernel->buf_size = sizeof(int16_t) * width * height;
  } else {
    kernel->buf_size = sizeof(uint8_t) * width * height;
  }

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


// Function 6 - Calculates median of a 16bit image
uint16_t getMedian16bit(uint16_t *a, uint32_t n)
{
  // Allocate an array of the same size and sort it.
  uint32_t i, j;

  uint16_t dpSorted[n];
  for (i = 0; i < n; ++i) {
    dpSorted[i] = a[i];
  }
  for (i = n - 1; i > 0; --i) {
    for (j = 0; j < i; ++j) {
      if (dpSorted[j] > dpSorted[j + 1]) {
        uint16_t dTemp = dpSorted[j];
        dpSorted[j] = dpSorted[j + 1];
        dpSorted[j + 1] = dTemp;
      }
    }
  }

  // Middle or average of middle values in the sorted array.
  uint16_t dMedian = 0;
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
  if (left->w != right->w || left->h != right->h) {
    printf("The dimensions of the left and right image to not match!");
    return;
  }
  if ((merged->w * merged->h) != (2 * right->w) * right->w) {
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


  for (uint32_t i = 0; i < loop_length; i++) {
    *YY = *UYVY_left; // Copies left gray pixel (Y) to the merged image YY, in first position
    YY++; // Moving to second position of merged image YY
    *YY = *UYVY_right; // Copies right gray pixel (Y) to the merged image YY, in second position
    YY++; // Moving to the next position, in preparation to copy left gray pixel (Y) to the merged image YY
    UYVY_left += 2; // Moving pointer to next gray pixel (Y), in the left image
    UYVY_right += 2; // Moving pointer to next gray pixel (Y), in the right image
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
  if (left->w != right->w || left->h != right->h) {
    printf("The dimensions of the left and right image to not match!");
    return;
  }
  if ((merged->w * merged->h) != (2 * right->w) * right->w) {
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

  for (uint32_t i = 0; i < left->h; i++) {
    //printf("Loop 1: %d\n", i);
    for (uint32_t j = 0; j < left->w; j++) {
      //printf("Loop 1: %d\n", j);
      *YY1 = *UYVY_left;
      *YY2 = *UYVY_right;
      YY1++;
      YY2++;
      UYVY_left += 2;
      UYVY_right += 2;
    }
    YY1 += merged->w; // Jumping pointer to second next row (i.e. over row with pixels from right image)
    YY2 += merged->w; // Jumping pointer to second next row (i.e. over row with pixels from left image)
  }
}


// Function 5 - Returns the maximum value in a uint8_t image
uint32_t maximum_intensity(struct image_t *img)
{
  if (img->type == IMAGE_GRAYSCALE) {
    uint32_t max = 0;
    for (uint32_t i = 0; i < img->buf_size; i++) {
      uint8_t *intensity = &((uint8_t *)img->buf)[i];

      if (*intensity > max) {
        max = *intensity;
      }
    }
    return max;
  } else if (img->type == IMAGE_INT16) {
    uint32_t max = 0;
    for (uint32_t i = 0; i < (img->w * img->h); i++) {
      uint16_t *intensity = &((uint16_t *)img->buf)[i];

      if (*intensity > max) {
        max = *intensity;
        printf("%d\n", *intensity);
        printf("location = %d\n", i);
      }
    }
    return max;
  } else {
    printf("ERROR: function does not support image type %d. Breaking out of function.", img->type);
    return 1;
  }


}


// Function 6 - Thresholds 8bit images given and turns all values >= threshold to 255
void thresholding_img(struct image_t *img, uint8_t threshold)
{
  for (uint32_t i = 0; i < img->buf_size; i++) {
    uint8_t *intensity = &((uint8_t *)img->buf)[i];

    if (*intensity >= threshold) {
      *intensity = 127;
    } else {
      *intensity = 0;
    }
  }
}


// Function 7 - Calculates principal point coordinates for a cropped image, based on the x
// and y coordinates of the cropped area (upper left-hand side: crop_y and crop_x).
void principal_points(struct point_t *c_output, const struct point_t *c_old_input, struct crop_t *img_cropped_info)
{
  c_output->y = c_old_input->y - img_cropped_info->y;
  c_output->x = c_old_input->x - img_cropped_info->x;
}


// Function 8a - Converts disparity to depth using focal length (in pixels) and baseline distance (in meters)
float disp_to_depth(const uint8_t d, const float b, const uint16_t f)
{
  return b * f / d;
}


// Function 8c - Converts 16bit fixed number disparity (pixels * 16) to 16bit disparity (pixels)
float dispfixed_to_disp(const int16_t d)
{
  return (d / 16.00);
}


// Function 8d - Converts 16bit fixed number disparity (pixels * 16) to 16bit depth (meters) using focal length (in pixels) and baseline distance (in meters)
float disp_to_depth_16bit(const int16_t d, const float b, const uint16_t f)
{
  return b * f / dispfixed_to_disp(d);
}


// Function 8b - Converts depth to disparity using focal length (in pixels) and baseline distance (in meters)
uint8_t depth_to_disp(const float depth, const float b, const uint16_t f)
{
  return round(b * f / depth);
}




// Function 9a - Calculates 3d points in a scene based on the 2d coordinates of the point in the
// image plane and the depth. d in in pixels, b is in meters and f is in pixels
void Vi_to_Vc(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x, const uint8_t d,
              const float b, const uint16_t f)
{
  // Calculating Z
  // In case disparity is 0 Z will be very very small to avoid detection of algorithm that
  // calculates closest edge point to target goal
  //printf("y=%d\n", image_point_y);
  //printf("x=%d\n", image_point_x);
  //printf("d=%d\n", d);


  if (d == 0) {
    scene_point->z = 0.0001;
  } else {
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


// Function 9b - Calculates 3d points in a scene based on the 2d coordinates of the point in the
// image plane and the depth. d in in pixels, b is in meters and f is in pixels
void Vi_to_Vc_depth(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x,
                    const float depth /*depth in m*/, const uint16_t f)
{
  // Calculating Z
  scene_point->z = depth;

  // Calculating Y
  scene_point->y = image_point_y * scene_point -> z / f;

  // Calculating X
  scene_point->x = image_point_x * scene_point -> z / f;
  //printf("Y (y=%d) =  %f\n", image_point->y, scene_point->Y);
  //printf("X (x=%d) =  %f\n", image_point->x, scene_point->X);
  //printf("Z (d=%d) =  %f\n", d, scene_point->Z);
}


// Function 9c - Calculates 3d points in a scene based on the 2d coordinates of the point in the
// image plane and the depth. d in in pixels, b is in meters and f is in pixels
void Vi_to_Vc16bit(struct FloatVect3 *scene_point, int32_t image_point_y, int32_t image_point_x, const uint16_t d,
                   const float b, const uint16_t f)
{
  // Calculating Z
  // In case disparity is 0 Z will be very very small to avoid detection of algorithm that
  // calculates closest edge point to target goal
  //printf("y=%d\n", image_point_y);
  //printf("x=%d\n", image_point_x);
  //printf("d=%d\n", d);


  if (d == 0) {
    scene_point->z = 0.0001;
  } else {
    scene_point->z = disp_to_depth_16bit(d, b, f);
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
  if (x >= (img->w) || x < 0) {
    printf("Error: index x=%d is out of bounds for axis 0 with size %d. Returning -1\n", x, img->w);
    return -1;
  } else if (y >= (img->h) || y < 0) {
    printf("Error: index y=%d is out of bounds for axis 0 with size %d. Returning -1\n", y, img->h);
    return -1;
  } else {
    return x + img->w * y;
  }
}


// Function 10b - Converts 2d coordinates into 1d coordinates (for 1d arrays) - using struct img_size_t for dimensions
int32_t indx1d_b(const int32_t y, const int32_t x, const struct img_size_t *img_dims)
{
  if (x >= (img_dims->w) || x < 0) {
    printf("Error: index %d is out of bounds for axis 0 with size %d. Returning -1\n", x, img_dims->w);
    return -1;
  } else if (y >= (img_dims->h) || y < 0) {
    printf("Error: index %d is out of bounds for axis 0 with size %d. Returning -1\n", y, img_dims->h);
    return -1;
  } else {
    return x + img_dims->w * y;
  }
}

// Function 10c - Converts 2d coordinates into 1d coordinates (for 1d arrays) - using two uint16_t values for dimensions
int32_t indx1d_c(const int32_t y, const int32_t x, const uint16_t img_height, const uint16_t img_width)
{
  if (x >= (img_width) || x < 0) {
    printf("Error: index x=%d is out of bounds for axis 0 with size %d. Returning -1\n", x, img_width);
    return -1;
  } else if (y >= (img_height) || y < 0) {
    printf("Error: index y=%d is out of bounds for axis 1 with size %d. Returning -1\n", y, img_height);
    return -1;
  } else {
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
void Vw_to_Vc(struct FloatVect3 *Vc, struct FloatVect3 *Vw, struct FloatRMat *Rrw, struct FloatVect3 *VRw,
              struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose)
{
  struct FloatVect3 Vr;

  // Print log only if enabled
  if (verbose != 0) {
    // Rotation matrix - World coordinate system expressed in the robot coordinate system
    printf("Rrw\n");
    printf("%f, %f, %f,\n", Rrw->m[0], Rrw->m[1], Rrw->m[2]);
    printf("%f, %f, %f,\n", Rrw->m[3], Rrw->m[4], Rrw->m[5]);
    printf("%f, %f, %f\n\n", Rrw->m[6], Rrw->m[7], Rrw->m[8]);

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
  if (verbose != 0) {
    // Robot coordinates
    printf("Vr\n");
    printf(" %f\n %f\n %f\n\n", Vr.x, Vr.y, Vr.z);

    // Rotation matrix - Robot coordinate system expressed in the camera coordinate system
    printf("Rcr\n");
    printf("%f, %f, %f,\n", Rcr->m[0], Rcr->m[1], Rcr->m[2]);
    printf("%f, %f, %f,\n", Rcr->m[3], Rcr->m[4], Rcr->m[5]);
    printf("%f, %f, %f\n\n", Rcr->m[6], Rcr->m[7], Rcr->m[8]);

    // Vector coordinates - Camera in the robot frame system
    printf("VCa (camera location)\n");
    printf(" %f\n %f\n %f\n\n", VCr->x, VCr->y, VCr->z);
  }

  // Camera coordinates from robot coordinates
  Va_to_Vb(Vc, &Vr, Rcr, VCr);

  // Print log only if enabled
  if (verbose != 0) {
    // Camera coordinates
    printf("Vc\n");
    printf(" %f\n %f\n %f\n\n", Vc->x, Vc->y, Vc->z);
  }
}


// Function 14 - Function wrapper to convert a point in the camera coordinate system back to a point in the world coordinate system
void Vc_to_Vw(struct FloatVect3 *Vw, struct FloatVect3 *Vc, struct FloatRMat *Rrw, struct FloatVect3 *VRw,
              struct FloatRMat *Rcr, struct FloatVect3 *VCr, const uint8_t verbose)
{
  struct FloatVect3 Vr;

  // Agent coordinates from camera coordinates
  Vb_to_Va(&Vr, Vc, Rcr, VCr);

  // Print log only if enabled
  if (verbose != 0) {
    // Back to robot coordinates
    printf("Vr - back calculated\n"); \
    printf(" %f\n %f\n %f\n\n", Vr.x, Vr.y, Vr.z);
  }


  // World coordinates from a coordinates
  Vb_to_Va(Vw, &Vr, Rrw, VRw);

  // Print log only if enabled
  if (verbose != 0) {
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
  struct FloatVect2 difference;
  float angle;

  VECT2_DIFF(difference, *VSETPOINTwned, *stateGetPositionNed_f());
  angle = atan2f(difference.y, difference.x);
  return angle;
}


// Function 17a - Function to calculate median disparity to a point (Vi) in an image (img), using a kernel structure (kernel_median)
uint8_t median_disparity_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median)
{
  // Creating Start and stop coordinates of in the image coordinate system, based on kernel size
  uint8_t VSTARTi_y = Vi->y  - (kernel_median->h / 2);
  uint8_t VSTARTi_x = Vi->x - (kernel_median->w / 2);
  uint8_t VSTOPi_y = Vi->y + (kernel_median->h / 2);
  uint8_t VSTOPi_x = Vi->x + (kernel_median->w / 2);

  // In case the upper bounds of the kernel are outside of the image area
  // (lower bound not important because of uint8_t type converting everything below 0 to 0):
  if (VSTOPi_y > img->h) {
    VSTOPi_y = (img->h - 1);
  }
  if (VSTOPi_x > img->w) {
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
  for (uint8_t Vi_y = VSTARTi_y; Vi_y < (VSTOPi_y + 1); Vi_y++) {
    for (uint8_t Vi_x = VSTARTi_x; Vi_x < (VSTOPi_x + 1); Vi_x++) {
      // Calculating kernel coordinates
      Vk_y = Vi_y - VSTARTi_y;
      Vk_x = Vi_x - VSTARTi_x;

      // Converting 2d indices to 1d indices
      index_img = indx1d_a(Vi_y, Vi_x, img);
      index_kernel = indx1d_c(Vk_y, Vk_x, median_kernel.h, median_kernel.w);


      // Saving disparity values of image underneath the kernel, into the kernel buffer
      ((uint8_t *) median_kernel.buf_values)[index_kernel] = ((uint8_t *) img->buf)[index_img];
    }
  }

  // Calculating median disparity value of values recoded by the kernel
  median = getMedian(((uint8_t *) median_kernel.buf_values), (median_kernel.h * median_kernel.w)); //

  return median;
}


// Function 17b - Function to calculate median depth (cm) to a point (Vi) in a 16bit image (img), using a kernel structure (kernel_median)
uint16_t median_depth_to_point(struct point_t *Vi, struct image_t *img, struct kernel_C1 *kernel_median)
{
  // Creating Start and stop coordinates of in the image coordinate system, based on kernel size
  uint16_t VSTARTi_y = Vi->y  - (kernel_median->h / 2);
  uint16_t VSTARTi_x = Vi->x - (kernel_median->w / 2);
  uint16_t VSTOPi_y = Vi->y + (kernel_median->h / 2);
  uint16_t VSTOPi_x = Vi->x + (kernel_median->w / 2);

  // In case the upper bounds of the kernel are outside of the image area
  // (lower bound not important because of uint8_t type converting everything below 0 to 0):
  if (VSTOPi_y > img->h) {
    VSTOPi_y = (img->h - 1);
  }
  if (VSTOPi_x > img->w) {
    VSTOPi_x = (img->w - 1);
  }



  // Declaring kernel coordinates
  uint16_t Vk_y;
  uint16_t Vk_x;

  // Declaring 1d indices (for result of transforming 2d coordinates into 1d coordinate)
  int32_t index_img;
  int32_t index_kernel;

  // Declaring variable to store median in
  uint16_t median;


  // Here we get the median value of a block in the middle of an image using a kernel structure
  for (uint16_t Vi_y = VSTARTi_y; Vi_y < (VSTOPi_y + 1); Vi_y++) {
    for (uint16_t Vi_x = VSTARTi_x; Vi_x < (VSTOPi_x + 1); Vi_x++) {
      // Calculating kernel coordinates
      Vk_y = Vi_y - VSTARTi_y;
      Vk_x = Vi_x - VSTARTi_x;

      // Converting 2d indices to 1d indices
      index_img = indx1d_a(Vi_y, Vi_x, img);
      index_kernel = indx1d_c(Vk_y, Vk_x, median_kernel.h, median_kernel.w);


      // Saving disparity values of image underneath the kernel, into the kernel buffer
      ((uint16_t *) median_kernel.buf_values)[index_kernel] = ((uint16_t *) img->buf)[index_img];

    }
  }

  // Calculating median disparity value of values recoded by the kernel
  median = getMedian16bit(((uint16_t *) median_kernel.buf_values), (median_kernel.h * median_kernel.w)); //

  return median;
}




// Function 18a - Function to find "best" (closest ideal pathway to goal from robot to edge to goal) - Using disparity image
// Returns a 3d Vector to the best "edge" and 1 if any edge is found and 0 if no edge is found.
uint8_t find_best_edge_coordinates(
  struct FloatVect3
  *VEDGECOORDINATESc, // Declared vector of coordinates of "best" edge detected in camera coordinate system
  struct FloatVect3 *VTARGETc,      // Declared vector of coordinates of goal in camera coordinate system
  struct image_t *img_edges,        // Image obtained from the external Sobel edge detection function = sobel_OCV
  struct image_t *img_disparity,      // Image obtained after simple block matching
  struct crop_t *edge_search_area,    // This structure holds information about the window in which edges are searched in
  uint8_t threshold,            // Above this disparity (pixels) edges are eligible for the WedgeBug algorithm (i.e. edges cannot be very far away)
  int16_t max_confidence          // This is the max confidence that edges were found - Works like a threshold
)
{
  // Declarations
  struct FloatVect3
    VEDGEc;       // A vector to save the point of an eligible detected edge point in the camera coordinate system.
  struct FloatVect3 VROBOTCENTERc;  // A vector to hold camera center coordinates in the camera coordinate system.
  struct point_t
    VCLOSESTEDGEi;     // A vector to save the point of the "best" eligible detected edge point in the image coordinate system.
  float f_distance_edge_to_goal;    // Saves distance from edge to goal
  float f_distance_robot_to_edge;   // Saves distance from robot to goal
  float distance;           // This stores distance from edge to goal. Its initialized with 255 as basically any edge found will be closer than that and will replace 255 met
  uint8_t edge_value;         // Variable to store the intensity value of a pixel in the img_edge
  uint8_t disparity;          // variable to store the disparity level of a pixel in the img_disparity
  uint8_t disparity_best;       // Variable to store disparity level of associated best edge found (for debugging)
  int16_t confidence;         // Confidence = number of edge pixels found
  int32_t indx;             // Variable to store 1d index calculated from 2d index

  // Initializations
  VROBOTCENTERc.x = 0.0; VROBOTCENTERc.y = 0.0; VROBOTCENTERc.z = 0.0;
  VCLOSESTEDGEi.x = 0; VCLOSESTEDGEi.y = 0;
  distance = 255;
  confidence = 0;
  disparity_best = 0;


  // Loop to calculate position (in image) of point closes to hypothetical target - Start
  for (uint16_t y = edge_search_area->y; y < (edge_search_area->y + edge_search_area->h);
       y++) { //for (uint32_t y = edge_search_area.y; y < (edge_search_area.y + edge_search_area.h); y++)
    for (uint16_t x = edge_search_area->x; x < (edge_search_area->x + edge_search_area->w); x++) {
      indx = indx1d_a(y, x, img_edges); // We convert the 2d index [x,y] into a 1d index
      edge_value = ((uint8_t *) img_edges->buf)[indx]; // We save the intensity of the current point
      disparity = ((uint8_t *) img_disparity->buf)[indx]; // We save the disparity of the current point

      // Two conditions must be met for an edge to be considered a viable route for the drone:
      // 1) This disparity of the current coordinate (x,y) must coincide with an edge pixel
      //    (as all non-edge pixels have been set to 0) - (edge_value != 0)
      // 2) The disparity of the current coordinate (x, y) must be above a certain threshold. This simulates vision cone - (disparity > threshold_disparity_of_edges)
      if ((edge_value != 0) && (disparity > threshold)) {
        // We increase the confidence for every edge found
        confidence++;
        Bound(confidence, 0, max_confidence);

        // We determine the offset from the principle point
        int32_t y_from_c = y - c_img_cropped.y; // NOTE. The variable "c_img_cropped" is a global variable
        int32_t x_from_c = x - c_img_cropped.x; // NOTE. The variable "c_img_cropped" is a global variable
        // We derive the 3d scene point using from the disparity saved earlier
        Vi_to_Vc(&VEDGEc, y_from_c, x_from_c, disparity, b, f); // NOTE. The variables "b" and "f" are a global variables
        // Calculating Euclidean distance (N2) - Edge to goal
        f_distance_edge_to_goal = float_vect3_norm_two_points(VTARGETc, &VEDGEc);
        // Calculating Euclidean distance (N2) - robot to edge
        f_distance_robot_to_edge = float_vect3_norm_two_points(&VEDGEc, &VROBOTCENTERc);


        // If current distance (using distance vector) is smaller than the previous minimum distance
        // measure then save new distance and point coordinates associated with it
        if ((f_distance_robot_to_edge + f_distance_edge_to_goal) < distance) {
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

  if (confidence == max_confidence) {
    ((uint8_t *) img_edges->buf)[indx1d_a(VCLOSESTEDGEi.y, VCLOSESTEDGEi.x, img_edges)] = 255;
    printf("Viable closest edge found: [%d, %d] (disparity = %d)\n", VCLOSESTEDGEi.y, VCLOSESTEDGEi.x, disparity_best);

    printf("At distance: %f\n", distance);
    confidence = 0;
    return 1;
  } else {
    printf("No viable edge found\n");
    confidence = 0;
    return 0;
  }
}



// Function 18b - Function to find "best" (closest ideal pathway to goal from robot to edge to goal) - Using depth image
// Returns a 3d Vector to the best "edge" and 1 if any edge is found and 0 if no edge is found.
uint8_t find_best_edge_coordinates2(
  struct FloatVect3
  *VEDGECOORDINATESc,   // Declared vector of coordinates of "best" edge detected in camera coordinate system
  struct FloatVect3 *VTARGETc,      // Declared vector of coordinates of goal in camera coordinate system
  struct image_t *img_edges,        // Image obtained from the external Sobel edge detection function = sobel_OCV
  struct image_t *img_depth,        // Image holding depth values (cm) obtained from the disparity image
  struct crop_t *edge_search_area,    // This structure holds information about the window in which edges are searched in
  uint16_t threshold,           // Below this depth (cm) edges are eligible for the WedgeBug algorithm (i.e. edges cannot be very far away)
  int16_t max_confidence          // This is the max confidence that edges were found - Works like a threshold
)
{
  // Declarations
  struct FloatVect3
    VEDGEc;       // A vector to save the point of an eligible detected edge point in the camera coordinate system.
  struct FloatVect3 VROBOTCENTERc;  // A vector to hold camera center coordinates in the camera coordinate system.
  struct point_t
    VCLOSESTEDGEi;     // A vector to save the point of the "best" eligible detected edge point in the image coordinate system.
  float f_distance_edge_to_goal;    // Saves distance from edge to goal
  float f_distance_robot_to_edge;   // Saves distance from robot to goal
  float distance;           // This stores distance from edge to goal. Its initialized with 255 as basically any edge found will be closer than that and will replace 255 meters
  uint8_t edge_value;         // Variable to store the intensity value of a pixel in the img_edge
  uint16_t depth;           // Variable to store the depth (in cm) of a pixel in the img_disparity
  uint16_t depth_best;        // Variable to store depth (cm) level of associated best edge found (for debugging)
  int16_t confidence;         // Confidence = number of edge pixels found
  int32_t indx;             // Variable to store 1d index calculated from 2d index

  // Initializations
  VROBOTCENTERc.x = 0.0; VROBOTCENTERc.y = 0.0; VROBOTCENTERc.z = 0.0;
  VCLOSESTEDGEi.x = 0; VCLOSESTEDGEi.y = 0;
  distance = 255;
  confidence = 0;
  depth_best = 0;


  // Loop to calculate position (in image) of point closes to hypothetical target - Start
  for (uint16_t y = edge_search_area->y; y < (edge_search_area->y + edge_search_area->h);
       y++) { //for (uint32_t y = edge_search_area.y; y < (edge_search_area.y + edge_search_area.h); y++)
    for (uint16_t x = edge_search_area->x; x < (edge_search_area->x + edge_search_area->w); x++) {
      indx = indx1d_a(y, x, img_edges); // We convert the 2d index [x,y] into a 1d index
      edge_value = ((uint8_t *) img_edges->buf)[indx]; // We save the intensity of the current point
      depth = ((uint16_t *) img_depth->buf)[indx];

      // Two conditions must be met for an edge to be considered a viable route for the drone:
      // 1) This disparity of the current coordinate (x,y) must coincide with an edge pixel
      //    (as all non-edge pixels have been set to 0) - (edge_value != 0)
      // 2) The disparity of the current coordinate (x, y) must be above a certain threshold. This simulates vision cone - (disparity > threshold_disparity_of_edges)
      if ((edge_value != 0) && (depth < threshold)) {
        // We increase the confidence for every edge found
        confidence++;
        Bound(confidence, 0, max_confidence);

        // We determine the offset from the principle point
        int32_t y_from_c = y - c_img_cropped.y; // NOTE. The variable "c_img_cropped" is a global variable
        int32_t x_from_c = x - c_img_cropped.x; // NOTE. The variable "c_img_cropped" is a global variable
        // We derive the 3d scene point using the depth from the depth image (img_depth). Note depth is in cm, but function takes m. Thus, we convert
        Vi_to_Vc_depth(&VEDGEc, y_from_c, x_from_c, (depth / 100.00),
                       f); // NOTE. The variables "b" and "f" are a global variables

        // Calculating Euclidean distance (N2) - Edge to goal
        f_distance_edge_to_goal =  float_vect3_norm_two_points(VTARGETc, &VEDGEc);
        // Calculating Euclidean distance (N2) - robot to edge
        f_distance_robot_to_edge =  float_vect3_norm_two_points(&VEDGEc, &VROBOTCENTERc);


        // If current distance (using distance vector) is smaller than the previous minimum distance
        // measure then save new distance and point coordinates associated with it
        if ((f_distance_robot_to_edge + f_distance_edge_to_goal) < distance) {
          // Saving closest edge point, in camera coordinate system
          VEDGECOORDINATESc->x = VEDGEc.x;
          VEDGECOORDINATESc->y = VEDGEc.y;
          VEDGECOORDINATESc->z = VEDGEc.z;
          // Saving disparity at point
          depth_best = depth;
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

  if (confidence == max_confidence) {
    ((uint8_t *) img_edges->buf)[indx1d_a(VCLOSESTEDGEi.y, VCLOSESTEDGEi.x, img_edges)] = 255;
    printf("Viable closest edge found: [%d, %d] (depth = %f)\n", VCLOSESTEDGEi.y, VCLOSESTEDGEi.x, (depth_best / 100.00));

    printf("At distance: %f\n", distance);
    confidence = 0;
    return 1;
  } else {
    printf("No viable edge found\n");
    confidence = 0;
    return 0;
  }
}



// Function 19 - Function to determine if setpoint was reached (it was reached if distance is below a set threshold)
uint8_t is_setpoint_reached(struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold)
{
  if (float_vect3_norm_two_points(VGOAL, VCURRENTPOSITION) < threshold) {return 1;}
  else                                  {return 0;}
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
  if (float_norm_two_angles(target_angle, current_angle) < threshold) {return 1;}
  else                                {return 0;}
}

// Function 22
uint8_t are_setpoint_and_angle_reached(
  struct FloatVect3 *VGOAL, struct FloatVect3 *VCURRENTPOSITION, float threshold_setpoint,
  float target_angle, float current_angle, float threshold_angle)
{
  if ((float_vect3_norm_two_points(VGOAL, VCURRENTPOSITION) < threshold_setpoint)
      && (float_norm_two_angles(target_angle, current_angle) < threshold_angle)) {return 1;}
  else                                                                            {return 0;}
}


// Function 23 - Function to convert 16bit disparity (unaltered from OCV output i.e. fixed point 16bit numbers) into 16bit depth values (cm)
void disp_to_depth_img(struct image_t *img_input, struct image_t *img16bit_output)
{
  //int n = 89;
  float disparity = 1;

  //printf("C location %d = %d\n", n, ((int16_t*)img16bit_input->buf)[n]);


  // Converting disparity values into depth (cm)
  for (int32_t i = 0; i < (img_input->h * img_input->w); i++) {

    if (img_input->type == IMAGE_GRAYSCALE) {
      disparity = disp_to_depth(((uint8_t *)img_input->buf)[i], b, f);
    } else if (img_input->type == IMAGE_INT16) {
      disparity = disp_to_depth_16bit(((uint16_t *)img_input->buf)[i], b, f);
    } else {
      printf("ERROR: function does not support image type %d. Breaking out of function.", img_input->type);
    }


    /*
    if(i == n)
    {
      printf("Depth in meters at %d = %f\n", n, disparity);
    }*/

    disparity = disparity * 100;

    /*if(i == n)
    {
      printf("Depth in cm at %d = %f\n", n, disparity);
    }*/


    ((uint16_t *)img16bit_output->buf)[i] = round(disparity);
  }
  //printf("Depth in cm at %d = %d\n", n, ((int16_t*)img16bit_output->buf)[n]);
}

// Function 24  - Function that encapsulates all of the background processes. Originally this was in the periodic function,
// but since it is used in the direct and the guidance navigation modes of the state machine, I made it a callable function
// for convenience
void background_processes(uint8_t save_images_flag)
{
  //Background processes
  // 1. Converting left and right image to 8bit grayscale for further processing
  image_to_grayscale(&img_left, &img_left_int8); // Converting left image from UYVY to gray scale for saving function
  image_to_grayscale(&img_right, &img_right_int8); // Converting right image from UYVY to gray scale for saving function

  // 2. Deriving disparity map from block matching (left image is reference image)
  SBM_OCV(&img_disparity_int8_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities,
          1);// Creating cropped disparity map image
  // For report: creating image for saving 1
  if (save_images_flag) {save_image_HM(&img_disparity_int8_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b_img1_post_SBM.bmp", heat_map_type);}

  /*
  // Optional thresholding of disparity map
  uint8_t thresh = 1;
  for (int32_t i = 0; i < (img_disparity_int8_cropped.h*img_disparity_int8_cropped.w); i++)
  {
    uint8_t disparity = ((uint8_t*)img_disparity_int8_cropped)[i];
    if(disparity < thresh)
    {
      ((uint8_t*)img_disparity_int8_cropped)[i] = 0; // if below disparity assume object is indefinately away
    }
  }*/

  // 3. Morphological operations 1
  // Needed to smoove object boundaries and to remove noise removing noise
  opening_OCV(&img_disparity_int8_cropped, &img_middle_int8_cropped, SE_opening_OCV, 1);
  // For report: creating image for saving 2
  if (save_images_flag) {save_image_HM(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b_img2_post_opening_8bit.bmp", heat_map_type);}

  closing_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped, SE_closing_OCV, 1);
  // For report: creating image for saving 3
  if (save_images_flag) {save_image_HM(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b_img3_post_closing_8bit.bmp", heat_map_type);}

  dilation_OCV(&img_middle_int8_cropped, &img_middle_int8_cropped, SE_dilation_OCV_1, 1);
  // For report: creating image for saving 4
  if (save_images_flag) {save_image_HM(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b_img4_post_dilation_8bit.bmp", heat_map_type);}

  // 4. Depth image
  disp_to_depth_img(&img_middle_int8_cropped, &img_depth_int16_cropped);
  // For report: creating image for saving 4
  if (save_images_flag) {save_image_HM(&img_depth_int16_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b_img5_post_depth_16bit.bmp", heat_map_type);}

  // 5. Sobel edge detection
  sobel_OCV(&img_depth_int16_cropped, &img_edges_int8_cropped, SE_sobel_OCV, threshold_edge_magnitude);
  // For report: creating image for saving 5
  if (save_images_flag) {save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b_img6_post_sobel_8bit.bmp");}
}


// Function 25  - Function that encapsulates all of the background processes. Originally this was in the periodic function,
// but since it is used in the direct and the guidance navigation modes of the state machine, I made it a callable function
// for convenience
void background_processes_16bit(uint8_t save_images_flag)
{
  //Background processes
  // 1. Converting left and right image to 8bit grayscale for further processing
  image_to_grayscale(&img_left, &img_left_int8); // Converting left image from UYVY to gray scale for saving function
  image_to_grayscale(&img_right, &img_right_int8); // Converting right image from UYVY to gray scale for saving function

  // 2. Deriving disparity map from block matching (left image is reference image)
  SBM_OCV(&img_disparity_int16_cropped, &img_left_int8, &img_right_int8, N_disparities, block_size_disparities,
          1);// Creating cropped disparity map image
  // For report: creating image for saving 1
  if (save_images_flag) {save_image_HM(&img_disparity_int16_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b2_img1_post_SBM_16bit.bmp", heat_map_type);}

  //printf("maximum_intensity = %d\n", maximum_intensity(&img_disparity_int16_cropped));

  /*
  // Optional thresholding of disparity map
  uint8_t thresh = 1;
  for (int32_t i = 0; i < (img_disparity_int8_cropped.h*img_disparity_int8_cropped.w); i++)
  {
    uint8_t disparity = ((uint8_t*)img_disparity_int8_cropped)[i];
    if(disparity < thresh)
    {
      ((uint8_t*)img_disparity_int8_cropped)[i] = 0; // if below disparity assume object is indefinately away
    }
  }*/

  // 3. Morphological operations 1
  // Needed to smoove object boundaries and to remove noise removing noise

  closing_OCV(&img_disparity_int16_cropped, &img_disparity_int16_cropped, SE_closing_OCV, 1);
  // For report: creating image for saving 3
  if (save_images_flag) {save_image_HM(&img_disparity_int16_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b2_img2_post_closing_16bit.bmp", heat_map_type);}


  opening_OCV(&img_disparity_int16_cropped, &img_disparity_int16_cropped, SE_opening_OCV, 1);
  // For report: creating image for saving 2
  if (save_images_flag) {save_image_HM(&img_disparity_int16_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b2_img3_post_opening_16bit.bmp", heat_map_type);}



  dilation_OCV(&img_disparity_int16_cropped, &img_disparity_int16_cropped, SE_dilation_OCV_1, 1);
  // For report: creating image for saving 4
  if (save_images_flag) {save_image_HM(&img_disparity_int16_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b2_img4_post_dilation_16bit.bmp", heat_map_type);}


  // 4. Depth image
  disp_to_depth_img(&img_disparity_int16_cropped, &img_depth_int16_cropped);
  // For report: creating image for saving 4
  if (save_images_flag) {save_image_HM(&img_depth_int16_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b2_img5_post_depth_16bit.bmp", heat_map_type);}


  // 5. Sobel edge detection
  sobel_OCV(&img_depth_int16_cropped, &img_edges_int8_cropped, SE_sobel_OCV, threshold_edge_magnitude);
  // For report: creating image for saving 5
  if (save_images_flag) {save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/for_report/b2_img6_post_sobel_8bit.bmp");}

  // 6. Morphological  operations 2
  // This is needed so that when using the edges as filters (to work on depth values
  // only found on edges) the underlying depth values are those of the foreground
  // and not the background
  erosion_OCV(&img_depth_int16_cropped, &img_depth_int16_cropped, SE_erosion_OCV, 1);
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
void wedgebug_init()
{
  //printf("Wedgebug init function was called\n");


  // Images creation process:
  // Creation of images
  // Creating structure to hold dimensions of normal camera images
  img_dims.w = WEDGEBUG_CAMERA_LEFT_WIDTH; img_dims.h = WEDGEBUG_CAMERA_LEFT_HEIGHT;


  image_create(&img_left, img_dims.w, img_dims.h, IMAGE_YUV422);  // To store left camera image
  image_create(&img_right, img_dims.w, img_dims.h, IMAGE_YUV422);// To store right camera image
  image_create(&img_left_int8, img_dims.w, img_dims.h, IMAGE_GRAYSCALE);  // To store gray scale version of left image
  image_create(&img_right_int8, img_dims.w, img_dims.h, IMAGE_GRAYSCALE);  // To store gray scale version of left image


  // Creation of images - Cropped:
  // Calculating cropped image details (x, y, width and height)
  post_disparity_crop_rect(&img_cropped_info, &img_dims, N_disparities, block_size_disparities);

  // Creating structure to hold dimensions of cropped camera images
  img_cropped_dims.w = img_cropped_info.w; img_cropped_dims.h = img_cropped_info.h;

  printf("img_cropped_info [w, h, x, y] = [%d, %d, %d, %d]\n", img_cropped_info.w, img_cropped_info.h, img_cropped_info.x,
         img_cropped_info.y);

  // Creating empty images - cropped - 8bit
  image_create(&img_left_int8_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_GRAYSCALE);// To store cropped depth - 8 bit
  image_create(&img_disparity_int8_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_GRAYSCALE);// To store cropped depth - 8 bit
  image_create(&img_middle_int8_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_GRAYSCALE); // To store intermediate image data from processing - 8 bit
  image_create(&img_edges_int8_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_GRAYSCALE); // To store edges image data from processing - 8 bit

  // Creating empty images - cropped - 16bit
  image_create(&img_disparity_int16_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_INT16);// To store cropped disparity - 16 bit
  image_create(&img_depth_int16_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_INT16);// To store cropped depth - 16 bit
  image_create(&img_middle_int16_cropped, img_cropped_dims.w, img_cropped_dims.h,
               IMAGE_INT16);// To store cropped middle image - 16 bit





  // Creation of kernels:
  // Creating structure to hold dimensions of kernels
  K_median_w = 43;
  K_median_h = 43;

  kernel_median_dims.w = K_median_w; kernel_median_dims.h = K_median_h;
  // Creating empty kernel:
  kernel_create(&median_kernel, kernel_median_dims.w, kernel_median_dims.h, IMAGE_GRAYSCALE);
  kernel_create(&median_kernel16bit, kernel_median_dims.w, kernel_median_dims.h, IMAGE_INT16);

  printf("median_kernel16bit [buf_size, h, w, type]  = [%d, %d, %d, %d]\n", median_kernel16bit.buf_size,
         median_kernel16bit.h, median_kernel16bit.w, median_kernel16bit.type);




  // Adding callback functions
  cv_add_to_device(&WEDGEBUG_CAMERA_LEFT, copy_left_img_func, WEDGEBUG_CAMERA_LEFT_FPS);
  cv_add_to_device(&WEDGEBUG_CAMERA_RIGHT, copy_right_img_func, WEDGEBUG_CAMERA_RIGHT_FPS);


  //Initialization of constant rotation matrices and transition vectors for frame to frame transformations
  // 1) Rotation matrix and transition vector to transform from world ENU frame to world NED frame
  Rwnedwenu.m[0] = 0; Rwnedwenu.m[1] = 1; Rwnedwenu.m[2] = 0;
  Rwnedwenu.m[3] = 1; Rwnedwenu.m[4] = 0; Rwnedwenu.m[5] = 0;
  Rwnedwenu.m[6] = 0; Rwnedwenu.m[7] = 0; Rwnedwenu.m[8] = -1;
  VNEDwenu.x = 0;
  VNEDwenu.y = 0;
  VNEDwenu.z = 0;
  // 2) Rotation matrix and transition vector to transform from robot frame to camera frame
  Rcr.m[0] = 0; Rcr.m[1] = 1; Rcr.m[2] = 0;
  Rcr.m[3] = 0; Rcr.m[4] = 0; Rcr.m[5] = 1;
  Rcr.m[6] = 1; Rcr.m[7] = 0; Rcr.m[8] = 0;
  VCr.x = 0;// i.e the camera is x meters away from robot center
  VCr.y = 0;
  VCr.z = 0;

  // Initializing goal vector in the NED world coordinate system
  VGOALwenu.x = WaypointX(WP_GOAL);
  VGOALwenu.y = WaypointY(WP_GOAL);
  VGOALwenu.z = WaypointAlt(WP_GOAL);
  Va_to_Vb(&VGOALwned, &VGOALwenu, &Rwnedwenu, &VNEDwenu);


  // Initializing start vector in the NED world coordinate system
  VSTARTwenu.x = WaypointX(WP_START);
  VSTARTwenu.y = WaypointY(WP_START);
  VSTARTwenu.z = WaypointAlt(WP_START);
  Va_to_Vb(&VSTARTwned, &VSTARTwenu, &Rwnedwenu, &VNEDwenu);


  // Calculating principal points of normal image and cropped image
  c_img.y = img_dims.h / 2;
  c_img.x = img_dims.w / 2;
  principal_points(&c_img_cropped, &c_img,
                   &img_cropped_info); // Calculates principal points for cropped image, considering the original dimensions


  // Initializing structuring element sizes
  SE_opening_OCV = 13;  // SE size for the opening operation
  SE_closing_OCV = 13;  // SE size for the closing operation
  SE_dilation_OCV_1 =
    51;//25;//51;//301;// SE size for the first dilation operation (Decides where edges are detected, increase to increase drone safety zone NOTE. This functionality should be replaced with c space expansion)
  SE_dilation_OCV_2 =
    11; // SE size for the second dilation operation (see state 3 "WEDGEBUG_START" and state 6 "POSITION_EDGE")
  SE_sobel_OCV = 5;     // SE size for the sobel operation, to detect edges
  SE_erosion_OCV =
    11;  // SE size for the erosion operation (see state 3 "WEDGEBUG_START" and state 6 "POSITION_EDGE", its needed to "drag" the depth of the foreground objects over the edges detected)

  // Setting thresholds - non-calculated
  threshold_median_depth =
    700;     //! Below this median depth (cm), an obstacle is considered to block the way (i.e. the blocking obstacle needs to be close)
  threshold_depth_of_edges = 770;   //! Below this depth (cm) edges are eligible for the WedgeBug algorithm
  threshold_edge_magnitude =
    5000;  // Edges with a magnitude above this value are detected. Above this value, edges are given the value 127, otherwise they are given the value zero.
  threshold_distance_to_goal = 0.25;  // Above this threshold (m), the goal is considered reached
  threshold_distance_to_goal_direct =
    1.0;//Above this threshold (m), the goal is considered reached in DIRECT_CONTROL mode
  threshold_distance_to_angle = 0.0004; // Above this threshold (radians), the angle/heading is considered reached

  // Setting thresholds - calculated
  //threshold_median_disparity = depth_to_disp((threshold_median_depth / 100.00), b, f);  //8// Above this median disparity, an obstacle is considered to block the way. >60 = close than 35cm
  //threshold_median_depth = (uint16_t) disp_to_depth(threshold_median_disparity, b, f) * 100;
  //threshold_disparity_of_edges = depth_to_disp((threshold_depth_of_edges / 100.00), b, f);  //5// Above this underlying disparity value, edges are considers eligible for detection

  // Initializing confidence parameters
  obstacle_confidence = 0;        // This is the confidence that an obstacle was spotted
  free_path_confidence = 0;       // This is the confidence that no obstacle was spotted
  position_confidence = 0;        // This is the confidence that the desired position was reached
  heading_confidence = 0;         // This is the confidence that the desired heading is reached
  //edge_found_micro_confidence = 0;    // This is the confidence that an edge was found
  edge_found_macro_confidence = 0;    // This is the confidence that an edge was found
  no_edge_found_confidence = 0;     // This is the confidence that no edge was found
  max_obstacle_confidence = 10;     // This is the max confidence that an obstacle was spotted
  max_free_path_confidence = 5;     // This is the max confidence that an obstacle was not spotted
  max_position_confidence = 30;     // This is the max confidence that a specific position was reached
  max_heading_confidence = 5;       // This is the max confidence that a specific heading was reached
  max_edge_found_micro_confidence = 1;//50  // This is the max confidence that edges were found
  max_edge_found_macro_confidence =
    1;  // This is the max confidence that edges were found (can be higher if angular change is slower or speed of periodic function is increase)
  max_no_edge_found_confidence = 10;    // This is the max confidence that no edges were found

  // Initializing boolean flags
  is_start_reached_flag = 0;        // Set to 1 if start position is reached, 0 otherwise
  is_setpoint_reached_flag = 0;     // Set to 1 if setpoint is reached, 0 otherwise
  is_obstacle_detected_flag = 0;      // Set to 1 if obstacle is detected, 0 otherwise
  is_path_free_flag = 0;          // Set to 1 if no obstacle is detected, 0 otherwise
  is_heading_reached_flag = 0;      // Set to 1 if heading is reached, 0 otherwise
  is_edge_found_macro_flag = 0;         // Set to 1 if best edge (according to macro confidence) was found, 0 otherwise
  is_edge_found_micro_flag = 0;         // Set to 1 if best edge (according to micro confidence) was found, 0 otherwise
  is_no_edge_found_flag = 0;        // Set to 1 if no edge was identified, 0 otherwise
  is_state_changed_flag = 0;        // Set to 1 if state was changed, 0 otherwise
  is_mode_changed_flag = 0;
  initial_heading.is_left_reached_flag = 0; // The scan has not reached the left maximum angle yet
  initial_heading.is_right_reached_flag = 0;// The scan has not reached the right maximum angle yet

  is_total_timer_on_flag = 0;
  threshold_distance_to_goal_manual = 0.5;

  save_images_flag = 1;       // For report: Flag to indicate if images should be saved



  // Initializing area over which edges are searched in

  // p3DWedgeBug:
  edge_search_area.y = 0;
  edge_search_area.h = img_disparity_int8_cropped.h;
  edge_search_area.x = 0;
  edge_search_area.w = img_disparity_int8_cropped.w;

//  // p2DWedgeBug:
//  edge_search_area.y = 216/2;
//  edge_search_area.h = 1;
//  edge_search_area.x = 0;
//  edge_search_area.w = img_disparity_int8_cropped.w;


  // Initializing Edge scan structure
  initial_heading.initiated = 0;      // 0 = it can be overwritten



  // Initializing debugging options

  allow_state_change_MOVE_TO_GOAL = 1; // Allows state change from within state "MOVE_TO_GOAL"
  allow_state_change_POSITION_GOAL = 1; // Allows state change from within state "POSITION_GOAL"
  allow_state_change_WEDGEBUG_START = 1; // Allows state change from within state "WEDGEBUG_START"
  allow_state_change_MOVE_TO_EDGE = 1; // Allows state change from within state "MOVE_TO_EDGE"
  allow_state_change_POSITION_EDGE = 1; // Allows state change from within state "POSITION_EDGE"
  allow_state_change_EDGE_SCAN = 1; // Allows state change from within state "EDGE_SCAN"

  // Other initializations
  previous_state = 0;           // Variable for state machine memory
  previous_mode = 0;
  VDISTANCEPOSITIONwned.x =
    VSTARTwned.x; // Initializing a vector to hold the current position, which is needed for calculating the distance traveled
  VDISTANCEPOSITIONwned.y = VSTARTwned.y;
  VDISTANCEPOSITIONwned.z = VSTARTwned.z;
  clock_total_time =
    0;         // Clock to measure total time (clock cycles)) it took for the robot to fly from start to goal
  clock_total_time_current = 0;     // Clock to hold time measured at start of the current cycle of the periodic function
  clock_total_time_previous = 0;    // Clock to hold time measured at start of the previous cycle of the periodic function
  distance_traveled = 0;          // Variable to hold the distance traveled of the robot (since start and up to the goal)
  number_of_states = NUMBER_OF_STATES;     // Variable to save the total number of states used in the finite state machine
  distance_robot_edge_goal =
    99999;     // Variable to hold distance from robot to edge to goal (used in EDGE_SCAN (9) state) - initialized with unreasonably high number
  clock_background_processes = 0;
  clock_FSM = 0;
  heat_map_type = 2; // Heat map used when saving image

  /*
   * Heat maps:
   * 0 = COLORMAP_AUTUMN
   * 1 = COLORMAP_BONE
   * 2 = COLORMAP_JET
   * 3 = COLORMAP_WINTER
   * 4 = COLORMAP_RAINBOW
   * 5 = COLORMAP_OCEAN
   * 6 = COLORMAP_SUMMER
   * 7 = COLORMAP_SPRING
   * 8 = COLORMAP_COOL
   * 9 = COLORMAP_HSV
   * 10 = COLORMAP_PINK
   * 11 = COLORMAP_HOT
       */

  /*
  enum navigation_state {
    MOVE_TO_GOAL = 1,
    POSITION_GOAL = 2,
    WEDGEBUG_START = 3,
    MOVE_TO_EDGE = 4,
    POSITION_EDGE = 5,
      EDGE_SCAN = 6

  };*/

}





void wedgebug_periodic()
{
  // your periodic code here.
  // freq = 15.0 Hz
  //printf("Wedgebug periodic function was called\n");


  /*
  enum control_mode_state {
  DIRECT_CONTROL = 1,
  AUTONOMOUS_GUIDED = 2,
  AUTONOMOUS_NAV = 3
  };
   */

  // Setting control mode state (needed as switch-case statement cannot use OR statment so that
  // AP_MODE_ATTITUDE_DIRECT and AP_MODE_ATTITUDE_Z_HOLD lead to the same outcome. Hence I use my
  // own states here
  switch (autopilot_get_mode()) {
    case (AP_MODE_ATTITUDE_DIRECT): {current_mode = DIRECT_CONTROL;} break;
    case (AP_MODE_ATTITUDE_Z_HOLD): {current_mode = DIRECT_CONTROL;} break;
    case (AP_MODE_GUIDED): {current_mode = AUTONOMOUS_GUIDED;} break;
    case (AP_MODE_NAV): {current_mode = AUTONOMOUS_NAV;} break;
    default: {printf("Unsupported control mode");} break;
  }

  printf("threshold_median_depth (m) = %f\n", threshold_median_depth / 100.00);

  // Debugging - setting default state
  //set_state(MOVE_TO_GOAL ,1);
  //printf("Current control mode %d\n", current_mode);
  //printf("Current state %d\n", current_state);



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


  // Checking is state was changed, if yes then all flags are reset and the is_mode_changed_flag
  // is set to 1 for this cycle only. Else, the  is_mode_changed_flag is set to 0 again;

  if (current_mode != previous_mode) {
    // Setting flag signifying that the state was changed
    is_mode_changed_flag = 1;

  } else if (is_mode_changed_flag == 1) {
    is_mode_changed_flag = 0;
  }

  if (is_mode_changed_flag == 1) {printf("Mode was changed!!!!!!!!!!!!!!!!!!!1\n");}
  printf("is_mode_changed_flag = %d\n", is_mode_changed_flag);

  previous_mode = current_mode;




  /*
  enum control_mode_state {
  DIRECT_CONTROL = 1,
  AUTONOMOUS_GUIDED = 2,
  AUTONOMOUS_NAV = 3
  };
   */

  /*
  enum navigation_state {
    MOVE_TO_GOAL = 1,
    POSITION_GOAL = 2,
    WEDGEBUG_START = 3,
    MOVE_TO_EDGE = 4,
    POSITION_EDGE = 5,
      EDGE_SCAN = 6

  };*/
  // Finite state machine - Only runs in guided mode
  //if ((autopilot_get_mode() == AP_MODE_GUIDED)) // If AP_MODE_GUIDED block - Start

  // Switch-case statement for current_mode
  switch (current_mode) { // Control mode state machine - Start
    // State 1 ################################################################################################################################################################### State 1: DIRECT_CONTROL
    case DIRECT_CONTROL: {
      printf("DIRECT_CONTROL = %d\n", DIRECT_CONTROL);

      // If the mode was just started, then we initialize the FSM with MOVE_TO_GOAL (1)
      if (is_mode_changed_flag) {
        set_state(MOVE_TO_GOAL, 1);
      }

      // We do not care about the height of the drone when measuring distance to goal in the DIRECT_CONTROL mode
      // So we create this pseudo 2d vector where the z coordinates are the same as the goal. This vector is used
      // to check if robot is close to goal
      struct FloatVect3 VR2dwned = {.x = VRwned.x, .y = VRwned.y, .z = VGOALwned.z};


      // Background processes - Includes image processing for use
      // Turn on for debugging
      //background_processes(save_images_flag);
      background_processes_16bit(save_images_flag);


      switch (current_state) { // Finite state machine - Start
        // State 1 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 1: MOVE_TO_GOAL
        case MOVE_TO_GOAL: {
          printf("MOVE_TO_GOAL = %d\n", MOVE_TO_GOAL);
          //median_disparity_in_front = median_disparity_to_point(&c_img_cropped, &img_disparity_int8_cropped, &median_kernel);
          //In case disparity is 0 (infinite distance or error we set it to one disparity
          // above the threshold as the likelihood that the object is too close is large (as opposed to it being infinitely far away)
          //printf("median_disparity_in_front = %d\n", median_disparity_in_front);
          //printf("median_depth_in_front = %f\n", disp_to_depth(median_disparity_in_front, b, f));
          //float depth = disp_to_depth(median_disparity_in_front, b, f);  // median_depth_in_front / 100.00


          median_depth_in_front = median_depth_to_point(&c_img_cropped, &img_depth_int16_cropped, &median_kernel16bit);
          float depth = median_depth_in_front / 100.00;
          printf("median_depth_in_front = %f\n", depth);
          printf("depth to goal = %f\n", VGOALc.z);


          if ((median_depth_in_front < threshold_median_depth) && (depth < VGOALc.z)) {
            printf("Obstacle is in front of goal\n");
          } else {
            printf("The path to the goal is free!\n");
          }


          is_edge_found_micro_flag  =
            find_best_edge_coordinates2(
              &VEDGECOORDINATESc,
              &VGOALc,//target_point,
              &img_edges_int8_cropped,
              &img_depth_int16_cropped,
              &edge_search_area,
              threshold_depth_of_edges,
              max_edge_found_micro_confidence);


//      // c. Checking if edges are located
//      // Running function to detect and save edge
//      is_edge_found_micro_flag  =
//          find_best_edge_coordinates(
//          &VEDGECOORDINATESc,
//          &VGOALc,//target_point,
//          &img_edges_int8_cropped,
//          &img_middle_int8_cropped,
//          &edge_search_area,
//          threshold_disparity_of_edges,
//          max_edge_found_micro_confidence);
//



          if (save_images_flag) {save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped_marked.bmp");}


          if (is_setpoint_reached_flag) {
            printf("Goal is reached\n");
            set_state(POSITION_GOAL, allow_state_change_MOVE_TO_GOAL);
          } else {
            // If this is the first cycle of this mode, then
            if (is_mode_changed_flag) {
              clock_total_time_current = clock();;
            }

            // ############ Metric 2 - Distance traveled (total)
            distance_traveled = distance_traveled + float_vect3_norm_two_points(&VDISTANCEPOSITIONwned, &VRwned);
            VDISTANCEPOSITIONwned.x = VRwned.x;
            VDISTANCEPOSITIONwned.y = VRwned.y;
            VDISTANCEPOSITIONwned.z = VRwned.z;

            // If the Goal is reached, set is_setpoint_reached_flag to 1 and record time
            if (is_setpoint_reached(&VGOALwned, &VR2dwned, threshold_distance_to_goal_direct)) {
              is_setpoint_reached_flag = 1;
              clock_total_time = clock() - clock_total_time_current;

            }
          }
        } break;


        // State 2 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 2: POSITION_GOAL
        case POSITION_GOAL: {
          printf("POSITION_GOAL = %d\n", POSITION_GOAL);
          printf("Total time to reach goal = %f\n", ((double)clock_total_time) / CLOCKS_PER_SEC);
          printf("Total distance_traveled = %f\n", distance_traveled);

        } break;


        // State 0 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 0
        default: {
          printf("default = %d\n", 0);
        }
      }  // Finite state machine - End


      printf("Time elapsed since start = %f\n", (((double)clock()) - ((double)clock_total_time_current)) / CLOCKS_PER_SEC);
      printf("distance_traveled = %f\n", distance_traveled);

    } break; // DIRECT_CONTROL - End


    // State 2 ################################################################################################################################################################### State 2: AUTONOMOUS_GUIDED
    case AUTONOMOUS_GUIDED: {
      printf("AUTONOMOUS_GUIDED = %d\n", AUTONOMOUS_GUIDED);

      // Checking is state was changed, if yes then all flags are reset and the is_state_changed_flag
      // is set to 1 for this cycle only. Else, the  is_state_changed_flag is set to 0 again;
      if (current_state != previous_state) {
        // Setting flag signifying that the state was changed
        is_state_changed_flag = 1;
        // Reset flags
        is_start_reached_flag = 0;        // Set to 1 if start position is reached, 0 otherwise
        is_setpoint_reached_flag = 0;     // Set to 1 if setpoint is reached, 0 otherwise
        is_obstacle_detected_flag = 0;      // Set to 1 if obstacle is detected, 0 otherwise
        is_path_free_flag = 0;          // Set to 1 if no obstacle is detected, 0 otherwise
        is_heading_reached_flag = 0;      // Set to 1 if heading is reached, 0 otherwise
        is_edge_found_macro_flag = 0;       // Set to 1 if best edge (according to macro confidence) was found, 0 otherwise
        is_edge_found_micro_flag = 0;       // Set to 1 if best edge (according to micro confidence) was found, 0 otherwise
        is_no_edge_found_flag = 0;        // Set to 1 if no edge was identified, 0 otherwise
        initial_heading.initiated = 0;      // 0 = it can be overwritten
        initial_heading.is_left_reached_flag = 0;// The scan has not reached the left maximum angle yet
        initial_heading.is_right_reached_flag = 0;// The scan has not reached the right maximum angle yet
        distance_robot_edge_goal = 99999; //
      } else if (is_state_changed_flag != 0) {
        is_state_changed_flag = 0;
      }

      // Background processes only happens if the current state is not POSITION_GOAL
      if (current_state != POSITION_GOAL) {
        // ############ Metric 1 - Recording current time
        clock_total_time_current = clock();
        //printf("clock_total_time_current = %f\n", (double)clock_total_time_current);
        // In case we are in the position start state and the state has changed, initialize clock_total_time_previous
        if ((current_state == MOVE_TO_GOAL) && is_mode_changed_flag) {
          printf("Metric 1 was started\n");
          //printf("clock_total_time_current set = %f\n", (double)clock_total_time_current);
          clock_total_time_previous = clock_total_time_current;
        }
        //Else check time difference to previous cycle and add to clock_total_time
        else {
          clock_total_time = clock_total_time + (clock_total_time_current - clock_total_time_previous);
          clock_total_time_previous = clock_total_time_current;
          //printf("clock_total_time_previous = %f\n", (double)clock_total_time_previous);
        }

        // ############ Metric 2 - Distance traveled (total)
        distance_traveled = distance_traveled + float_vect3_norm_two_points(&VDISTANCEPOSITIONwned, &VRwned);

        VDISTANCEPOSITIONwned.x = VRwned.x;
        VDISTANCEPOSITIONwned.y = VRwned.y;
        VDISTANCEPOSITIONwned.z = VRwned.z;

        // ############ Metric 3 - Runtime average of background processes (see below) - Start:
        clock_t time; // Creating variable to hold time (number of cycles)
        time = clock(); // Saving current time, way below it is used to calculate time spent in a cycle
        counter_cycles++; // Counting how many times a state was activated (needed for average calculation)

        // Background processes - Includes image processing for use
        //background_processes(save_images_flag);
        background_processes_16bit(save_images_flag);

        // ############ Metric 3 - Runtime average of background processes (see below) - End:
        clock_background_processes = clock_background_processes + (clock() - time);;

        // ############ Metric 4 - Runtime average per state - Start:
        clock_FSM = clock(); // Saving current time, way below it is used to calculate time spent in a cycle
        counter_state[current_state]++; // Counting how many times a state was activated (needed for average calculation). This is done here as state may change in FSM
      }

      // Initializing previous_state variable for next cycle
      // This happens here and not above as the metric above depends on the previous state
      previous_state = current_state;

      switch (current_state) { // Finite state machine - Start
        // State 1 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 1: MOVE_TO_GOAL
        case MOVE_TO_GOAL: {
          printf("MOVE_TO_GOAL = %d\n", MOVE_TO_GOAL);
          // 1. Moving robot towards the goal
          // Checking if goal is reached, if not continue to move
          if (is_setpoint_reached_flag) {
            printf("Goal is reached\n");
            set_state(POSITION_GOAL, allow_state_change_MOVE_TO_GOAL);

          }
          // Else, move to goal and check confidence that goal has been reached.
          else {
            // Sets setpoint to goal position and orientates drones towards the goal as well
            autopilot_guided_goto_ned(VGOALwned.x, VGOALwned.y, VGOALwned.z, heading_towards_waypoint(WP_GOAL));


            // If start appears to be reached increase confidence
            if (is_setpoint_reached(&VGOALwned, &VRwned, threshold_distance_to_goal)) {
              position_confidence++;
              Bound(position_confidence, 0, max_position_confidence);
            }

            // If the position_confidence is high enough, set is_start_reached_flag to 1 and reset position_confidence
            if (position_confidence == max_position_confidence) {
              is_setpoint_reached_flag = 1;
              position_confidence = 0;
            }
          }


          // 2. Check if obstacle is in way, when one is detected change state (dependent on negative flag from 1)
          // If the setpoint has not been reached, keep looking for obstacles
          if (!is_setpoint_reached_flag) {
            // Checking if obstacle is in way, if not continue checking for it
            if (is_obstacle_detected_flag) {
              printf("Object detected!!!!!!!!\n");
              // 1. Seting setpoint to current location
              guidance_h_hover_enter();
              set_state(WEDGEBUG_START, allow_state_change_MOVE_TO_GOAL);
            }
            // Else, check confidence that obstacle is there
            else { // This happens continuously, as long as the state is active and the goal has not been reached. It stops when the flag has been set below.
              // Calculate median depth in front
              median_depth_in_front = median_depth_to_point(&c_img_cropped, &img_depth_int16_cropped, &median_kernel16bit);
              float depth = median_depth_in_front / 100.00;
              printf("median_depth_in_front = %f\n", depth);
              printf("depth to goal = %f\n", VGOALc.z);


              // If obstacle appears to be detected AND its in front of goal point, increase obstacle_confidence
              if ((median_depth_in_front < threshold_median_depth)
                  && (depth < VGOALc.z)) { // NOTE. The first logical statement is in centimeters and the second in meters
                printf("Increasing confidence\n");
                obstacle_confidence++;
                Bound(obstacle_confidence, 0, max_obstacle_confidence);
                printf("obstacle_confidence = %d\n", obstacle_confidence);
                printf("max_obstacle_confidence = %d\n", max_obstacle_confidence);
              } else {
                obstacle_confidence = 0;
              }
              // If the obstacle_confidence is high enough, set is_obstacle_detected_flag to 1 and reset obstacle_confidence
              if (obstacle_confidence == max_obstacle_confidence) {
                is_obstacle_detected_flag = 1;
                obstacle_confidence = 0;
                // Setting holding point so that when state changes to wedgebug the drone stays at one spot, when looking for edges
                VHOLDINGPOINTwned.x = VRwned.x;
                VHOLDINGPOINTwned.y = VRwned.y;
                VHOLDINGPOINTwned.z = VRwned.z;
              }
            }
          }

          printf("position_confidence = %d\n", position_confidence);
          printf("obstacle_confidence = %d\n", obstacle_confidence);
        } break;


        // State 2 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 2: POSITION_GOAL
        case POSITION_GOAL: {
          printf("POSITION_GOAL = %d\n", POSITION_GOAL);
          // Since the drone is at the goal we will swithc bach to the NAV mode
          //autopilot_mode_auto2 = AP_MODE_NAV;
          //autopilot_static_set_mode(AP_MODE_NAV);

          printf("Total time to reach goal = %f\n", ((double)clock_total_time) / CLOCKS_PER_SEC);
          printf("Total distance_traveled = %f\n", distance_traveled);
          printf("Average runtime of background processes = %f\n",
                 (clock_background_processes / CLOCKS_PER_SEC / counter_cycles));

          for (uint8_t i = 0; i < number_of_states; i++) {
            printf("Average runtime of state %d = %f\n", i, (time_state[i] / CLOCKS_PER_SEC / counter_state[i]));
          }
        } break;


        // State 3 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 3: WEDGEBUG_START
        case WEDGEBUG_START: {
          printf("WEDGEBUG_START = %d\n", WEDGEBUG_START);
          /* Details:
           * 1) The robot checks if holding point was reached, at which it first detected that an obstacle was it its way
           * 2) If that holding point is reached, the robot then scans for any edges.
           *
           */

          // 1. Move robot towards holding point
          // Checking if holding point is reached, if not continue to move
          if (is_setpoint_reached_flag) {
            printf("Holding point is reached\n");
          }
          // Else, move to holding point and check confidence that holding point has been reached.
          else { // This happens continuously, as long as the state is active. It stops when the flag has been set below.

            // Sets setpoint to goal position and orientates drones towards the goal as well
            autopilot_guided_goto_ned(VHOLDINGPOINTwned.x, VHOLDINGPOINTwned.y, VHOLDINGPOINTwned.z,
                                      heading_towards_waypoint(WP_GOAL));

            // If holding point appears to be reached increase confidence
            if (is_setpoint_reached(&VHOLDINGPOINTwned, &VRwned, threshold_distance_to_goal)) {
              position_confidence++;
              Bound(position_confidence, 0, max_position_confidence);
            }

            // If the position_confidence is high enough, set is_setpoint_reached_flag to 1 and reset position_confidence
            if (position_confidence == max_position_confidence) {
              is_setpoint_reached_flag = 1;
              position_confidence = 0;
            }
          }


          // 2. Scan for edges and see if any exist or not, then change state (dependent on flag from 1).
          if (is_setpoint_reached_flag) {
            // a. Stopping drone movement (a second time, just to make sure)
            guidance_h_hover_enter();

            // b. Checking if edges are located
            // Running function to detect and save edge
            is_edge_found_micro_flag  =
              find_best_edge_coordinates2(
                &VEDGECOORDINATESc,
                &VGOALc,//target_point,
                &img_edges_int8_cropped,
                &img_depth_int16_cropped,
                &edge_search_area,
                threshold_depth_of_edges,
                max_edge_found_micro_confidence);


            //printf("was_edge_found?: %d\n", was_edge_found);


            // d. In this if-else statement we check if an edge was detected. If no edge is detected (by find_best_edge_coordinates) no_edge_found_confidence is
            // increased, whereas if an edge is found the edge_found_macro_confidence is increased. If one of the confidences reaches the
            // maximum the respective flag is turned on.
            // If obstacle is in way
            if (is_edge_found_macro_flag) {
              set_state(MOVE_TO_EDGE, allow_state_change_WEDGEBUG_START);
            }
            // Else if no edge is detected
            else if (is_no_edge_found_flag) {
              printf("Edge not found!!!!!!!!!!!!!!!!!\n");
              set_state(EDGE_SCAN, allow_state_change_WEDGEBUG_START);
            }
            // Else, check if micro edge has been detected (by find_best_edge_coordinates) or not and increase respective confidences.
            // Also, check if macro confidence of having detected an edge is reached or whether maximum confidence of having no edges
            // detected is reached. If any are reached, respective flags are set
            else {
              // This if statement increase the confidence
              // If edge is detected in find_best_edge_coordinates, increase edge_found_macro_confidence
              if (is_edge_found_micro_flag) {
                edge_found_macro_confidence++;
                Bound(edge_found_macro_confidence, 0, max_edge_found_macro_confidence);
              }
              // If no edge is detected in find_best_edge_coordinates, increase no_edge_found_confidence
              else {
                no_edge_found_confidence++;
                Bound(no_edge_found_confidence, 0, no_edge_found_confidence);
              }

              // If the edge_found_macro_confidence is high enough, set is_edge_found_macro_flag to 1 and reset edge_found_macro_confidence and no_edge_found_confidence
              if (edge_found_macro_confidence == max_edge_found_macro_confidence) {
                is_edge_found_macro_flag = 1;
                edge_found_macro_confidence = 0;
                no_edge_found_confidence = 0;

                // Since we are sure that a suitable edge has been found, the camera coordinates are transformed to
                // world coordinates and saved in a global variable, for used in subsequent state to make drone move
                // towards this point
                Vb_to_Va(&VEDGECOORDINATESr, &VEDGECOORDINATESc, &Rcr, &VCr);
                Vb_to_Va(&VEDGECOORDINATESwned, &VEDGECOORDINATESr, &Rrwned, &VRwned);
                Va_to_Vb(&VEDGECOORDINATESwenu, &VEDGECOORDINATESwned, &Rwnedwenu, &VNEDwenu);
                //printf("Optimal edge found\n");
                //printf("Camera coordinates of edge: [%f, %f, %f]\n", VEDGECOORDINATESc.x, VEDGECOORDINATESc.y, VEDGECOORDINATESc.z);
                // Saving best edge in global variable. // Note. VPBESTEDGECOORDINATESwned is used in the MOVE_TO_EDGE (7) and POSITION_EDGE (8) states
                VPBESTEDGECOORDINATESwned.x = VEDGECOORDINATESwned.x;
                VPBESTEDGECOORDINATESwned.y = VEDGECOORDINATESwned.y;
                VPBESTEDGECOORDINATESwned.z = VEDGECOORDINATESwned.z;

                // Making snapshot of image with edge coordinates highlighted. Comment out if not needed
                if (save_images_flag) {save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped_marked.bmp");}

              }
              // If the no_edge_found_macro_confidence is high enough, set is_no_edge_found_macro_flag to 1 and reset edge_found_macro_confidence and no_edge_found_confidence
              if (no_edge_found_confidence == max_no_edge_found_confidence) {
                is_no_edge_found_flag = 1;
                no_edge_found_confidence = 0;
                edge_found_macro_confidence = 0;
              }
            }
            printf("Edge found confidence = %d\n", edge_found_macro_confidence);
            printf("No Edge found confidence = %d\n", no_edge_found_confidence);
          }
        } break;


        // State 4 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 4: MOVE_TO_EDGE
        case MOVE_TO_EDGE: {
          printf("MOVE_TO_EDGE = %d\n", MOVE_TO_EDGE);

          //printf("Camera coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESc.x, VEDGECOORDINATESc.y, VEDGECOORDINATESc.z);
          //printf("Robot coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESr.x, VEDGECOORDINATESr.y, VEDGECOORDINATESr.z);
          printf("World NED coordinates of edge setpoint = [%f, %f, %f]\n", VPBESTEDGECOORDINATESwned.x,
                 VPBESTEDGECOORDINATESwned.y, VPBESTEDGECOORDINATESwned.z);
          //printf("World ENU coordinates of edge setpoint = [%f, %f, %f]\n", VEDGECOORDINATESwenu.x, VEDGECOORDINATESwenu.y, VEDGECOORDINATESwenu.z);


          // 1. Orientate robot towards object edge detected
          // If correct angle is reached, nothing happens
          if (is_heading_reached_flag) {
            printf("Heading is reached\n");
          }
          // Else, adjust heading and check confidence that heading is reached
          else { // This happens continuously, as long as the state is active. It stops when the flag has been set below
            // Set desired heading
            guidance_h_set_guided_heading(heading_towards_setpoint_WNED(
                                            &VPBESTEDGECOORDINATESwned));//   heading_towards_waypoint(WP_GOAL));

            // If heading appears to be reached increase confidence
            if (is_heading_reached(heading_towards_setpoint_WNED(&VPBESTEDGECOORDINATESwned), heading,
                                   threshold_distance_to_angle)) {
              heading_confidence++;
              Bound(heading_confidence, 0, max_heading_confidence);
            }

            // If the heading_confidence is high enough, set is_heading_reached_flag to 1 and reset heading_confidence
            if (heading_confidence == max_heading_confidence) {
              is_heading_reached_flag = 1;
              heading_confidence = 0;
            }
          }


          // 2.Move robot to edge detected (dependent on flag from 1)
          // If the robot faces the right direction, go to the object edge coordinates (NEW) detected
          if (is_heading_reached_flag) {
            // If edge setpoint is reached, stay at it
            if (is_setpoint_reached_flag) {
              printf("Edge is reached\n");
              autopilot_guided_goto_ned(VPBESTEDGECOORDINATESwned.x, VPBESTEDGECOORDINATESwned.y, VPBESTEDGECOORDINATESwned.z,
                                        heading_towards_waypoint(WP_GOAL));
            }
            // Else, move to edge setpoint and check confidence that edge setpoint is reached
            else { // This happens continuously, as long as the state is active. It stops when the flag has been set below.

              // Sets setpoint to edge position and orientates robot towards it as well
              guidance_h_set_guided_pos(VPBESTEDGECOORDINATESwned.x, VPBESTEDGECOORDINATESwned.y);
              guidance_v_set_guided_z(VPBESTEDGECOORDINATESwned.z);

              // If edge appears to be reached increase confidence
              if (is_setpoint_reached(&VPBESTEDGECOORDINATESwned, &VRwned,
                                      threshold_distance_to_goal)) { // && !is_setpoint_reached_flag)
                position_confidence++;
                Bound(position_confidence, 0, max_position_confidence);
              }

              // If the position_confidence is high enough, set is_setpoint_reached_flag to 1 and reset position_confidence
              if (position_confidence == max_position_confidence) {
                is_setpoint_reached_flag = 1;
                position_confidence = 0;
              }
            }
          }



          // 3. Set next state (dependent on 1 and 2)
          // If the robot faces the correct angle AND the edge is then reached, the state is changed to "MOVE_TO_GOAL"
          if (is_heading_reached_flag && is_setpoint_reached_flag) {
            printf("Position and Heading are reached\n");
            set_state(POSITION_EDGE, allow_state_change_MOVE_TO_EDGE);

          }
          printf("position_confidence = %d\n", position_confidence);
          printf("heading_confidence = %d\n", heading_confidence);
        } break;


        // State 5 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 5: POSITION_EDGE
        case POSITION_EDGE: {
          printf("POSITION_EDGE = %d\n", POSITION_EDGE);

          // This ensures that the robot stay on the edge at all times
          guidance_h_set_guided_pos(VPBESTEDGECOORDINATESwned.x, VPBESTEDGECOORDINATESwned.y);
          guidance_v_set_guided_z(VPBESTEDGECOORDINATESwned.z);

          //  1. Orientates robot towards the final goal
          // If robot faces goal, robot stays on current edge
          if (is_heading_reached_flag) {
            printf("Heading is reached\n");
            guidance_h_set_guided_heading(heading_towards_waypoint(WP_GOAL));
          }
          // Else, adjust heading and check confidence that heading is reached
          else { // This happens continuously, as long as the state is active. It stops when the flag has been set.
            // Set desired heading
            guidance_h_set_guided_heading(heading_towards_waypoint(WP_GOAL));

            // If heading appears to be reached increase confidence
            if (is_heading_reached(heading_towards_waypoint(WP_GOAL), heading, threshold_distance_to_angle)) {
              heading_confidence++;
              Bound(heading_confidence, 0, max_heading_confidence);
            }

            // If the heading_confidence is high enough, set is_heading_reached_flag to 1 and reset heading_confidence
            if (heading_confidence == max_heading_confidence) {
              is_heading_reached_flag = 1;
              heading_confidence = 0;
            }
          }


          // 2. Check if object is blocking the path or not and set next state (dependent on flag from 1)
          // If the robot faces the right direction, check if obstacle exists or not and change state
          if (is_heading_reached_flag) {
            // In this if-else statement we check if an obstacle is blocking the path or not. If no obstacle is blocking the path the free_path_confidence is
            // increased, whereas if an obstacle is blocking the path the obstacle_confidence is increased. If one of the confidences reaches the
            // maximum the respective flag is turned on. These statements only execute if the robot faces the goal (see above)
            // If obstacle is in way
            if (is_obstacle_detected_flag) {
              printf("Object detected!!!!!!!!\n");
              // Setting new holding point for WEDGEBUG_START state
              VHOLDINGPOINTwned.x = VRwned.x;
              VHOLDINGPOINTwned.y = VRwned.y;
              VHOLDINGPOINTwned.z = VRwned.z;
              set_state(WEDGEBUG_START, allow_state_change_POSITION_EDGE);
            }
            // Else if the path is clear
            else if (is_path_free_flag) {
              printf("Path is free\n");
              set_state(MOVE_TO_GOAL, allow_state_change_POSITION_EDGE);
            }
            // Else, check for obstacles and free path and check confidence that an obstacle is there or that the path is free
            else {
              // Calculate median depth in front
              median_depth_in_front = median_depth_to_point(&c_img_cropped, &img_depth_int16_cropped,
                                      &median_kernel16bit); // Median depth in centimeters (cm)
              printf("Median_depth_in_front (m) = %f\n", (median_depth_in_front / 100.00)); // Median depth in meters (m)
              printf("Depth to goal = %f\n", VGOALc.z); // Depth in meters (m)

              // This if-else statement increase the confidence
              // If obstacle appears to be detected AND its in front of goal point, increase obstacle_confidence
              if ((median_depth_in_front < threshold_median_depth)
                  && ((median_depth_in_front / 100.00) <
                      VGOALc.z)) { // NOTE. The first logical statement is in centimeters and the second in meters
                obstacle_confidence++;
                Bound(obstacle_confidence, 0, max_obstacle_confidence);
                free_path_confidence = 0;

              }
              // If obstacle appears not detected, increase free_path_confidence
              else {
                free_path_confidence++;
                Bound(free_path_confidence, 0, max_free_path_confidence);
                obstacle_confidence = 0;
              }
              // If the obstacle_confidence is high enough, set is_obstacle_detected_flag to 1 and reset obstacle_confidence and free_path_confidence
              if (obstacle_confidence == max_obstacle_confidence) {
                is_obstacle_detected_flag = 1;
                obstacle_confidence = 0;
                free_path_confidence = 0;
              }
              // If the free_path_confidence is high enough, set is_path_free_flag to 1 and reset free_path_confidence and obstacle_confidence
              if (free_path_confidence == max_free_path_confidence) {
                is_path_free_flag = 1;
                free_path_confidence = 0;
                obstacle_confidence = 0;
              }
            }

          }
          printf("Obstacle confidence = %d\n", obstacle_confidence);
          printf("Free path confidence = %d\n", free_path_confidence);
        } break;


        // State 6 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 6: EDGE_SCAN
        case EDGE_SCAN: {
          printf("EDGE_SCAN = %d\n", EDGE_SCAN);

          // 1. The robot is tasked to stay at the holding point for the entire duration of this state
          // Making drone hover, so that it does not drift from its current position
          guidance_h_set_guided_pos(VHOLDINGPOINTwned.x, VHOLDINGPOINTwned.y); guidance_v_set_guided_z(VHOLDINGPOINTwned.z);

          // 2. Checking if edges are located
          // Running function to detect and save edge
          is_edge_found_micro_flag  =
            find_best_edge_coordinates2(
              &VEDGECOORDINATESc,
              &VGOALc,//target_point,
              &img_edges_int8_cropped,
              &img_depth_int16_cropped,
              &edge_search_area,
              threshold_depth_of_edges,
              max_edge_found_micro_confidence);


          // 3. The identified best edge is compared to the previous identified best edge, and replace if the total distance (robot-edge-goal)
          // has decreased (dependent on flag from 2)
          // If edge was found
          if (is_edge_found_micro_flag) {
            // First we convert edge camera coordinates into world coordinates (ned and enu)
            //VEDGECOORDINATESc.z = VEDGECOORDINATESc.z - 0.25;// add this to make sure drone does not exactly stop on the edge point identified (it might crash otherwise)
            Vb_to_Va(&VEDGECOORDINATESr, &VEDGECOORDINATESc, &Rcr, &VCr);
            Vb_to_Va(&VEDGECOORDINATESwned, &VEDGECOORDINATESr, &Rrwned, &VRwned);
            Va_to_Vb(&VEDGECOORDINATESwenu, &VEDGECOORDINATESwned, &Rwnedwenu, &VNEDwenu);

            // Second, we increase confidence that an edge was spotted
            edge_found_macro_confidence++;

            // Third, we calculate the distance of the edge currently spotted, if distance is smaller
            // than previous distance VPBESTEDGECOORDINATESwned is updated with the coordinates
            // Note. VPBESTEDGECOORDINATESwned is used in the MOVE_TO_EDGE (7) and POSITION_EDGE (8) states
            // Calculating Euclidean distance (N2) - Edge to goal
            float f_distance_edge_to_goal =  float_vect3_norm_two_points(&VGOALwned, &VEDGECOORDINATESwned);
            // Calculating Euclidean distance (N2) - robot to edge
            float f_distance_robot_to_edge =  float_vect3_norm_two_points(&VEDGECOORDINATESwned, &VRwned);
            float distance_total = f_distance_robot_to_edge + f_distance_edge_to_goal;
            printf("distance_total  =%f\n", distance_total);
            if (distance_robot_edge_goal > distance_total) {
              distance_robot_edge_goal = distance_total;
              VPBESTEDGECOORDINATESwned.x = VEDGECOORDINATESwned.x;
              VPBESTEDGECOORDINATESwned.y = VEDGECOORDINATESwned.y;
              VPBESTEDGECOORDINATESwned.z = VEDGECOORDINATESwned.z;
            }
          }

          printf("Current minimum distance =%f\n", distance_robot_edge_goal);
          printf("is_edge_found_micro_flag = %d\n", is_edge_found_micro_flag);


          // 4. Initializing current heading parameters (if they have not been initialized before)
          if (initial_heading.initiated == 0) {
            initial_heading.heading_initial = heading;
            initial_heading.heading_max_left = (heading - max_edge_search_angle) + (WEDGEBUG_HFOV /
                                               2); // this way, when the the center of the drone is facing left, its fov does not exceed the maximum angle to the left
            FLOAT_ANGLE_NORMALIZE(initial_heading.heading_max_left);
            initial_heading.heading_max_right = (heading + max_edge_search_angle) - (WEDGEBUG_HFOV /
                                                2);  // this way, when the the center of the drone is facing right, its fov does not exceed the maximum angle to the right
            FLOAT_ANGLE_NORMALIZE(initial_heading.heading_max_right);
            initial_heading.initiated = 1;
          }


          // 5. The drone turns right to find any edges
          // Code for looking left
          // If left heading has been reached (i.e. if the flag is activated)
          if (initial_heading.is_left_reached_flag) {
            printf("Left heading is/was reached\n");
          }
          // Else,  adjust angle to to face more left and check confidence that left heading has been reached
          else { // This happens continuously, as long as the state is active. It stops when the flag has been set below
            // Set heading to maximum left heading
            guidance_h_set_guided_heading(initial_heading.heading_max_left);

            // If heading appears to be reached increase confidence
            if (is_heading_reached(initial_heading.heading_max_left, heading, threshold_distance_to_angle)) {
              heading_confidence++;
              Bound(heading_confidence, 0, max_heading_confidence);
            }
            // If the heading_confidence is high enough, set initial_heading.is_left_reached_flag to 1 and reset heading_confidence
            if (heading_confidence == max_heading_confidence) {
              initial_heading.is_left_reached_flag = 1;
              heading_confidence = 0;
            }
          }



          // 6. The drone turns right to find any edges (dependent on flag from 5)
          // Code for looking right - Only runs if robot has previously looked left (initial_heading.heading_max_left = 1)
          if (initial_heading.is_left_reached_flag) {
            // If right heading has been reached (i.e. if the flag is activated)
            if (initial_heading.is_right_reached_flag) {
              printf("Right heading is/was reached\n");
            }
            // Else,  adjust angle to to face more right and check confidence that right heading has been reached
            else {
              // Set heading to maximum left heading
              guidance_h_set_guided_heading(initial_heading.heading_max_right);

              // If heading appears to be reached increase confidence
              if (is_heading_reached(initial_heading.heading_max_right, heading, threshold_distance_to_angle)) {
                heading_confidence++;
                Bound(heading_confidence, 0, max_heading_confidence);
              }
              // If the heading_confidence is high enough, set initial_heading.is_right_reached_flag to 1 and resets heading_confidence
              if (heading_confidence == max_heading_confidence) {
                initial_heading.is_right_reached_flag = 1;
                heading_confidence = 0;
              }
            }
          }


          // 7. Check if confidence of having found a suitable edge is large enough or not and set next state (dependent on flags from 5 and 6)
          if (initial_heading.is_left_reached_flag && initial_heading.is_right_reached_flag) {
            if (is_edge_found_macro_flag) {
              printf("Edge has been found\n");
              set_state(MOVE_TO_EDGE, allow_state_change_EDGE_SCAN);
            } else if (is_no_edge_found_flag) {
              printf("Minimum has been encountered\n");
            } else {

              // If edge_found_macro_confidence is >= max_edge_found_macro_confidence then set is_edge_found_macro_flag
              // and reset edge_found_macro_confidence
              if (edge_found_macro_confidence >= max_edge_found_macro_confidence) {
                is_edge_found_macro_flag = 1;
                edge_found_macro_confidence = 0;
              }
              // Else, set is_no_edge_found_flag and reset edge_found_macro_confidence
              else {
                is_no_edge_found_flag = 1;
                edge_found_macro_confidence = 0;
              }

            }
          }

          printf("heading_confidence = %d\n", heading_confidence);
          printf("edge_found_confidence = %d\n", edge_found_macro_confidence);
        } break;

        // State 0 --------------------------------------------------------------------------------------------------------------------------------------------------------------- State 0
        default: {
          printf("default = %d\n", 0);
        }
        break;

      } // Finite state machine - End



      // ############ Metric 4 - Runtime average per state - End:
      clock_FSM = clock() - clock_FSM; // Calculating time it took for the FSM to finish running
      time_state[current_state] += ((double)clock_FSM); // Adding calculated time to total time of current state

      printf("Time elapsed since start = %f\n", ((double)clock_total_time) / CLOCKS_PER_SEC);
      printf("distance_traveled = %f\n", distance_traveled);
    } break; // AUTONOMOUS_GUIDED - End


    // State 3 ################################################################################################################################################################### State 3: AUTONOMOUS_NAV
    case AUTONOMOUS_NAV: {
      printf("AUTONOMOUS_NAV = %d\n", AUTONOMOUS_NAV);
      // Background processes - Includes image processing for use
      background_processes_16bit(save_images_flag);


      median_depth_in_front = median_depth_to_point(&c_img_cropped, &img_depth_int16_cropped, &median_kernel16bit);
      float depth = median_depth_in_front / 100.00;
      printf("median_depth_in_front = %f\n", depth);
      printf("depth to goal = %f\n", VGOALc.z);


      if ((median_depth_in_front < threshold_median_depth) && (depth < VGOALc.z)) {
        printf("Obstacle is in front of goal\n");
      } else {
        printf("The path to the goal is free!\n");
      }


      is_edge_found_micro_flag  =
        find_best_edge_coordinates2(
          &VEDGECOORDINATESc,
          &VGOALc,//target_point,
          &img_edges_int8_cropped,
          &img_depth_int16_cropped,
          &edge_search_area,
          threshold_depth_of_edges,
          max_edge_found_micro_confidence);


//      // c. Checking if edges are located
//      // Running function to detect and save edge
//      is_edge_found_micro_flag  =
//          find_best_edge_coordinates(
//          &VEDGECOORDINATESc,
//          &VGOALc,//target_point,
//          &img_edges_int8_cropped,
//          &img_middle_int8_cropped,
//          &edge_search_area,
//          threshold_disparity_of_edges,
//          max_edge_found_micro_confidence);
//



      if (save_images_flag) {save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped_marked.bmp");}



    } break; // AUTONOMOUS_NAV - End



    // State 0 ################################################################################################################################################################### State 0
    default:
    {printf("Unsupported control mode");} break;

  }  // Control mode state machine - End




  /*
    // printing flags
  printf("is_start_reached_flag = %d\n", is_start_reached_flag);
  printf("is_setpoint_reached_flag = %d\n", is_setpoint_reached_flag);
  printf("is_obstacle_detected_flag = %d\n", is_obstacle_detected_flag);
  printf("is_path_free_flag = %d\n", is_path_free_flag);
  printf("is_heading_reached_flag = %d\n", is_heading_reached_flag);
  printf("is_state_changed_flag = %d\n", is_state_changed_flag);
  printf("initial_heading.initiated = %d\n", initial_heading.initiated);
  printf("initial_heading.is_left_reached_flag = %d\n", initial_heading.is_left_reached_flag);
  printf("initial_heading.is_right_reached_flag = %d\n", initial_heading.is_right_reached_flag);

  */
  printf("\n\n");



  save_image_gray(&img_left_int8, "/home/dureade/Documents/paparazzi_images/img_left_int8.bmp");
  save_image_gray(&img_right_int8, "/home/dureade/Documents/paparazzi_images/img_right_int8.bmp");
  save_image_HM(&img_disparity_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_disparity_int8_cropped.bmp",
                heat_map_type);
  //save_image_gray(&img_left_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_left_int8_cropped.bmp");
  save_image_HM(&img_middle_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_intermediate_int8_cropped.bmp",
                heat_map_type);
  save_image_gray(&img_edges_int8_cropped, "/home/dureade/Documents/paparazzi_images/img_edges_int8_cropped.bmp");



  /*
  // Size of variables
  printf("img_left = %d\n", img_left.buf_size);
  printf("img_right = %d\n", img_right.buf_size);
  printf("img_left_int8 = %d\n", img_left_int8.buf_size);
  printf("img_left_int8_cropped = %d\n", img_left_int8_cropped.buf_size);
  printf("img_right_int8 = %d\n", img_right_int8.buf_size);
  printf("img_disparity_int8_cropped = %d\n", img_disparity_int8_cropped.buf_size);
  printf("img_edges_int8_cropped = %d\n", img_edges_int8_cropped.buf_size);
  printf("median_kernel = %d\n", median_kernel.buf_size);
  printf("SE_opening_OCV = %lu\n", sizeof(SE_opening_OCV));
  printf("VSTARTwenu.x = %lu\n", sizeof(VSTARTwenu.x));
  printf("current_state = %lu\n", sizeof(current_state));
  printf("current_mode = %lu\n", sizeof(current_mode));
  */








  /*
   * Heat maps:
   * 0 = COLORMAP_AUTUMN
   * 1 = COLORMAP_BONE
   * 2 = COLORMAP_JET
   * 3 = COLORMAP_WINTER
   * 4 = COLORMAP_RAINBOW
   * 5 = COLORMAP_OCEAN
   * 6 = COLORMAP_SUMMER
   * 7 = COLORMAP_SPRING
   * 8 = COLORMAP_COOL
   * 9 = COLORMAP_HSV
   * 10 = COLORMAP_PINK
   * 11 = COLORMAP_HOT
   */


}



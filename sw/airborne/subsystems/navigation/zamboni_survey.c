#include "zamboni_survey.h"

#include "subsystems/nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

int counter;
/**
variables used to store values from the flight plan
**/
float x_wp_center, y_wp_center;
float x_wp_dir, y_wp_dir;
float z_sweep_length;
float z_sweep_spacing;
int z_sweep_lines;
float z_shot_dist;
float z_altitude;

/**
static variables, used for initial calculations
**/
// properties for the filightpattern
float flight_angle, zamboni_return_angle;
float dx_sweep_width, dy_sweep_width;
float dx_flightline, dy_flightline;
float dx_flight_vec, dy_flight_vec;
float turnradius1, turnradius2;
int laps;

// points for navigation
float x_seg_start, y_seg_start;
float x_seg_end, y_seg_end;
float x_turn_center1, y_turn_center1;
float x_turn_center2, y_turn_center2;
float x_ret_start, y_ret_start;
float x_ret_end, y_ret_end;

// variables used for initial calculations
  float dx_flight_wpts, dy_flight_wpts;
  float len;

// constant for storing value for pre-leave-angle, (leave turncircles a small angle before the 180deg turns are compleated to get a smoother transition to flight-lines)
  int pre_leave_angle=2;

z_survey_stage z_stage;
/*
z_stage starts at ENTRY and than circles trought the other
states until to rectangle is completely covered
ENTRY : getting in the right position and height for the first flyover
SEG   : fly from seg_start to seg_end and take pictures,
        then calculate navigation points of next flyover
TURN1 : do a 180° turn around seg_center1
RET   : fly from ret_start to ret_end
TURN2 : do a 180° turn around seg_center2
*/

/**
 initializes the variables needed for the survey to start
   wp_center     :  the waypoint defining the center of the survey-rectangle
   wp_dir        :  the waypoint defining the orientation of the survey-rectangle
   sweep_length  :  the length of the survey-rectangle
   sweep_spacing :  distance between the sweeps
   sweep_lines   :  number of sweep_lines to fly
   altitude      :  the altitude that must be reached before the flyover starts
**/
bool_t init_zamboni_survey(uint8_t center_wp, uint8_t dir_wp, float sweep_length, float sweep_spacing, int sweep_lines, float altitude)
{
  counter = 0;
  // copy variables from the flight plan
  x_wp_center = waypoints[center_wp].x;
  y_wp_center = waypoints[center_wp].y;
  x_wp_dir = waypoints[dir_wp].x;
  y_wp_dir = waypoints[dir_wp].y;
  z_sweep_length = sweep_length;
  z_sweep_spacing = sweep_spacing;
  z_sweep_lines = sweep_lines;
  //z_shot_dist = shot_dist;
  z_altitude = altitude;

  // if turning right leave circle before angle is reached, if turning left - leave after
  if (z_sweep_spacing>0) pre_leave_angle -= pre_leave_angle;

  // calculate the flight_angle
  dx_flight_wpts = x_wp_dir - x_wp_center;
  dy_flight_wpts = y_wp_dir - y_wp_center;
  if (dy_flight_wpts == 0) dy_flight_wpts = 0.01; // to avoid dividing by zero
  flight_angle = 180*atan2(dx_flight_wpts, dy_flight_wpts)/M_PI;
  zamboni_return_angle = flight_angle + 180;
  if (zamboni_return_angle > 359) zamboni_return_angle -= 360;

  // calculate the flightline vector from start to end of one flightline, (delta x and delta y for one flightline)
  // (used later to move the flight pattern one flightline up for each round)
  len = sqrtf(dx_flight_wpts * dx_flight_wpts + dy_flight_wpts * dy_flight_wpts);
  dx_flight_vec = dx_flight_wpts/len;
  dy_flight_vec = dy_flight_wpts/len;
  dx_flightline = dx_flight_vec * z_sweep_length;
  dy_flightline = dy_flight_vec * z_sweep_length;

  // calculate the vector from one flightline perpendicular to the next flightline left, seen in the flightdirection. (Delta x and delta y betwen two adjecent flightlines)
  // (used later to move the flight pattern one flightline up for each round)
  dx_sweep_width = -(dy_flight_wpts/len) * z_sweep_spacing;
  dy_sweep_width = +(dx_flight_wpts/len) * z_sweep_spacing;

  // calculate number of laps to fly and turning radius for each end
  laps = (z_sweep_lines+1)/2;
  turnradius1 = (laps-1) * z_sweep_spacing * 0.5;
  turnradius2 = laps * z_sweep_spacing * 0.5;

  //CALCULATE THE NAVIGATION POINTS
  //start and end of flight-line in flight-direction
  x_seg_start = x_wp_center - dx_flightline * 0.5;
  y_seg_start = y_wp_center - dy_flightline * 0.5;
  x_seg_end = x_wp_center + dx_flightline * 0.5;
  y_seg_end = y_wp_center + dy_flightline * 0.5;

  //start and end of flight-line in return-direction
  x_ret_start = x_seg_end - dx_sweep_width * (laps-1);
  y_ret_start = y_seg_end - dy_sweep_width * (laps-1);
  x_ret_end = x_seg_start - dx_sweep_width * (laps-1);
  y_ret_end = y_seg_start - dy_sweep_width * (laps-1);

  //turn-centers at both ends
  x_turn_center1 = (x_seg_end + x_ret_start)/2;
  y_turn_center1 = (y_seg_end + y_ret_start)/2;
  x_turn_center2 = (x_seg_start + x_ret_end + dx_sweep_width) / 2;
  y_turn_center2 = (y_seg_start + y_ret_end + dy_sweep_width) / 2;

  //fast climbing to desired altitude
  NavVerticalAutoThrottleMode(100.0);
  NavVerticalAltitudeMode(z_altitude, 0.0);

  z_stage = Z_ENTRY;

  return FALSE;
}

/**
   main navigation routine. This is called periodically evaluates the current
   Position and stage and navigates accordingly.
   Returns True until the survey is finished
**/
bool_t zamboni_survey(void)
{
  // retain altitude
  NavVerticalAutoThrottleMode(0.0);
  NavVerticalAltitudeMode(z_altitude, 0.0);

  //go from center of field to end of field - (before starting the syrvey)
  if (z_stage == Z_ENTRY) {
    nav_route_xy(x_wp_center, y_wp_center, x_seg_end, y_seg_end);
    if (nav_approaching_xy(x_seg_end, y_seg_end, x_wp_center, y_wp_center, CARROT)) {
      z_stage = Z_TURN1;
      NavVerticalAutoThrottleMode(0.0);
      nav_init_stage();
    }
  }

  //Turn from stage to return
  else if (z_stage == Z_TURN1) {
    nav_circle_XY(x_turn_center1, y_turn_center1, turnradius1);
    if (NavCourseCloseTo(zamboni_return_angle+pre_leave_angle)){
        // && nav_approaching_xy(x_seg_end, y_seg_end, x_seg_start, y_seg_start, CARROT
      //calculate SEG-points for the next flyover
      x_seg_start = x_seg_start + dx_sweep_width;
      y_seg_start = y_seg_start + dy_sweep_width;
      x_seg_end = x_seg_end + dx_sweep_width;
      y_seg_end = y_seg_end + dy_sweep_width;

      z_stage = Z_RET;
      nav_init_stage();
      #ifdef DIGITAL_CAM
        //dc_survey(z_shot_dist, x_ret_start - dx_flight_vec * z_shot_dist, y_ret_start - dy_flight_vec * z_shot_dist);
        LINE_START_FUNCTION;
      #endif
    }
  }

  //fly the segment until seg_end is reached
  else if (z_stage == Z_RET) {
    nav_route_xy(x_ret_start, y_ret_start, x_ret_end, y_ret_end);
    if (nav_approaching_xy(x_ret_end, y_ret_end, x_ret_start, y_ret_start, 0)) {
      counter = counter + 1;
      #ifdef DIGITAL_CAM
        //dc_stop();
        LINE_STOP_FUNCTION;
      #endif
      z_stage = Z_TURN2;
    }
  }

  //turn from stage to return
  else if (z_stage == Z_TURN2) {
    nav_circle_XY(x_turn_center2, y_turn_center2, turnradius2);
    if (NavCourseCloseTo(flight_angle+pre_leave_angle)) {
      //counter = counter + 1;
      z_stage = Z_SEG;
      nav_init_stage();
      #ifdef DIGITAL_CAM
        //dc_survey(z_shot_dist, x_seg_start + dx_flight_vec * z_shot_dist, y_seg_start + dy_flight_vec * z_shot_dist);
        LINE_START_FUNCTION;
      #endif
    }

  //return
  } else if (z_stage == Z_SEG) {
    nav_route_xy(x_seg_start, y_seg_start, x_seg_end, y_seg_end);
    if (nav_approaching_xy(x_seg_end, y_seg_end, x_seg_start, y_seg_start, 0)) {

      // calculate the rest of the points for the next fly-over
      x_ret_start = x_ret_start + dx_sweep_width;
      y_ret_start = y_ret_start + dy_sweep_width;
      x_ret_end = x_ret_end + dx_sweep_width;
      y_ret_end = y_ret_end + dy_sweep_width;
      x_turn_center1 = (x_seg_end + x_ret_start)/2;
      y_turn_center1 = (y_seg_end + y_ret_start)/2;
      x_turn_center2 = (x_seg_start + x_ret_end + dx_sweep_width) / 2;
      y_turn_center2 = (y_seg_start + y_ret_end + dy_sweep_width) / 2;

      z_stage = Z_TURN1;
      nav_init_stage();
      #ifdef DIGITAL_CAM
        //dc_stop();
        LINE_STOP_FUNCTION;
      #endif
    }
  }
  if (counter >= laps) {
    #ifdef DIGITAL_CAM
    LINE_STOP_FUNCTION;
    #endif
    return FALSE;
  }
  else {
    return TRUE;
  }
}

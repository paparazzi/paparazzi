#include "poly_survey_adv.h"

#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif


/**
The following variables are set by poly_survey_init and not changed later on
**/

// precomputed vectors to ease calculations
point2d dir_vec;
point2d sweep_vec;
point2d rad_vec;

//the polygon from the flightplan
uint8_t poly_first;
uint8_t poly_count;

//desired properties of the flyover
float psa_min_rad;
float psa_sweep_width;
float psa_shot_dist;
float psa_altitude;

//direction for the flyover (0° == N)
int segment_angle;
int return_angle;

/**
The Following variables are dynamic, changed while navigating.
**/

/*
psa_stage starts at ENTRY and than circles trought the other
states until to polygon is completely covered
ENTRY : getting in the right position and height for the first flyover
SEG   : fly from seg_start to seg_end and take pictures,
        then calculate navigation points of next flyover
TURN1 : do a 180° turn around seg_center1
RET   : fly from ret_start to ret_end
TURN2 : do a 180° turn around seg_center2
*/
survey_stage psa_stage;

// points for navigation
point2d seg_start;
point2d seg_end;
point2d seg_center1;
point2d seg_center2;
point2d entry_center;
point2d ret_start;
point2d ret_end;


//helper functions and macro
#define VEC_CALC(A, B, C, OP) A.x = B.x OP C.x; A.y = B.y OP C.y;

static point2d vec_add(point2d a, point2d b)
{
  point2d tmp;
  VEC_CALC(tmp, a, b, +);

  return tmp;
}

static void nav_points(point2d start, point2d end)
{
  nav_route_xy(start.x, start.y, end.x, end.y);
}

/**
   intercept two lines and give back the point of intersection
   returns        : FALSE if no intersection can be found or intersection does not lie between points a and b
   else TRUE
   p              : returns intersection
   x, y           : first line is defined by point x and y (goes through this points)
   a1, a2, b1, b2 : second line by coordinates a1/a2, b1/b2
**/
static bool_t intercept_two_lines(point2d *p, point2d x, point2d y, float a1, float a2, float b1, float b2)
{
  float div, fac;

  div = (((b2 - a2)*(y.x - x.x)) + ((x.y - y.y)*(b1 - a1)));
  if (div == 0) return FALSE;
  fac = ((y.x*(x.y - a2)) + (x.x*(a2 - y.y)) + (a1*(y.y - x.y))) / div;
  if (fac > 1.0) return FALSE;
  if (fac < 0.0) return FALSE;

  p->x = a1 + fac*(b1 - a1);
  p->y = a2 + fac*(b2 - a2);

  return TRUE;
}

/**
   intersects a line with the polygon and gives back the two intersection points
   returns : TRUE if two intersection can be found, else FALSE
   x, y    : intersection points
   a, b    : define the line to intersection
**/
static bool_t get_two_intersects(point2d *x, point2d *y, point2d a, point2d b)
{
  int i, count = 0;
  point2d tmp;

  for (i=0;i<poly_count-1;i++)
    if(intercept_two_lines(&tmp,a,b,waypoints[poly_first+i].x,waypoints[poly_first+i].y,waypoints[poly_first+i+1].x,waypoints[poly_first+i+1].y)) {
      if (count == 0) {
        *x = tmp;
        count++;
      }
      else {
        *y = tmp;
        count++;
        break;
      }
    }

  //wrapover first,last polygon waypoint
  if (count == 1 && intercept_two_lines(&tmp,a,b,waypoints[poly_first+poly_count-1].x,waypoints[poly_first+poly_count-1].y,waypoints[poly_first].x,waypoints[poly_first].y)) {
    *y = tmp;
    count++;
  }

  if (count != 2)
    return FALSE;

  //change points
  if (fabs(dir_vec.x) > fabs(dir_vec.y)) {
    if ((y->x - x->x) / dir_vec.x < 0.0){
      tmp = *x;
      *x = *y;
      *y = tmp;
    }
  }
  else
    if ((y->y - x->y) / dir_vec.y < 0.0) {
      tmp = *x;
      *x = *y;
      *y = tmp;
    }

  return TRUE;
}

/**
   initializes the variables needed for the survey to start
   first_wp    :  the first Waypoint of the polygon
   size        :  the number of points that make up the polygon
   angle       :  angle in which to do the flyovers
   sweep_width :  distance between the sweeps
   shot_dist   :  distance between the shots
   min_rad     :  minimal radius when navigating
   altitude    :  the altitude that must be reached before the flyover starts
**/
bool_t init_poly_survey_adv(uint8_t first_wp, uint8_t size, float angle, float sweep_width, float shot_dist, float min_rad, float altitude)
{
  int i;
  point2d small, sweep;
  float div, len, angle_rad = angle/180.0*M_PI;

  if (angle < 0.0) angle += 360.0;
  if (angle >= 360.0) angle -= 360.0;

  poly_first = first_wp;
  poly_count = size;

  psa_sweep_width = sweep_width;
  psa_min_rad = min_rad;
  psa_shot_dist = shot_dist;
  psa_altitude = altitude;

  segment_angle = angle;
  return_angle = angle+180;
  if (return_angle > 359) return_angle -= 360;

  if (angle <= 45.0 || angle >= 315.0) {
    //north
    dir_vec.y = 1.0;
    dir_vec.x = 1.0*tanf(angle_rad);
    sweep.x = 1.0;
    sweep.y = - dir_vec.x / dir_vec.y;
  }
  else if (angle <= 135.0) {
    //east
    dir_vec.x = 1.0;
    dir_vec.y = 1.0/tanf(angle_rad);
    sweep.y = - 1.0;
    sweep.x = dir_vec.y / dir_vec.x;
  }
  else if (angle <= 225.0) {
    //south
    dir_vec.y = -1.0;
    dir_vec.x = -1.0*tanf(angle_rad);
    sweep.x = -1.0;
    sweep.y = dir_vec.x / dir_vec.y;
  }
  else {
    //west
    dir_vec.x = -1.0;
    dir_vec.y = -1.0/tanf(angle_rad);
    sweep.y = 1.0;
    sweep.x = - dir_vec.y / dir_vec.x;
  }

  //normalize
  len = sqrt(sweep.x*sweep.x+sweep.y*sweep.y);
  sweep.x = sweep.x / len;
  sweep.y = sweep.y / len;

  rad_vec.x = sweep.x * psa_min_rad;
  rad_vec.y = sweep.y * psa_min_rad;
  sweep_vec.x = sweep.x * psa_sweep_width;
  sweep_vec.y = sweep.y * psa_sweep_width;

  //begin at leftmost position (relative to dir_vec)
  small.x = waypoints[poly_first].x;
  small.y = waypoints[poly_first].y;

  div = (sweep_vec.y*dir_vec.x) - (sweep_vec.x*dir_vec.y);

  //cacluate the leftmost point if one sees the dir vec as going "up" and the sweep vec as going right
  if (div < 0.0) {
    for(i=1;i<poly_count;i++)
      if ((dir_vec.x*(waypoints[poly_first+i].y - small.y)) + (dir_vec.y*(small.x - waypoints[poly_first+i].x)) > 0.0) {
        small.x = waypoints[poly_first+i].x;
        small.y = waypoints[poly_first+i].y;
      }
  }
  else
    for(i=1;i<poly_count;i++)
      if ((dir_vec.x*(waypoints[poly_first+i].y - small.y)) + (dir_vec.y*(small.x - waypoints[poly_first+i].x)) > 0.0) {
        small.x = waypoints[poly_first+i].x;
        small.y = waypoints[poly_first+i].y;
      }

  //calculate the line the defines the first flyover
  seg_start.x = small.x + 0.5*sweep_vec.x;
  seg_start.y = small.y + 0.5*sweep_vec.y;
  VEC_CALC(seg_end, seg_start, dir_vec, +);

  if (!get_two_intersects(&seg_start, &seg_end, seg_start, seg_end)) {
    psa_stage = ERR;
    return FALSE;
  }

  //center of the entry circle
  entry_center.x = seg_start.x - rad_vec.x;
  entry_center.y = seg_start.y - rad_vec.y;

  //fast climbing to desired altitude
  NavVerticalAutoThrottleMode(100.0);
  NavVerticalAltitudeMode(psa_altitude, 0.0);

  psa_stage = ENTRY;

  return FALSE;
}

/**
   main navigation routine. This is called periodically evaluates the current
   Position and stage and navigates accordingly.
   Returns True until the survey is finished
**/
bool_t poly_survey_adv(void)
{
  //entry circle around entry-center until the desired altitude is reached
  if (psa_stage == ENTRY) {
    nav_circle_XY(entry_center.x, entry_center.y, -psa_min_rad);
    if (NavCourseCloseTo(segment_angle)
        && nav_approaching_xy(seg_start.x, seg_start.y, last_x, last_y, CARROT)
        && fabs(estimator_z - psa_altitude) <= 20) {
      psa_stage = SEG;
      NavVerticalAutoThrottleMode(0.0);
      nav_init_stage();
#ifdef DIGITAL_CAM
      dc_survey(psa_shot_dist, seg_start.x - dir_vec.x*psa_shot_dist*0.5, seg_start.y - dir_vec.y*psa_shot_dist*0.5);
#endif
    }
  }
  //fly the segment until seg_end is reached
  if (psa_stage == SEG) {
    nav_points(seg_start, seg_end);
    //calculate all needed points for the next flyover
    if (nav_approaching_xy(seg_end.x, seg_end.y, seg_start.x, seg_start.y, 0)) {
#ifdef DIGITAL_CAM
      dc_stop();
#endif
      VEC_CALC(seg_center1, seg_end, rad_vec, -);
      ret_start.x = seg_end.x - 2*rad_vec.x;
      ret_start.y = seg_end.y - 2*rad_vec.y;

      //if we get no intersection the survey is finished
      if (!get_two_intersects(&seg_start, &seg_end, vec_add(seg_start, sweep_vec), vec_add(seg_end, sweep_vec)))
        return FALSE;

      ret_end.x = seg_start.x - sweep_vec.x - 2*rad_vec.x;
      ret_end.y = seg_start.y - sweep_vec.y - 2*rad_vec.y;

      seg_center2.x = seg_start.x - 0.5*(2.0*rad_vec.x+sweep_vec.x);
      seg_center2.y = seg_start.y - 0.5*(2.0*rad_vec.y+sweep_vec.y);

      psa_stage = TURN1;
      nav_init_stage();
    }
  }
  //turn from stage to return
  else if (psa_stage == TURN1) {
    nav_circle_XY(seg_center1.x, seg_center1.y, -psa_min_rad);
    if (NavCourseCloseTo(return_angle)) {
      psa_stage = RET;
      nav_init_stage();
    }
    //return
  } else if (psa_stage == RET) {
    nav_points(ret_start, ret_end);
    if (nav_approaching_xy(ret_end.x, ret_end.y, ret_start.x, ret_start.y, 0)) {
      psa_stage = TURN2;
      nav_init_stage();
    }
    //turn from return to stage
  } else if (psa_stage == TURN2) {
    nav_circle_XY(seg_center2.x, seg_center2.y, -(2*psa_min_rad+psa_sweep_width)*0.5);
    if (NavCourseCloseTo(segment_angle)) {
      psa_stage = SEG;
      nav_init_stage();
#ifdef DIGITAL_CAM
      dc_survey(psa_shot_dist, seg_start.x - dir_vec.x*psa_shot_dist*0.5, seg_start.y - dir_vec.y*psa_shot_dist*0.5);
#endif
    }
  }

  return TRUE;
}

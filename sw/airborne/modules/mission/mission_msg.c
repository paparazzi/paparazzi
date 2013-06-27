/** \file mission_msg.c
 *  \brief the new messages for mission planner library
 *  \ edited by Anh Truong
 */

//#define MISSION_C


#include "modules/mission/mission_msg.h"

#include <math.h>

#include "subsystems/datalink/downlink.h"

#include "state.h"
#include "autopilot.h"
#include "subsystems/gps.h"
#include "generated/airframe.h"
#include "dl_protocol.h"


struct _mission mission;


void mission_init(void) {
  mission.mission_insert_idx = 0;
  mission.mission_extract_idx = 0;
}

int mission_msg_GOTO_WP(float x, float y, float a, uint16_t d) {
  mission.elements[insert_idx].type = MissionWP;
  mission.elements[insert_idx].element.mission_wp.wp.x = x;
  mission.elements[insert_idx].element.mission_wp.wp.y = y;
  mission.elements[insert_idx].element.mission_wp.wp.a = a;
  mission.elements[insert_idx].element.duration = d;

  insert_idx++;
  if (insert_idx == MISSION_ELEMENT_NB) insert_idx = 0;
  return TRUE;
}

int mission_msg_SEGMENT(float x1, float y1, float x2, float y2) {
  if (nav_approaching_xy(x2, y2, x1, y1, CARROT)) {
    return FALSE; // end of block and go to the next block
//    NextBlock();
  }
  else {
    //NextBlock()
    //nav_init_stage();
    nav_route_xy(x1, y1, x2, y2);
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(5.0, 0.);
  }
  return TRUE;
}



/** Status along the mission path */
enum mission_msg_PATH_status { Path12, Path23, Path34, Path45 };
static enum mission_msg_PATH_status mission_msg_PATH_status;

int mission_msg_PATH_init( void ) {
  // status of the first segment of the path
  mission_msg_PATH_status = Path12;

  // path finder here to generate intermediate waypoints
  // these five points are defined as global variables in mission_info.h
  //struct tabWaypoint tabW;
  //tabW = pathFinder();
  //pathFinder();

  return FALSE;
}

int mission_msg_PATH(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, float x5, float y5) {

  switch(mission_msg_PATH_status) {
  case Path12: /* path 1-2 */
    nav_route_xy(x1, y1, x2, y2);
    if (nav_approaching_xy(x2, y2, x1, y1, CARROT)) {
      mission_msg_PATH_status = Path23;
      //nav_init_stage();
    }
    break;
  case Path23: /* path 2-3 */
    nav_route_xy(x2, y2, x3, y3);
    if (nav_approaching_xy(x3, y3, x2, y2, CARROT)) {
      mission_msg_PATH_status = Path34;
      //nav_init_stage();
    }
    break;
  case Path34: /* path 3-4 */
    nav_route_xy(x3, y3, x4, y4);
    if (nav_approaching_xy(x4, y4, x3, y3, CARROT)) {
      mission_msg_PATH_status = Path45;
      //nav_init_stage();
    }
    break;
  case Path45: /* path 4-5 */
    nav_route_xy(x4, y4, x5, y5);
    if (nav_approaching_xy(x5, y5, x4, y4, CARROT)) {
      //mission_msg_PATH_status = Path12;
      //nav_init_stage();
      return FALSE; /* This path ends when AC arrive to point 5 */
    }
    break;
  }


  return TRUE; /* do the function */
}





/*
 * Copyright (C) 2013 ENAC
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file mission_nav.h
 *  @brief mission navigation
 */

#include <stdio.h>
#include "modules/mission/mission_nav.h"
//#include "std.h"
//#include "subsystems/nav.h"


int mission_nav_GOTO_WP(float x, float y) {
  fly_to_xy(x, y);
  return FALSE;
}

//int mission_nav_SEGMENT(float x1, float y1, float x2, float y2) { 
bool_t mission_nav_SEGMENT(float x1, float y1, float x2, float y2) {

  if (nav_approaching_xy(x2, y2, x1, y1, CARROT)) {
    //mission_nav_finish = 1;
    return FALSE; // end of block and go to the next block
  } 
  else {
    nav_route_xy(x1, y1, x2, y2);
    printf("doing mission SEGMENT ...\n\n");fflush(stdout);
  }
  return TRUE;
}

//int mission_nav_CIRCLE(float x, float y, float radius){
bool_t mission_nav_CIRCLE(float x, float y, float radius){  
  nav_circle_XY(x, y, radius);
  printf("doing mission CIRCLE ...\n\n");fflush(stdout);
  return TRUE;
}



int mission_path_seg_idx;
mission_path_seg_idx = 1;
//int mission_nav_PATH(struct _mission_path mission_path_nav) {
bool_t mission_nav_PATH(struct _mission_path mission_path_nav) {
  float x1, y1, x2, y2, x3, y3, x4, y4, x5, y5;
  switch (mission_path_seg_idx) {
  case 1: // path 1-2  
    x1 = mission_path_nav.path[0].x;
    y1 = mission_path_nav.path[0].y;
    x2 = mission_path_nav.path[1].x;
    y2 = mission_path_nav.path[1].y;
    nav_route_xy(x1, y1, x2, y2);
    printf("doing mission PATH: path 1-2...\n\n");fflush(stdout);
    if (nav_approaching_xy(x2, y2, x1, y1, CARROT)) {
      mission_path_seg_idx = 2;
    }
    break; 
  case 2: // path 2-3
    x2 = mission_path_nav.path[1].x;
    y2 = mission_path_nav.path[1].y;
    x3 = mission_path_nav.path[2].x;
    y3 = mission_path_nav.path[2].y;
    nav_route_xy(x2, y2, x3, y3);
    printf("doing mission PATH: path 2-3...\n\n");fflush(stdout);
    if (nav_approaching_xy(x3, y3, x2, y2, CARROT)) {
      mission_path_seg_idx = 3;
    }
    break;      
  case 3: // path 3-4 
    x3 = mission_path_nav.path[2].x;
    y3 = mission_path_nav.path[2].y;
    x4 = mission_path_nav.path[3].x;
    y4 = mission_path_nav.path[3].y;
    nav_route_xy(x3, y3, x4, y4);
    printf("doing mission PATH: path 3-4...\n\n");fflush(stdout);
    if (nav_approaching_xy(x4, y4, x3, y3, CARROT)) {
      mission_path_seg_idx = 4;
    }
    break;      
  case 4: // path 4-5 
    x4 = mission_path_nav.path[3].x;
    y4 = mission_path_nav.path[3].y;
    x5 = mission_path_nav.path[4].x;
    y5 = mission_path_nav.path[4].y;
    nav_route_xy(x4, y4, x5, y5);
    printf("doing mission PATH: path 4-5...\n\n");fflush(stdout);
    if (nav_approaching_xy(x5, y5, x4, y4, CARROT)) {
      //mission_nav_finish = 1;
      return FALSE; // This path ends when AC arrive to point 5
    }
    break;           
  }

  return TRUE; // do the function
}


bool_t mission_nav_return;
bool_t mission_nav_finish;
int i_mission;
int mission_call(struct _mission mission) {
//void mission_nav() {
  //int i_mission;  
  // detect where the mission list is (by detecting duration non NULL)??????????????????????????????????????????????????????????????????????
  
     float wp_mission_x, wp_mission_y;
     float wp_mission_from_x, wp_mission_from_y, wp_mission_to_x, wp_mission_to_y; 
     float wp_center_x, wp_center_y, radius;
     struct _mission_path mission_path_nav;

     switch (mission.elements[i_mission].type){
     case MissionWP:
        wp_mission_x = mission.elements[i_mission].element.mission_wp.wp.x;
        wp_mission_y = mission.elements[i_mission].element.mission_wp.wp.y;
        //mission_nav_GOTO_WP(wp_mission_x, wp_mission_y); 
        mission_nav_GOTO_WP(wp_mission_x, wp_mission_y);       
        break;

     case MissionSegment:
        wp_mission_from_x = mission.elements[i_mission].element.mission_segment.from.x;
        wp_mission_from_y = mission.elements[i_mission].element.mission_segment.from.y;
        wp_mission_to_x = mission.elements[i_mission].element.mission_segment.to.x;
        wp_mission_to_y = mission.elements[i_mission].element.mission_segment.to.y;
        //mission_nav_SEGMENT(wp_mission_from_x, wp_mission_from_y, wp_mission_to_x, wp_mission_to_y);
        mission_nav_return = mission_nav_SEGMENT(wp_mission_from_x, wp_mission_from_y, wp_mission_to_x, wp_mission_to_y);               
        break;

     case MissionCircle:
        wp_center_x = mission.elements[i_mission].element.mission_circle.center.x;
        wp_center_y = mission.elements[i_mission].element.mission_circle.center.y;
        radius = mission.elements[i_mission].element.mission_circle.radius;        
        //mission_nav_CIRCLE(wp_center_x, wp_center_y, radius); 
        mission_nav_return = mission_nav_CIRCLE(wp_center_x, wp_center_y, radius);  
        break;

     case MissionPath:         
        mission_path_nav = mission.elements[i_mission].element.mission_path;       
        //mission_nav_PATH(mission_path_nav);
        mission_nav_return = mission_nav_PATH(mission_path_nav);       
        break;
     }

  if (!(mission_nav_return)){ //when mission.elements is finished           
     printf("finish mission.elements  %d\n\n\n", i_mission);fflush(stdout);
     i_mission++;
     printf("goto mission.elements  %d\n\n\n", i_mission);fflush(stdout);
  } 

  
  if (i_mission == MISSION_ELEMENT_NB) i_mission = 0;
  return TRUE; /* do the function */

}



int goto_mission_msg(uint8_t ac_id, uint8_t mission_id){
  i_mission = mission_id;
  mission_call(mission);
}

/** \file mission_msg.c
 *  \brief the new messages for mission planner library
 *  \ edited by Anh Truong
 */

//#define MISSION_C

#include <stdio.h>
#include "modules/mission/mission_msg.h"

#include <math.h>

#include "subsystems/datalink/downlink.h"

/*#include "state.h"
#include "autopilot.h"
#include "subsystems/gps.h"
#include "generated/airframe.h"
#include "dl_protocol.h"*/


struct _mission mission;


void mission_init(void) {
  mission.mission_insert_idx = 0;
  mission.mission_extract_idx = 0;
  //mission_nav_finish = FALSE;
}

int mission_msg_GOTO_WP(float x, float y, float a, uint16_t d) {
  //int insert_idx = mission.mission_insert_idx;
  mission.elements[mission.mission_insert_idx].type = MissionWP;
  mission.elements[mission.mission_insert_idx].element.mission_wp.wp.x = x;
  mission.elements[mission.mission_insert_idx].element.mission_wp.wp.y = y;
  mission.elements[mission.mission_insert_idx].element.mission_wp.wp.a = a;
  mission.elements[mission.mission_insert_idx].duration = d;

  //printf("stocking mission GOTO_WP...\n");fflush(stdout);
  mission.mission_insert_idx++;
  if (mission.mission_insert_idx == MISSION_ELEMENT_NB) mission.mission_insert_idx = 0;
  return TRUE;
}

int mission_msg_SEGMENT(float x1, float y1, float x2, float y2, float a, uint16_t d) {
  //int insert_idx = mission.mission_insert_idx;
  mission.elements[mission.mission_insert_idx].type = MissionSegment;
  mission.elements[mission.mission_insert_idx].element.mission_segment.from.x = x1;
  mission.elements[mission.mission_insert_idx].element.mission_segment.from.y = y1;
  mission.elements[mission.mission_insert_idx].element.mission_segment.from.a = a;
  mission.elements[mission.mission_insert_idx].element.mission_segment.to.x = x2;
  mission.elements[mission.mission_insert_idx].element.mission_segment.to.y = y2;
  mission.elements[mission.mission_insert_idx].element.mission_segment.to.a = a;
  mission.elements[mission.mission_insert_idx].duration = d;

  //printf("stocking mission SEGMENT...\n");fflush(stdout);
  //printf("mission.mission_insert_idx = %d\n\n", mission.mission_insert_idx);fflush(stdout);
  mission.mission_insert_idx++;
  if (mission.mission_insert_idx == MISSION_ELEMENT_NB) mission.mission_insert_idx = 0;
  return TRUE; 
}


int mission_msg_CIRCLE(float x, float y, float radius, float a, uint16_t d) {
  //int insert_idx = mission.mission_insert_idx;
  mission.elements[mission.mission_insert_idx].type = MissionCircle;
  mission.elements[mission.mission_insert_idx].element.mission_circle.center.x = x;
  mission.elements[mission.mission_insert_idx].element.mission_circle.center.y = y;
  mission.elements[mission.mission_insert_idx].element.mission_circle.center.a = a;
  mission.elements[mission.mission_insert_idx].element.mission_circle.radius = radius;
  mission.elements[mission.mission_insert_idx].duration = d;

  //printf("stocking mission CIRCLE...\n");fflush(stdout);
  //printf("mission.mission_insert_idx = %d\n\n", mission.mission_insert_idx);fflush(stdout);
  mission.mission_insert_idx++;
  if (mission.mission_insert_idx == MISSION_ELEMENT_NB) mission.mission_insert_idx = 0;
  return TRUE; 
}

//int i_path, i_point;
int mission_msg_PATH(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, float x5, float y5, float a, uint16_t d) {
  //int insert_idx = mission.mission_insert_idx;
  mission.elements[mission.mission_insert_idx].type = MissionPath;
           
        int i_path;
        i_path = 0;      
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].x = x1;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].y = y1;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].a = a;
        
    
        i_path++;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].x = x2;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].y = y2;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].a = a;
        

        i_path++;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].x = x3;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].y = y3;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].a = a;
        

        i_path++;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].x = x4;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].y = y4;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].a = a;
      

        i_path++;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].x = x5;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].y = y5;
        mission.elements[mission.mission_insert_idx].element.mission_path.path[i_path].a = a;
        
 
  mission.elements[mission.mission_insert_idx].duration = d;
  
  //printf("stocking mission PATH...\n");fflush(stdout);
  //printf("mission.mission_insert_idx = %d\n\n", mission.mission_insert_idx);fflush(stdout);
  mission.mission_insert_idx++;
  if (mission.mission_insert_idx == MISSION_ELEMENT_NB) mission.mission_insert_idx = 0;
  return TRUE;
}





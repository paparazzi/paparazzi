#ifndef COMMON_NAV_H
#define COMMON_NAV_H

#include "std.h"

extern float dist2_to_home;
extern bool_t too_far_from_home;

struct point {
  float x;
  float y;
  float a;
};

#define WaypointX(_wp) (waypoints[_wp].x)
#define WaypointY(_wp) (waypoints[_wp].y)
#define WaypointAlt(_wp) (waypoints[_wp].a)

extern const uint8_t nb_waypoint;
extern struct point waypoints[];
/** size == nb_waypoint, waypoint 0 is a dummy waypoint */ 

/** In s */
extern uint16_t stage_time, block_time;

extern uint8_t nav_stage, nav_block;

extern float ground_alt; /* m */

extern int32_t nav_utm_east0;  /* m */
extern int32_t nav_utm_north0; /* m */
extern uint8_t nav_utm_zone0;


void nav_init_stage( void );
void nav_init_block(void);
void compute_dist2_to_home(void);
unit_t nav_reset_reference( void ) __attribute__ ((unused));
unit_t nav_update_waypoints_alt( void ) __attribute__ ((unused));


#define InitStage() { nav_init_stage(); return; }

#define Block(x) case x: nav_block=x;
#define InitBlock() { nav_stage = 0; block_time = 0; InitStage(); }
#define NextBlock() { nav_block++; nav_init_block(); return; }
#define GotoBlock(b) { nav_block=b; nav_init_block(); return; }

#define Stage(s) case s: nav_stage=s;
#define NextStage() { nav_stage++; InitStage() }
#define NextStageFrom(wp) { last_wp = wp; NextStage() }
#define GotoStage(s) { nav_stage = s; InitStage() }

#define Label(x) label_ ## x:
#define Goto(x) { goto label_ ## x; }

#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define NavBlockTime() (block_time)
#define LessThan(_x, _y) ((_x) < (_y))

#define NavSetGroundReferenceHere() ({ nav_reset_reference(); nav_update_waypoints_alt(); FALSE; })

#endif /* COMMON_NAV_H */

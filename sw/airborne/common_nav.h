#ifndef COMMON_NAV_H
#define COMMON_NAV_H

#include "std.h"

extern float max_dist_from_home;
extern float dist2_to_home;
extern float dist2_to_wp;
extern bool_t too_far_from_home;

struct point {
  float x;
  float y;
  float a;
};

#define WaypointX(_wp) (waypoints[_wp].x)
#define WaypointY(_wp) (waypoints[_wp].y)
#define WaypointAlt(_wp) (waypoints[_wp].a)

#define MoveWaypoint(_id, _ux, _uy, _a) { \
  if (_id < nb_waypoint) { \
    float dx, dy; \
    dx = _ux - nav_utm_east0 - waypoints[WP_HOME].x; \
    dy = _uy - nav_utm_north0 - waypoints[WP_HOME].y; \
    BoundAbs(dx, max_dist_from_home); \
    BoundAbs(dy, max_dist_from_home); \
    waypoints[_id].x = waypoints[WP_HOME].x + dx; \
    waypoints[_id].y = waypoints[WP_HOME].y + dy; \
    waypoints[_id].a = _a; \
  } \
}


extern const uint8_t nb_waypoint;
extern struct point waypoints[];
/** size == nb_waypoint, waypoint 0 is a dummy waypoint */ 

/** In s */
extern uint16_t stage_time, block_time;

extern uint8_t nav_stage, nav_block;
extern uint8_t last_block, last_stage;


extern float ground_alt; /* m */

extern int32_t nav_utm_east0;  /* m */
extern int32_t nav_utm_north0; /* m */
extern uint8_t nav_utm_zone0;


void nav_init_stage( void );
void nav_init_block(void);
void nav_goto_block(uint8_t block_id);
void compute_dist2_to_home(void);
unit_t nav_reset_reference( void ) __attribute__ ((unused));
unit_t nav_update_waypoints_alt( void ) __attribute__ ((unused));
void common_nav_periodic_task_4Hz(void);


#define InitStage() nav_init_stage();

#define Block(x) case x: nav_block=x;
#define NextBlock() { nav_block++; nav_init_block(); }
#define GotoBlock(b) { nav_block=b; nav_init_block(); }

#define Stage(s) case s: nav_stage=s;
#define NextStageAndBreak() { nav_stage++; InitStage(); break; }
#define NextStageAndBreakFrom(wp) { last_wp = wp; NextStageAndBreak(); }

#define Label(x) label_ ## x:
#define Goto(x) { goto label_ ## x; }
#define Return() ({ nav_block=last_block; nav_stage=last_stage; block_time=0; FALSE;})

#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define NavBlockTime() (block_time)
#define LessThan(_x, _y) ((_x) < (_y))

#define NavSetGroundReferenceHere() ({ nav_reset_reference(); nav_update_waypoints_alt(); FALSE; })

#define NavSetWaypointHere(_wp) ({ \
  waypoints[_wp].x = estimator_x; \
  waypoints[_wp].y = estimator_y; \
  FALSE; \
})

#endif /* COMMON_NAV_H */

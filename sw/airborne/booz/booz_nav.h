#ifndef BOOZ_NAV_H
#define BOOZ_NAV_H

#include "std.h"

#include "booz_nav_hover.h"

extern void booz_nav_init(void);
extern void booz_nav_run(void);
extern void booz_nav_read_setpoints_from_rc(void);


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

extern uint16_t stage_time, block_time;
extern uint8_t nav_stage, nav_block;

extern void nav_init_stage( void );
#define InitStage() { nav_init_stage(); return; }




#endif /* BOOZ_NAV_H */

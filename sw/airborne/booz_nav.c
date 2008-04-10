#include "booz_nav.h"

#include <math.h>

#include "booz_estimator.h"
#include "booz_control.h"
#include "radio_control.h"


#define Block(x) case x: nav_block=x;
#define InitBlock() { nav_stage = 0; block_time = 0; InitStage(); }
#define NextBlock() { nav_block++; InitBlock(); }
#define GotoBlock(b) { nav_block=b; InitBlock(); }

#define Stage(s) case s: nav_stage=s;
#define NextStage() { nav_stage++; InitStage() }
#define NextStageFrom(wp) { last_wp = wp; NextStage() }
#define GotoStage(s) { nav_stage = s; InitStage() }

#define NavGotoWaypoint(_wp) {}
#define NavVerticalAutoThrottleMode(_foo) {}
#define NavVerticalAltitudeMode(_foo, _bar) {}

#define NavCircleWaypoint(_foo, _bar) {}

extern void nav_home(void);
#define NAV_C
#include "flight_plan.h"

const uint8_t nb_waypoint;
struct point waypoints[NB_WAYPOINT] = WAYPOINTS; 

uint8_t nav_stage, nav_block;
uint16_t stage_time, block_time;


#include "booz_nav_hover.h"

void booz_nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
  booz_nav_hover_init();
}


void booz_nav_run(void) {
  booz_nav_hover_run();
  BoozControlAttitudeSetSetPoints(booz_nav_hover_phi_command, booz_nav_hover_theta_command,
				  booz_nav_hover_psi_sp, booz_nav_hover_power_command);
}

void booz_nav_read_setpoints_from_rc(void) {
  booz_nav_hover_read_setpoints_from_rc();
}




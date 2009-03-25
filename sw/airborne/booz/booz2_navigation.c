#define NAV_C

#include "booz2_navigation.h"

#include "booz2_ins.h"

#include "flight_plan.h"

#include "pprz_algebra_int.h"

const uint8_t nb_waypoint = NB_WAYPOINT;
struct EnuCoor_i waypoints[NB_WAYPOINT] = WAYPOINTS_INT32;

struct EnuCoor_i booz2_navigation_target;
struct EnuCoor_i booz2_navigation_carrot;

struct EnuCoor_i nav_last_point;

uint16_t stage_time, block_time;

uint8_t nav_stage, nav_block;
uint8_t last_block, last_stage;
uint8_t last_wp __attribute__ ((unused));

int32_t ground_alt, nav_altitude;

#define ARRIVED_AT_WAYPOINT (20 << 8)
#define CARROT_DIST (10 << 8)

void booz2_nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
  ground_alt = (int32_t)(GROUND_ALT * 100); // cm
  nav_altitude = ground_alt + SECURITY_HEIGHT;
  INT32_VECT3_COPY( booz2_navigation_target, waypoints[WP_HOME]);
  INT32_VECT3_COPY( booz2_navigation_carrot, waypoints[WP_HOME]);

}

void booz2_nav_run(void) {

  /* compute a vector to the waypoint */
  struct Int32Vect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, booz2_navigation_target, booz_ins_enu_pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -(1<<15), (1<<15));

  int32_t dist_to_waypoint;
  INT32_VECT2_NORM(dist_to_waypoint, path_to_waypoint);
  
  if (dist_to_waypoint < ARRIVED_AT_WAYPOINT) {
    VECT2_COPY( booz2_navigation_carrot, booz2_navigation_target);
  }
  else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, CARROT_DIST, path_to_waypoint);
    VECT2_SDIV(path_to_carrot, dist_to_waypoint, path_to_carrot);
    VECT2_SUM(booz2_navigation_carrot, path_to_carrot, booz_ins_enu_pos);
  }
}


bool_t nav_approaching_from(uint8_t wp_idx, uint8_t from_idx __attribute__ ((unused))) {
  uint32_t dist_to_point;
  struct Int32Vect2 diff;
  VECT2_DIFF(diff, waypoints[wp_idx], booz_ins_enu_pos);
  INT32_VECT2_NORM(dist_to_point, diff);
  if (dist_to_point < ARRIVED_AT_WAYPOINT) return TRUE;
  return FALSE; // complete to take "from" into account
}

static int32_t previous_ground_alt;

/** Reset the geographic reference to the current GPS fix */
unit_t nav_reset_reference( void ) {
  booz_ins_ltp_initialised = FALSE;
  previous_ground_alt = ground_alt;
  ground_alt = booz_ins_enu_pos.z;
  return 0;
}

/** Shift altitude of the waypoint according to a new ground altitude */
unit_t nav_update_waypoints_alt( void ) {
  uint8_t i;
  for(i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].z += ground_alt - previous_ground_alt;
  }
  return 0;
}

void nav_init_stage( void ) {
  INT32_VECT3_COPY(nav_last_point, booz_ins_enu_pos);
  stage_time = 0;
}

void nav_init_block(void) { 
  if (nav_block >= NB_BLOCK)
    nav_block=NB_BLOCK-1;
  nav_stage = 0;
  block_time = 0;
  InitStage();
}

void nav_goto_block(uint8_t b) {
  if (b != nav_block) { /* To avoid a loop in a the current block */
    last_block = nav_block;
    last_stage = nav_stage;
  }
  GotoBlock(b);
}

void nav_periodic_task_10Hz() {
  RunOnceEvery(10, { stage_time++;  block_time++; });

  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  booz2_nav_run();
}

void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos) {
  if (wp_id < nb_waypoint) {
    INT32_VECT3_COPY(waypoints[wp_id],(*new_pos));
  }
}

void nav_home(void) {}


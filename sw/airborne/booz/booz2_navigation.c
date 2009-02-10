#define NAV_C

#include "booz2_navigation.h"

#include "booz2_ins.h"

#include "flight_plan.h"


const uint8_t nb_waypoint = NB_WAYPOINT;
struct Pprz_int32_lla waypoints[NB_WAYPOINT] = WAYPOINTS_LATLONG;

struct Pprz_int32_lla booz2_navigation_target;
struct Pprz_int32_lla booz2_navigation_carrot;

struct Pprz_int32_lla nav_last_point;

uint16_t stage_time, block_time;

uint8_t nav_stage, nav_block;
uint8_t last_block, last_stage;
uint8_t last_wp __attribute__ ((unused));

int32_t ground_alt, nav_altitude;

#define ARRIVED_AT_WAYPOINT (300)
#define CARROT_DIST (200)

void booz2_nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
  ground_alt = (int32_t)(GROUND_ALT * 100); // cm
  PPRZ_INT32_LLA_COPY( booz2_navigation_target, waypoints[WP_HOME]);
  PPRZ_INT32_LLA_COPY( booz2_navigation_carrot, waypoints[WP_HOME]);

}

void booz2_nav_run(void) {

  /* compute a vector to the waypoint */
  struct Pprz_int32_lla path_to_waypoint;
  PPRZ_INT32_LLA_DIFF_LL(path_to_waypoint, booz2_navigation_target, booz_ins_position_lla);

  /* saturate it */
  PPRZ_INT32_LLA_STRIM_LL(path_to_waypoint, -(1<<15), (1<<15));

  int32_t dist_to_waypoint;
  PPRZ_INT32_LLA_NORM_LL(dist_to_waypoint, path_to_waypoint);
  
  if (dist_to_waypoint < ARRIVED_AT_WAYPOINT) {
    PPRZ_INT32_LLA_COPY( booz2_navigation_carrot, booz2_navigation_target);
  }
  else {
    struct Pprz_int32_lla path_to_carrot;
    PPRZ_INT32_LLA_SMULT_LL(path_to_carrot, path_to_waypoint, CARROT_DIST);
    PPRZ_INT32_LLA_SDIV_LL(path_to_carrot, path_to_carrot, dist_to_waypoint);
    PPRZ_INT32_LLA_SUM_LL(booz2_navigation_carrot, path_to_carrot, booz_ins_position_lla);
  }
  // FIXME
  //  PPRZ_INT32_LLA_COPY(booz2_guidance_h_pos_sp, booz2_navigation_carrot);

}


bool_t nav_approaching_from(uint8_t wp_idx, uint8_t from_idx __attribute__ ((unused))) {
  uint32_t dist_to_point;
  struct Pprz_int32_lla diff;
  PPRZ_INT32_LLA_DIFF_LL(diff, waypoints[wp_idx], booz_ins_position_lla);
  PPRZ_INT32_LLA_NORM_LL(dist_to_point, diff);
  if (dist_to_point < ARRIVED_AT_WAYPOINT) return TRUE;
  return FALSE; // complete to take "from" into account
}

static int32_t previous_ground_alt;

/** Reset the geographic reference to the current GPS fix */
unit_t nav_reset_reference( void ) {
  uint8_t i;
  for(i = 0; i < NB_WAYPOINT; i++) {
    struct Pprz_int32_lla diff;
    PPRZ_INT32_LLA_DIFF_LL(diff, waypoints[i], booz_ins_position_init_lla);
    PPRZ_INT32_LLA_SUM_LL(waypoints[i], booz_ins_position_lla, diff);
  }
  PPRZ_INT32_LLA_COPY(booz_ins_position_init_lla, booz_ins_position_lla);
  previous_ground_alt = ground_alt;
  ground_alt = booz_ins_position_lla.alt;
  return 0;
}

/** Shift altitude of the waypoint according to a new ground altitude */
unit_t nav_update_waypoints_alt( void ) {
  uint8_t i;
  for(i = 0; i < NB_WAYPOINT; i++) {
    waypoints[i].alt += ground_alt - previous_ground_alt;
  }
  return 0;
}

void nav_init_stage( void ) {
  PPRZ_INT32_LLA_COPY(nav_last_point, booz_ins_position_lla);
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

void nav_move_waypoint(uint8_t wp_id, struct Pprz_int32_lla new_pos) {
  if (wp_id < nb_waypoint) {
    PPRZ_INT32_LLA_COPY(waypoints[wp_id],new_pos);
  }
}

void nav_home(void) {}


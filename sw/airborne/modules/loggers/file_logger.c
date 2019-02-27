/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include "generated/modules.h"
#include "mcu_periph/sys_time.h"

#include <stdio.h>
#include "std.h"

#include <pthread.h>

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** Select variables to log */
#ifndef FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
#define FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE FALSE
#endif
#ifndef FILE_LOGGER_LOG_AUTOPILOT
#define FILE_LOGGER_LOG_AUTOPILOT FALSE
#endif
#ifndef FILE_LOGGER_LOG_ATTITUDE
#define FILE_LOGGER_LOG_ATTITUDE FALSE
#endif
#ifndef FILE_LOGGER_LOG_LTP_POS
#define FILE_LOGGER_LOG_LTP_POS FALSE
#endif
#ifndef FILE_LOGGER_LOG_LTP_VEL
#define FILE_LOGGER_LOG_LTP_VEL FALSE
#endif
#ifndef FILE_LOGGER_LOG_BODY_VEL
#define FILE_LOGGER_LOG_BODY_VEL FALSE
#endif
#ifndef FILE_LOGGER_LOG_SONAR_BEBOP
#define FILE_LOGGER_LOG_SONAR_BEBOP FALSE
#endif
#ifndef FILE_LOGGER_LOG_BARO
#define FILE_LOGGER_LOG_BARO FALSE
#endif
#ifndef FILE_LOGGER_LOG_BATTERY
#define FILE_LOGGER_LOG_BATTERY FALSE
#endif
#ifndef FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
#define FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE FALSE
#endif
#ifndef FILE_LOGGER_LOG_PERCEVITE_VECTORS
#define FILE_LOGGER_LOG_PERCEVITE_VECTORS FALSE
#endif
#ifndef FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
#define FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS FALSE
#endif

#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
#include "subsystems/navigation/common_flight_plan.h"
#endif
#if FILE_LOGGER_LOG_AUTOPILOT
#include "autopilot.h"
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
#include "modules/sonar/sonar_bebop.h"
#endif
#if FILE_LOGGER_LOG_BARO
#include "modules/air_data/air_data.h"
#endif
#if FILE_LOGGER_LOG_BATTERY
#include "subsystems/electrical.h"
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
  FILE_LOGGER_LOG_PERCEVITE_VECTORS || \
  FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
#include "modules/percevite/percevite.h"
#include "subsystems/navigation/waypoints.h"
#endif


/** Log frame buffer */
struct log_frame_t {
  uint32_t counter;
  float time;
  float dt;
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
  uint8_t block;
  uint8_t stage;
#endif
#if FILE_LOGGER_LOG_AUTOPILOT
  uint8_t ap_mode;
  bool ap_motors_on;
  bool ap_kill_throttle;
  bool ap_in_flight;
  bool ap_ground_detected;
#endif
#if FILE_LOGGER_LOG_ATTITUDE
  struct FloatEulers att;
#endif
#if FILE_LOGGER_LOG_LTP_POS
  struct NedCoor_f ltp_pos;
#endif
#if FILE_LOGGER_LOG_LTP_VEL
  struct NedCoor_f ltp_vel;
#endif
#if FILE_LOGGER_LOG_BODY_VEL
  struct FloatVect3 body_vel;
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
  float sonar_bebop;
#endif
#if FILE_LOGGER_LOG_BARO
  bool baro_valid;
  float baro;
#endif
#if FILE_LOGGER_LOG_BATTERY
  float vsupply;
  float current;
  bool bat_low;
  bool bat_crit;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
    FILE_LOGGER_LOG_PERCEVITE_VECTORS || \
    FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
  bool percevite_ok;
  float percevite_timeout;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
  struct FloatVect3 percevite_vel;
  float percevite_time_since_vel;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VECTORS
  struct FloatVect3 percevite_request;
  uint8_t percevite_request_flags;
  struct FloatVect3 percevite_reply;
  uint8_t percevite_reply_flags;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
  uint8_t percevite_wp_id;
  struct NedCoor_f percevite_wp_pos;
  uint8_t percevite_tgt_id;
  struct NedCoor_f percevite_tgt_pos;
#endif
};
static volatile struct log_frame_t log_frame;

static pthread_t log_thread; ///< Logging thread
static volatile bool log_frame_requested = FALSE; ///< TRUE if periodic should fill log buffer
static volatile bool log_thread_is_running = FALSE; ///< TRUE while log thread is running
static volatile bool log_thread_should_run = FALSE; ///< TRUE as long as log thread should run

/** Logging THREAD */
void *file_logger_thread(void *arg);
void *file_logger_thread(void *arg) {
  log_thread_is_running = TRUE;
  FILE *file_logger = NULL;

  /* Open file */
  uint32_t counter = 0;
  char filename[512];
  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }
  file_logger = fopen(filename, "w");
  if(!file_logger) {
    log_thread_is_running = FALSE;
    return NULL;
  }

  fprintf(file_logger, "counter,time,dt,");
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
  fprintf(file_logger, "block,stage,");
#endif
#if FILE_LOGGER_LOG_AUTOPILOT
  fprintf(file_logger, "ap_mode,ap_motors_on,ap_kill_throttle,ap_in_flight,ap_ground_detected,");
#endif
#if FILE_LOGGER_LOG_ATTITUDE
  fprintf(file_logger, "att_phi,att_theta,att_psi,");
#endif
#if FILE_LOGGER_LOG_LTP_POS
  fprintf(file_logger, "pos_ltp_x,pos_ltp_y,pos_ltp_z,");
#endif
#if FILE_LOGGER_LOG_LTP_VEL
  fprintf(file_logger, "vel_ltp_x,vel_ltp_y,vel_ltp_z,");
#endif
#if FILE_LOGGER_LOG_BODY_VEL
  fprintf(file_logger, "vel_body_x,vel_body_y,vel_body_z,");
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
  fprintf(file_logger, "sonar_bebop,");
#endif
#if FILE_LOGGER_LOG_BARO
  fprintf(file_logger, "baro_valid,baro,");
#endif
#if FILE_LOGGER_LOG_BATTERY
  fprintf(file_logger, "v_supply,current,bat_low,bat_crit,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
    FILE_LOGGER_LOG_PERCEVITE_VECTORS || \
    FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
  fprintf(file_logger, "percevite_ok,");
  fprintf(file_logger, "percevite_timeout,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
  fprintf(file_logger, "percevite_vx,percevite_vy,percevite_vz,percevite_time_since_vel,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VECTORS
  fprintf(file_logger, "percevite_request_x,percevite_request_y,percevite_request_z,percevite_request_flags,percevite_reply_x,percevite_reply_y,percevite_reply_z,percevite_reply_flags,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
  fprintf(file_logger, "percevite_wp_id,percevite_wp_x,percevite_wp_y,percevite_wp_z,percevite_tgt_id,percevite_tgt_x,percevite_tgt_y,percevite_tgt_z,");
#endif
  fprintf(file_logger, "\n");

  /* Write log */
  while(log_thread_should_run) {
    log_frame_requested = TRUE;
    while(log_frame_requested && log_thread_should_run) pthread_yield(); // Spin until frame is filled in by periodic
    fprintf(file_logger, "%u,%f,%f,",
        log_frame.counter, log_frame.time, log_frame.dt);
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
    fprintf(file_logger, "%u,%u,",
        log_frame.block, log_frame.stage);
#endif
#if FILE_LOGGER_LOG_AUTOPILOT
    fprintf(file_logger, "%u,%d,%d,%d,%d,",
        log_frame.ap_mode, log_frame.ap_motors_on,
        log_frame.ap_kill_throttle, log_frame.ap_in_flight,
        log_frame.ap_ground_detected);
#endif
#if FILE_LOGGER_LOG_ATTITUDE
    fprintf(file_logger, "%f,%f,%f,",
        log_frame.att.phi, log_frame.att.theta, log_frame.att.psi);
#endif
#if FILE_LOGGER_LOG_LTP_POS
    fprintf(file_logger, "%f,%f,%f,",
        log_frame.ltp_pos.x, log_frame.ltp_pos.y, log_frame.ltp_pos.z);
#endif
#if FILE_LOGGER_LOG_LTP_VEL
    fprintf(file_logger, "%f,%f,%f,",
        log_frame.ltp_vel.x, log_frame.ltp_vel.y, log_frame.ltp_vel.z);
#endif
#if FILE_LOGGER_LOG_BODY_VEL
    fprintf(file_logger, "%f,%f,%f,",
        log_frame.body_vel.x, log_frame.body_vel.y, log_frame.body_vel.z);
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
    fprintf(file_logger, "%f,",
        log_frame.sonar_bebop);
#endif
#if FILE_LOGGER_LOG_BARO
    fprintf(file_logger, "%d,%f,",
        log_frame.baro_valid, log_frame.baro);
#endif
#if FILE_LOGGER_LOG_BATTERY
    fprintf(file_logger, "%f,%f,%d,%d,",
        log_frame.vsupply, log_frame.current,
        log_frame.bat_low, log_frame.bat_crit);
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
  FILE_LOGGER_LOG_PERCEVITE_VECTORS || \
  FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
    fprintf(file_logger, "%d,%f,",
        log_frame.percevite_ok,
        log_frame.percevite_timeout);
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
    fprintf(file_logger, "%f,%f,%f,%f,",
        log_frame.percevite_vel.x,
        log_frame.percevite_vel.y,
        log_frame.percevite_vel.z,
        log_frame.percevite_time_since_vel);
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VECTORS
    fprintf(file_logger, "%f,%f,%f,%u,%f,%f,%f,%u,",
        log_frame.percevite_request.x,
        log_frame.percevite_request.y,
        log_frame.percevite_request.z,
        log_frame.percevite_request_flags,
        log_frame.percevite_reply.x,
        log_frame.percevite_reply.y,
        log_frame.percevite_reply.z,
        log_frame.percevite_reply_flags);
#endif
#if FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
    fprintf(file_logger, "%u,%f,%f,%f,%u,%f,%f,%f,",
        log_frame.percevite_wp_id,
        log_frame.percevite_wp_pos.x,
        log_frame.percevite_wp_pos.y,
        log_frame.percevite_wp_pos.z,
        log_frame.percevite_tgt_id,
        log_frame.percevite_tgt_pos.x,
        log_frame.percevite_tgt_pos.y,
        log_frame.percevite_tgt_pos.z);
#endif
    fprintf(file_logger, "\n");
  }

  /* Close file */
  fclose(file_logger);
  log_thread_is_running = FALSE;
  return NULL;
}


/** Start the file logger and open a new file */
void file_logger_start(void)
{
  if(!log_thread_is_running) {
    log_thread_should_run = TRUE;
    log_thread_is_running = TRUE;
    pthread_create(&log_thread, NULL, &file_logger_thread, NULL);
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  log_thread_should_run = FALSE;
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  static float time = 0;
  static uint32_t counter = 0;

  counter++;
  float prev_time = time;
  time = get_sys_time_float();

  if(log_frame_requested) {
    log_frame.counter = counter;
    log_frame.time = time;
    log_frame.dt = time - prev_time;
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
    log_frame.block = nav_block;
    log_frame.stage = nav_stage;
#endif
#if FILE_LOGGER_LOG_AUTOPILOT
    log_frame.ap_mode = autopilot.mode;
    log_frame.ap_motors_on = autopilot.motors_on;
    log_frame.ap_kill_throttle = autopilot.kill_throttle;
    log_frame.ap_in_flight = autopilot.in_flight;
    log_frame.ap_ground_detected = autopilot.ground_detected;
#endif
#if FILE_LOGGER_LOG_ATTITUDE
    log_frame.att = *stateGetNedToBodyEulers_f();
#endif
#if FILE_LOGGER_LOG_LTP_POS
    log_frame.ltp_pos = *stateGetPositionNed_f();
#endif
#if FILE_LOGGER_LOG_LTP_VEL
    log_frame.ltp_vel = *stateGetSpeedNed_f();
#endif
#if FILE_LOGGER_LOG_BODY_VEL
    struct FloatRMat *R = stateGetNedToBodyRMat_f();
    struct NedCoor_f *vel_ltp = stateGetSpeedNed_f();
    MAT33_VECT3_MUL(log_frame.body_vel, *R, *vel_ltp);
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
    log_frame.sonar_bebop = sonar_bebop.distance;
#endif
#if FILE_LOGGER_LOG_BARO
    log_frame.baro_valid = air_data.amsl_baro_valid;
    log_frame.baro = air_data.amsl_baro;
#endif
#if FILE_LOGGER_LOG_BATTERY
    log_frame.vsupply = electrical.vsupply / 10.0;
    log_frame.current = electrical.current / 1000.0;
    log_frame.bat_low = electrical.bat_low;
    log_frame.bat_crit = electrical.bat_critical;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
    FILE_LOGGER_LOG_PERCEVITE_VECTORS || \
    FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
    log_frame.percevite_ok = PerceviteOk();
    log_frame.percevite_timeout = percevite.timeout;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
    log_frame.percevite_vel = percevite_logging.velocity;
    log_frame.percevite_time_since_vel = percevite.time_since_velocity;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VECTORS
    log_frame.percevite_request.x = percevite_logging.request.x;
    log_frame.percevite_request.y = percevite_logging.request.y;
    log_frame.percevite_request.z = percevite_logging.request.z;
    log_frame.percevite_request_flags = percevite_logging.request_flags;
    log_frame.percevite_reply.x = percevite_logging.reply.x;
    log_frame.percevite_reply.y = percevite_logging.reply.y;
    log_frame.percevite_reply.z = percevite_logging.reply.z;
    log_frame.percevite_reply_flags = percevite_logging.reply_flags;
#endif
#if FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
    struct NedCoor_f wp = {0.0, 0.0, 0.0};
    if(percevite.wp < nb_waypoint) {
      wp.x = waypoint_get_y(percevite.wp); // Note: convert to NED
      wp.y = waypoint_get_x(percevite.wp);
      wp.z = -waypoint_get_alt(percevite.wp);
    }
    struct NedCoor_f tgt = {0.0, 0.0, 0.0};
    if(percevite_logging.target_wp < nb_waypoint) {
      tgt.x = waypoint_get_y(percevite_logging.target_wp); // Note: convert to NED
      tgt.y = waypoint_get_x(percevite_logging.target_wp);
      tgt.z = -waypoint_get_alt(percevite_logging.target_wp);
    }
    log_frame.percevite_wp_id = percevite.wp;
    log_frame.percevite_wp_pos = wp;
    log_frame.percevite_tgt_id = percevite_logging.target_wp;
    log_frame.percevite_tgt_pos = tgt;
#endif

    log_frame_requested = FALSE;
  }
}

/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow_module.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */


#include "opticflow_module.h"

// Computervision Runs in a thread
#include "opticflow/opticflow_thread.h"
#include "opticflow/inter_thread_data.h"

// Navigate Based On Vision, needed to call init/run_hover_stabilization_onvision
#include "opticflow/hover_stabilization.h"

// Threaded computer vision
#include <pthread.h>

// Sockets
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

int cv_sockets[2];

// Paparazzi Data
#include "state.h"
#include "subsystems/abi.h"

// Downlink
#include "subsystems/datalink/downlink.h"


struct PPRZinfo opticflow_module_data;

/** height above ground level, from ABI
 * Used for scale computation, negative value means invalid.
 */
/** default sonar/agl to use in opticflow visual_estimator */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST
#endif
abi_event agl_ev;
static void agl_cb(uint8_t sender_id, float distance);

static void agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  if (distance > 0) {
    opticflow_module_data.agl = distance;
  }
}

#define DEBUG_INFO(X, ...) ;

void opticflow_module_init(void)
{
  // get AGL from sonar via ABI
  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &agl_ev, agl_cb);

  // Initialize local data
  opticflow_module_data.cnt = 0;
  opticflow_module_data.phi = 0;
  opticflow_module_data.theta = 0;
  opticflow_module_data.agl = 0;

  // Stabilization Code Initialization
  init_hover_stabilization_onvision();
}


void opticflow_module_run(void)
{
  // Send Updated data to thread
  opticflow_module_data.cnt++;
  opticflow_module_data.phi = stateGetNedToBodyEulers_f()->phi;
  opticflow_module_data.theta = stateGetNedToBodyEulers_f()->theta;
  int bytes_written = write(cv_sockets[0], &opticflow_module_data, sizeof(opticflow_module_data));
  if (bytes_written != sizeof(opticflow_module_data) && errno !=4){
    printf("[module] Failed to write to socket: written = %d, error=%d, %s.\n",bytes_written, errno, strerror(errno));
  }
  else {
    DEBUG_INFO("[module] Write # %d (%d bytes)\n",opticflow_module_data.cnt, bytes_written);
  }

  // Read Latest Vision Module Results
  struct CVresults vision_results;
  // Warning: if the vision runs faster than the module, you need to read multiple times
  int bytes_read = recv(cv_sockets[0], &vision_results, sizeof(vision_results), MSG_DONTWAIT);
  if (bytes_read != sizeof(vision_results)) {
    if (bytes_read != -1) {
      printf("[module] Failed to read %d bytes: CV results from socket errno=%d.\n",bytes_read, errno);
    }
  } else {
    ////////////////////////////////////////////
    // Module-Side Code
    ////////////////////////////////////////////
    DEBUG_INFO("[module] Read vision %d\n",vision_results.cnt);
    run_hover_stabilization_onvision(&vision_results);
  }
}

void opticflow_module_start(void)
{
  pthread_t computervision_thread;
  if (socketpair(AF_UNIX, SOCK_DGRAM, 0, cv_sockets) == 0) {
    ////////////////////////////////////////////
    // Thread-Side Code
    ////////////////////////////////////////////
    int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main,
                            &cv_sockets[1]);
    if (rc) {
      printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
    }
  }
  else {
    perror("Could not create socket.\n");
  }
}

void opticflow_module_stop(void)
{
  computervision_thread_request_exit();
}

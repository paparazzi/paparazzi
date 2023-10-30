/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
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
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "paparazzi.h"
#include "generated/airframe.h"


#include "nps_main.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_ivy.h"
#include "nps_flightgear.h"

// nps_autopilot.c is not compiled in HITL
struct NpsAutopilot nps_autopilot;

int main(int argc, char **argv)
{
  nps_main_init(argc, argv);

  nps_hitl_impl_init();

  if (nps_main.fg_host) {
    pthread_create(&th_flight_gear, NULL, nps_flight_gear_loop, NULL);
  }
  pthread_create(&th_display_ivy, NULL, nps_main_display, NULL);
  pthread_create(&th_main_loop, NULL, nps_main_loop, NULL);
  pthread_join(th_main_loop, NULL);

  return 0;
}

void nps_radio_and_autopilot_init(void)
{
}

void nps_update_launch_from_dl(uint8_t value)
{
  nps_autopilot.launch = value;
  printf("Launch value=%u\n",nps_autopilot.launch);
}

void nps_main_run_sim_step(void)
{
  nps_atmosphere_update(SIM_DT);

  nps_fdm_run_step(nps_autopilot.launch, nps_autopilot.commands, NPS_COMMANDS_NB);

  nps_sensors_run_step(nps_main.sim_time);
}


void *nps_main_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = SIM_DT * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  // check the sim time difference from the realtime
  // fdm.time - simulation time
  struct timespec startTime;
  struct timespec realTime;
  clock_get_current_time(&startTime);
  double start_secs = ntime_to_double(&startTime);
  double real_secs = 0;
  double real_time = 0;
  static int guard;

  while (TRUE) {
    clock_get_current_time(&requestStart);

    pthread_mutex_lock(&fdm_mutex);

    // check the current simulation time
    clock_get_current_time(&realTime);
    real_secs = ntime_to_double(&realTime);
    real_time = real_secs - start_secs; // real time elapsed

    guard = 0;
    while ((real_time - fdm.time) > SIM_DT) {
      nps_main_run_sim_step();
      nps_main.sim_time = fdm.time;
      guard++;
      if (guard > 2) {
        //If we are too much behind, catch up incrementaly
        break;
      }
    }
    pthread_mutex_unlock(&fdm_mutex);

    clock_get_current_time(&requestEnd);

    // Calculate time it took
    task_ns = (requestEnd.tv_sec - requestStart.tv_sec) * 1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    // task took less than one period, sleep for the rest of time
    if (task_ns < period_ns) {
      waitFor.tv_sec = 0;
      waitFor.tv_nsec = period_ns - task_ns;
      nanosleep(&waitFor, NULL);
    } else {
      // task took longer than the period
#ifdef PRINT_TIME
      printf("MAIN THREAD: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }
  return(NULL);
}



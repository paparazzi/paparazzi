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
#include <sys/time.h>

#include "nps_main.h"
#include "nps_fdm.h"




int main(int argc, char **argv)
{
  if (nps_main_init(argc, argv)) {
    return 1;
  }

  if (nps_main.fg_host) {
    pthread_create(&th_flight_gear, NULL, nps_flight_gear_loop, NULL);
  }
  pthread_create(&th_display_ivy, NULL, nps_main_display, NULL);
  pthread_create(&th_main_loop, NULL, nps_main_loop, NULL);
  pthread_join(th_main_loop, NULL);

  return 0;
}


void nps_update_launch_from_dl(uint8_t value __attribute__((unused))) {}


void nps_radio_and_autopilot_init(void)
{
  enum NpsRadioControlType rc_type;
  char *rc_dev = NULL;
  if (nps_main.norc) {
    rc_type = NORC;
  } else if (nps_main.js_dev) {
    rc_type = JOYSTICK;
    rc_dev = nps_main.js_dev;
  } else if (nps_main.spektrum_dev) {
    rc_type = SPEKTRUM;
    rc_dev = nps_main.spektrum_dev;
  } else {
    rc_type = SCRIPT;
  }
  nps_autopilot_init(rc_type, nps_main.rc_script, rc_dev);
}


void nps_main_run_sim_step(void)
{
  nps_atmosphere_update(SIM_DT);

  nps_autopilot_run_systime_step();

  nps_fdm_run_step(nps_autopilot.launch, nps_autopilot.commands, NPS_COMMANDS_NB);

  nps_sensors_run_step(nps_main.sim_time);

  nps_autopilot_run_step(nps_main.sim_time);

}


void *nps_main_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = HOST_TIMEOUT_MS * 1000000LL; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  struct timeval tv_now;
  double  host_time_now;

  while (TRUE) {
    if (pauseSignal) {
      char line[128];
      double tf = 1.0;
      double t1, t2, irt;

      gettimeofday(&tv_now, NULL);
      t1 = time_to_double(&tv_now);
      // unscale to initial real time
      irt = t1 - (t1 - nps_main.scaled_initial_time) * nps_main.host_time_factor;

      printf("Press <enter> to continue (or CTRL-Z to suspend).\nEnter a new time factor if needed (current: %f): ",
             nps_main.host_time_factor);
      fflush(stdout);
      if (fgets(line, 127, stdin)) {
        if ((sscanf(line, " %le ", &tf) == 1)) {
          if (tf > 0 && tf < 1000) {
            nps_main.host_time_factor = tf;
          }
        }
        printf("Time factor is %f\n", nps_main.host_time_factor);
      }
      gettimeofday(&tv_now, NULL);
      t2 = time_to_double(&tv_now);
      // add the pause to initial real time
      irt += t2 - t1;
      nps_main.real_initial_time += t2 - t1;
      // convert to scaled initial real time
      nps_main.scaled_initial_time = t2 - (t2 - irt) / nps_main.host_time_factor;
      pauseSignal = 0;
    }

    clock_get_current_time(&requestStart); // init measurement (after the pause signal)

    gettimeofday(&tv_now, NULL);
    host_time_now = time_to_double(&tv_now);
    double host_time_elapsed = nps_main.host_time_factor * (host_time_now  - nps_main.scaled_initial_time);

#if DEBUG_NPS_TIME
    printf("%f,%f,%f,%f,%f,%f,", nps_main.host_time_factor, host_time_elapsed, host_time_now, nps_main.scaled_initial_time,
           nps_main.sim_time, nps_main.display_time);
#endif

    int cnt = 0;
    static int prev_cnt = 0;
    static int grow_cnt = 0;
    while (nps_main.sim_time <= host_time_elapsed) {
      pthread_mutex_lock(&fdm_mutex);
      nps_main_run_sim_step();
      nps_main.sim_time += SIM_DT;
      pthread_mutex_unlock(&fdm_mutex);
      cnt++;
    }

    // Check to make sure the simulation doesn't get too far behind real time looping
    if (cnt > (prev_cnt)) {grow_cnt++;}
    else { grow_cnt--;}
    if (grow_cnt < 0) {grow_cnt = 0;}
    prev_cnt = cnt;

    if (grow_cnt > 10) {
      printf("Warning: The time factor is too large for efficient operation! Please reduce the time factor.\n");
    }

#if DEBUG_NPS_TIME
    printf("%f,%f\n", nps_main.sim_time, nps_main.display_time);
#endif

    clock_get_current_time(&requestEnd); // end measurement

    // Calculate time it took
    task_ns = (requestEnd.tv_sec - requestStart.tv_sec) * 1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    if (task_ns > 0) {
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

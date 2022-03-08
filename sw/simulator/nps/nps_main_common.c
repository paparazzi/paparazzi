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

#include "nps_main.h"
#include <signal.h>
#include <stdio.h>
#include <getopt.h>

#include "nps_flightgear.h"

#include "nps_ivy.h"

#ifdef __MACH__
pthread_mutex_t clock_mutex; // mutex for clock
void clock_get_current_time(struct timespec *ts)
{
  pthread_mutex_lock(&clock_mutex);
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;
  pthread_mutex_unlock(&clock_mutex);
}
#endif

void tstp_hdl(int n __attribute__((unused)))
{
  if (pauseSignal) {
    pauseSignal = 0;
    signal(SIGTSTP, SIG_DFL);
    raise(SIGTSTP);
  } else {
    pauseSignal = 1;
  }
}


void cont_hdl(int n __attribute__((unused)))
{
  signal(SIGCONT, cont_hdl);
  signal(SIGTSTP, tstp_hdl);
  printf("Press <enter> to continue.\n");
}


double time_to_double(struct timeval *t)
{
  return ((double)t->tv_sec + (double)(t->tv_usec * 1e-6));
}

double ntime_to_double(struct timespec *t)
{
  return ((double)t->tv_sec + (double)(t->tv_nsec * 1e-9));
}

int nps_main_init(int argc, char **argv)
{
  pauseSignal = 0;

  if (!nps_main_parse_options(argc, argv)) { return 1; }

  /* disable buffering for stdout,
   * so it properly works in paparazzi center
   * where it is not detected as interactive
   * and hence fully buffered instead of line buffered
   */
  setbuf(stdout, NULL);


  nps_main.sim_time = 0.;
  nps_main.display_time = 0.;
  struct timeval t;
  gettimeofday(&t, NULL);
  nps_main.real_initial_time = time_to_double(&t);
  nps_main.scaled_initial_time = time_to_double(&t);

  nps_fdm_init(SIM_DT);
  nps_atmosphere_init();
  nps_sensors_init(nps_main.sim_time);
  printf("Simulating with dt of %f\n", SIM_DT);

  nps_radio_and_autopilot_init();

#if DEBUG_NPS_TIME
  printf("host_time_factor,host_time_elapsed,host_time_now,scaled_initial_time,sim_time_before,display_time_before,sim_time_after,display_time_after\n");
#endif

  signal(SIGCONT, cont_hdl);
  signal(SIGTSTP, tstp_hdl);
  printf("Time factor is %f. (Press Ctrl-Z to change)\n", nps_main.host_time_factor);

  return 0;
}


void nps_set_time_factor(float time_factor)
{
  if (time_factor < 0.0 || time_factor > 100.0) {
    return;
  }
  if (fabs(nps_main.host_time_factor - time_factor) < 0.01) {
    return;
  }

  struct timeval tv_now;
  double t_now, t_elapsed;

  gettimeofday(&tv_now, NULL);
  t_now = time_to_double(&tv_now);

  /* "virtual" elapsed time with old time factor */
  t_elapsed = (t_now - nps_main.scaled_initial_time) * nps_main.host_time_factor;

  /* set new time factor */
  nps_main.host_time_factor = time_factor;
  printf("Time factor is %f\n", nps_main.host_time_factor);
  fflush(stdout);

  /* set new "virtual" scaled initial time using new time factor*/
  nps_main.scaled_initial_time = t_now - t_elapsed / nps_main.host_time_factor;
}


bool nps_main_parse_options(int argc, char **argv)
{

  nps_main.fg_host = NULL;
  nps_main.fg_port = 5501;
  nps_main.fg_port_in = 5502;
  nps_main.fg_time_offset = 0;
  nps_main.js_dev = NULL;
  nps_main.spektrum_dev = NULL;
  nps_main.rc_script = 0;
  nps_main.norc = false;
  nps_main.ivy_bus = NULL;
  nps_main.host_time_factor = 1.0;
  nps_main.fg_fdm = 0;
  nps_main.nodisplay = false;

  static const char *usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h                                     Display this help\n"
    "   --fg_host <flight gear host>           e.g. 127.0.0.1\n"
    "   --fg_port <flight gear port>           e.g. 5501\n"
    "   --fg_port_in <flight gear in port>     e.g. 5502\n"
    "   --fg_time_offset <offset in seconds>   e.g. 21600 for 6h\n"
    "   -j --js_dev <optional joystick index>  e.g. 1 (default 0)\n"
    "   --spektrum_dev <spektrum device>       e.g. /dev/ttyUSB0\n"
    "   --rc_script <number>                   e.g. 0\n"
    "   --norc                                 e.g. disable RC\n"
    "   --ivy_bus <ivy bus>                    e.g. 127.255.255.255\n"
    "   --time_factor <factor>                 e.g. 2.5\n"
    "   --nodisplay                            e.g. disable NPS ivy messages\n"
    "   --fg_fdm";


  while (1) {

    static struct option long_options[] = {
      {"fg_host", 1, NULL, 0},
      {"fg_port", 1, NULL, 0},
      {"fg_time_offset", 1, NULL, 0},
      {"js_dev", 2, NULL, 0},
      {"spektrum_dev", 1, NULL, 0},
      {"rc_script", 1, NULL, 0},
      {"norc", 0, NULL, 0},
      {"ivy_bus", 1, NULL, 0},
      {"time_factor", 1, NULL, 0},
      {"fg_fdm", 0, NULL, 0},
      {"fg_port_in", 1, NULL, 0},
      {"nodisplay", 0, NULL, 0},
      {0, 0, 0, 0}
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "jh",
                        long_options, &option_index);
    if (c == -1) {
      break;
    }

    switch (c) {
      case 0:
        switch (option_index) {
          case 0:
            nps_main.fg_host = strdup(optarg); break;
          case 1:
            nps_main.fg_port = atoi(optarg); break;
          case 2:
            nps_main.fg_time_offset = atoi(optarg); break;
          case 3:
            if (optarg == NULL) {nps_main.js_dev = strdup("0");}
            else {nps_main.js_dev = strdup(optarg);}
            break;
          case 4:
            nps_main.spektrum_dev = strdup(optarg); break;
          case 5:
            nps_main.rc_script = atoi(optarg); break;
          case 6:
            nps_main.norc = true; break;
          case 7:
            nps_main.ivy_bus = strdup(optarg); break;
          case 8:
            nps_main.host_time_factor = atof(optarg); break;
          case 9:
            nps_main.fg_fdm = 1; break;
          case 10:
            nps_main.fg_port_in = atoi(optarg); break;
          case 11:
            nps_main.nodisplay = true; break;
          default:
            break;
        }
        break;

      case 'j':
        if (optarg == NULL) {nps_main.js_dev = strdup("0");}
        else {nps_main.js_dev = strdup(optarg);}
        break;

      case 'h':
        fprintf(stderr, usage, argv[0]);
        exit(0);

      default:
        printf("?? getopt returned character code 0%o ??\n", c);
        fprintf(stderr, usage, argv[0]);
        exit(EXIT_FAILURE);
    }
  }
  return TRUE;
}


void *nps_flight_gear_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = DISPLAY_DT * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds


  nps_flightgear_init(nps_main.fg_host, nps_main.fg_port, nps_main.fg_port_in, nps_main.fg_time_offset);

  while (TRUE) {
    clock_get_current_time(&requestStart);

    pthread_mutex_lock(&fdm_mutex);
    if (nps_main.fg_host) {
      if (nps_main.fg_fdm) {
        nps_flightgear_send_fdm();
      } else {
        nps_flightgear_send();
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
      printf("FG THREAD: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }

  return(NULL);
}



void *nps_main_display(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = 3 * DISPLAY_DT * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  nps_ivy_init(nps_main.ivy_bus);

  // start the loop only if no_display is false
  if (!nps_main.nodisplay) {
    while (TRUE) {
      clock_get_current_time(&requestStart);

      nps_ivy_display(&fdm, &sensors);

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
        printf("IVY DISPLAY THREAD: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
               (double)task_ns / 1E6, (double)period_ns / 1E6);
  #endif
      }
    }
  }
  return(NULL);
}

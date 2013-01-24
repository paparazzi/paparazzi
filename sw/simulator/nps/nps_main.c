/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
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
#include <signal.h>
#include <glib.h>
#include <sys/time.h>
#include <getopt.h>

#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_autopilot.h"
#include "nps_ivy.h"
#include "nps_flightgear.h"

#include "mcu_periph/sys_time.h"
#define SIM_DT     (SYS_TIME_RESOLUTION)
#define DISPLAY_DT (1./30.)
#define HOST_TIMEOUT_MS 40
#define HOST_TIME_FACTOR 1.

static struct {
  double real_initial_time;
  double scaled_initial_time;
  double host_time_factor;
  double sim_time;
  double display_time;
  char* fg_host;
  unsigned int fg_port;
  unsigned int fg_time_offset;
  char* js_dev;
  char* spektrum_dev;
  int rc_script;
  char* ivy_bus;
} nps_main;

static bool_t nps_main_parse_options(int argc, char** argv);
static void nps_main_init(void);
static void nps_main_display(void);
static void nps_main_run_sim_step(void);
static gboolean nps_main_periodic(gpointer data __attribute__ ((unused)));

int pauseSignal = 0;

void tstp_hdl(int n __attribute__ ((unused))) {
  if (pauseSignal) {
    pauseSignal = 0;
    signal (SIGTSTP, SIG_DFL);
    raise(SIGTSTP);
  } else {
    pauseSignal = 1;
  }
}

void cont_hdl (int n __attribute__ ((unused))) {
   signal (SIGCONT, cont_hdl);
   signal (SIGTSTP, tstp_hdl);
   printf("Press <enter> to continue.\n");
 }

double time_to_double(struct timeval *t) {
    return ((double)t->tv_sec + (double)(t->tv_usec * 1e-6));
}

int main ( int argc, char** argv) {

  if (!nps_main_parse_options(argc, argv)) return 1;

  /* disable buffering for stdout,
   * so it properly works in paparazzi center
   * where it is not detected as interactive
   * and hence fully buffered instead of line buffered
   */
  setbuf(stdout, NULL);

  nps_main_init();

  signal(SIGCONT, cont_hdl);
  signal(SIGTSTP, tstp_hdl);
  printf("Time factor is %f. (Press Ctrl-Z to change)\n", nps_main.host_time_factor);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  g_timeout_add(HOST_TIMEOUT_MS, nps_main_periodic, NULL);
  g_main_loop_run(ml);

  return 0;
}


static void nps_main_init(void) {

  nps_main.sim_time = 0.;
  nps_main.display_time = 0.;
  struct timeval t;
  gettimeofday (&t, NULL);
  nps_main.real_initial_time = time_to_double(&t);
  nps_main.scaled_initial_time = time_to_double(&t);
  nps_main.host_time_factor = HOST_TIME_FACTOR;

  nps_ivy_init(nps_main.ivy_bus);
  nps_fdm_init(SIM_DT);
  nps_sensors_init(nps_main.sim_time);

  enum NpsRadioControlType rc_type;
  char* rc_dev = NULL;
  if (nps_main.js_dev) {
    rc_type = JOYSTICK;
    rc_dev = nps_main.js_dev;
  }
  else if (nps_main.spektrum_dev) {
    rc_type = SPEKTRUM;
    rc_dev = nps_main.spektrum_dev;
  }
  else {
    rc_type = SCRIPT;
  }
  nps_autopilot_init(rc_type, nps_main.rc_script, rc_dev);

  if (nps_main.fg_host)
    nps_flightgear_init(nps_main.fg_host, nps_main.fg_port, nps_main.fg_time_offset);

#if DEBUG_NPS_TIME
  printf("host_time_factor,host_time_elapsed,host_time_now,scaled_initial_time,sim_time_before,display_time_before,sim_time_after,display_time_after\n");
#endif

}



static void nps_main_run_sim_step(void) {
  //  printf("sim at %f\n", nps_main.sim_time);

  nps_autopilot_run_systime_step();

  nps_fdm_run_step(autopilot.commands);

  nps_sensors_run_step(nps_main.sim_time);

  nps_autopilot_run_step(nps_main.sim_time);

}


static void nps_main_display(void) {
  //  printf("display at %f\n", nps_main.display_time);
  nps_ivy_display();
  if (nps_main.fg_host)
    nps_flightgear_send();
}


static gboolean nps_main_periodic(gpointer data __attribute__ ((unused))) {
  struct timeval tv_now;
  double  host_time_now;

  if (pauseSignal) {
    char line[128];
    double tf = 1.0;
    double t1, t2, irt;

    gettimeofday(&tv_now, NULL);
    t1 = time_to_double(&tv_now);
    /* unscale to initial real time*/
    irt = t1 - (t1 - nps_main.scaled_initial_time)*nps_main.host_time_factor;

    printf("Press <enter> to continue (or CTRL-Z to suspend).\nEnter a new time factor if needed (current: %f): ", nps_main.host_time_factor);
    fflush(stdout);
    if (fgets(line,127,stdin)) {
      if ((sscanf(line," %le ", &tf) == 1)) {
        if (tf > 0 && tf < 1000)
          nps_main.host_time_factor = tf;
      }
    printf("Time factor is %f\n", nps_main.host_time_factor);
    }
    gettimeofday(&tv_now, NULL);
    t2 = time_to_double(&tv_now);
    /* add the pause to initial real time */
    irt += t2 - t1;
    nps_main.real_initial_time += t2 - t1;
    /* convert to scaled initial real time */
    nps_main.scaled_initial_time = t2 - (t2 - irt)/nps_main.host_time_factor;
    pauseSignal = 0;
  }

  gettimeofday (&tv_now, NULL);
  host_time_now = time_to_double(&tv_now);
  double host_time_elapsed = nps_main.host_time_factor *(host_time_now  - nps_main.scaled_initial_time);

#if DEBUG_NPS_TIME
  printf("%f,%f,%f,%f,%f,%f,",nps_main.host_time_factor,host_time_elapsed,host_time_now,nps_main.scaled_initial_time,nps_main.sim_time,nps_main.display_time);
#endif

  int cnt = 0;
  static int prev_cnt = 0;
  static int grow_cnt = 0;
  while (nps_main.sim_time <= host_time_elapsed) {
    nps_main_run_sim_step();
    nps_main.sim_time += SIM_DT;
    if (nps_main.display_time < (host_time_now - nps_main.real_initial_time)) {
      nps_main_display();
      nps_main.display_time += DISPLAY_DT;
    }
    cnt++;
  }

  /* Check to make sure the simulation doesn't get too far behind real time looping */
  if (cnt > (prev_cnt)) {grow_cnt++;}
  else { grow_cnt--;}
  if (grow_cnt < 0) {grow_cnt = 0;}
  prev_cnt = cnt;

  if (grow_cnt > 10) {
    printf("Warning: The time factor is too large for efficient operation! Please reduce the time factor.\n");
  }

#if DEBUG_NPS_TIME
  printf("%f,%f\n",nps_main.sim_time,nps_main.display_time);
#endif

  return TRUE;

}


static bool_t nps_main_parse_options(int argc, char** argv) {

  nps_main.fg_host = NULL;
  nps_main.fg_port = 5501;
  nps_main.fg_time_offset = 0;
  nps_main.js_dev = NULL;
  nps_main.spektrum_dev = NULL;
  nps_main.rc_script = 0;
  nps_main.ivy_bus = NULL;

  static const char* usage =
"Usage: %s [options]\n"
" Options :\n"
"   -h                                     Display this help\n"
"   --fg_host <flight gear host>           e.g. 127.0.0.1\n"
"   --fg_port <flight gear port>           e.g. 5501\n"
"   --fg_time_offset <offset in seconds>   e.g. 21600 for 6h\n"
"   -j --js_dev <optional joystick index>  e.g. 1 (default 0)\n"
"   --spektrum_dev <spektrum device>       e.g. /dev/ttyUSB0\n"
"   --rc_script <number>                   e.g. 0\n"
"   --ivy_bus <ivy bus>                    e.g. 127.255.255.255\n";


  while (1) {

    static struct option long_options[] = {
      {"fg_host", 1, NULL, 0},
      {"fg_port", 1, NULL, 0},
      {"fg_time_offset", 1, NULL, 0},
      {"js_dev", 2, NULL, 0},
      {"spektrum_dev", 1, NULL, 0},
      {"rc_script", 1, NULL, 0},
      {"ivy_bus", 1, NULL, 0},
      {0, 0, 0, 0}
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "jh",
                        long_options, &option_index);
    if (c == -1)
      break;

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
            nps_main.ivy_bus = strdup(optarg); break;
        }
        break;

      case 'j':
        if (optarg == NULL) {nps_main.js_dev = strdup("0");}
        else {nps_main.js_dev = strdup(optarg);}
        break;

      case 'h':
        fprintf(stderr, usage, argv[0]);
        exit(0);

      default: /* $B!G(B?$B!G(B */
        printf("?? getopt returned character code 0%o ??\n", c);
        fprintf(stderr, usage, argv[0]);
        exit(EXIT_FAILURE);
    }
  }
  return TRUE;
}

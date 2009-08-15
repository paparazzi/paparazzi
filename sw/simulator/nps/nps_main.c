#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <sys/time.h>
#include <getopt.h>

#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_autopilot.h"
#include "nps_ivy.h"
#include "nps_flightgear.h"

#define SIM_DT     (1./512.)
#define DISPLAY_DT (1./30.)
#define HOST_TIMEOUT_MS 40
#define HOST_TIME_FACTOR 1.

static struct {
  double scaled_initial_time;
  double host_time_factor;
  double sim_time;
  double display_time;
  char* fg_host;
  unsigned int fg_port;
  char* js_dev;
  char* spektrum_dev;
  int rc_script;
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

double time_to_double(timeval *t) {
	return ((double)t->tv_sec + (double)(t->tv_usec * 1e-6));
}

int main ( int argc, char** argv) {

  if (!nps_main_parse_options(argc, argv)) return 1;

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
  timeval t;
  gettimeofday (&t, NULL);
  nps_main.scaled_initial_time = time_to_double(&t);
  nps_main.host_time_factor = HOST_TIME_FACTOR;

  nps_ivy_init();
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
    nps_flightgear_init(nps_main.fg_host, nps_main.fg_port);

}



static void nps_main_run_sim_step(void) {


  //  printf("sim at %f\n", nps_main.sim_time);

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
	/* convert to scaled initial real time */
	nps_main.scaled_initial_time = t2 - (t2 - irt)/nps_main.host_time_factor;
	pauseSignal = 0;
  }

  gettimeofday (&tv_now, NULL);
  host_time_now = time_to_double(&tv_now);
  double host_time_elapsed = nps_main.host_time_factor *(host_time_now  - nps_main.scaled_initial_time);

  while (nps_main.sim_time <= host_time_elapsed) {
    nps_main_run_sim_step();
    nps_main.sim_time += SIM_DT;
    if (nps_main.display_time < nps_main.sim_time) {
      nps_main_display();
      nps_main.display_time += DISPLAY_DT;
    }
  }

  return TRUE;

}


static bool_t nps_main_parse_options(int argc, char** argv) {

  nps_main.fg_host = NULL;
  nps_main.fg_port = 5501;
  nps_main.js_dev = NULL;
  nps_main.spektrum_dev = NULL;
  nps_main.rc_script = 0;

  static const char* usage =
"Usage: %s [options]\n"
" Options :\n"
"   --fg_host flight gear host\n"
"   --fg_port flight gear port\n"
"   -j --js_dev joystick device\n"
"   --spektrum_dev spektrum device\n"
"   --rc_script no\n";


  while (1) {

    static struct option long_options[] = {
      {"fg_host", 1, NULL, 0},
      {"fg_port", 1, NULL, 0},
      {"js_dev", 1, NULL, 0},
      {"spektrum_dev", 1, NULL, 0},
      {"rc_script", 1, NULL, 0},
      {0, 0, 0, 0}
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "j:",
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
	nps_main.js_dev = strdup(optarg); break;
      case 3:
	nps_main.spektrum_dev = strdup(optarg); break;
      case 4:
	nps_main.rc_script = atoi(optarg); break;
      }
      break;

    case 'j':
      nps_main.js_dev = strdup(optarg);
      break;

    default: /* ’?’ */
      printf("?? getopt returned character code 0%o ??\n", c);
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  return TRUE;
}

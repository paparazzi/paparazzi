#ifndef NPS_MAIN_H
#define NPS_MAIN_H

#include <pthread.h>
#include <sys/time.h>
#include "nps_fdm.h"
#include "mcu_periph/sys_time.h"
#include "nps_atmosphere.h"
#include "nps_sensors.h"
#include "nps_autopilot.h"

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
#include <mach/clock.h>
#include <mach/mach.h>
void clock_get_current_time(struct timespec *ts);
#else // Linux
#define clock_get_current_time(_x) clock_gettime(CLOCK_REALTIME, _x)
#endif // #ifdef __MACH__

#define SIM_DT     (1./SYS_TIME_FREQUENCY)
#define DISPLAY_DT (1./30.)
#define HOST_TIMEOUT_MS 40

extern pthread_t th_flight_gear; // sends/receives flight gear packets
extern pthread_t th_display_ivy; // sends Ivy messages
extern pthread_t th_main_loop; // handles simulation

extern pthread_mutex_t fdm_mutex; // mutex for fdm data

extern int pauseSignal; // for catching SIGTSTP

extern bool nps_main_parse_options(int argc, char **argv);

extern int nps_main_init(int argc, char **argv);
extern void nps_radio_and_autopilot_init(void);
extern void nps_main_run_sim_step(void);
extern void nps_set_time_factor(float time_factor);

extern void* nps_main_loop(void* data __attribute__((unused)));
extern void* nps_flight_gear_loop(void* data __attribute__((unused)));
extern void* nps_main_display(void* data __attribute__((unused)));

extern void tstp_hdl(int n __attribute__((unused)));
extern void cont_hdl(int n __attribute__((unused)));

extern double time_to_double(struct timeval *t);
extern double ntime_to_double(struct timespec *t);

void nps_update_launch_from_dl(uint8_t value);

struct NpsMain {
  double real_initial_time;
  double scaled_initial_time;
  double host_time_factor;
  double sim_time;
  double display_time;
  char *fg_host;
  unsigned int fg_port;
  unsigned int fg_port_in;
  unsigned int fg_time_offset;
  int fg_fdm;
  char *js_dev;
  char *spektrum_dev;
  int rc_script;
  bool norc;
  char *ivy_bus;
  bool nodisplay;
};

extern struct NpsMain nps_main;

#endif /* NPS_MAIN_H */

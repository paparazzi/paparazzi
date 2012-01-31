/* Definitions and declarations required to compile autopilot code on a
   i386 architecture. Bindings for OCaml. */

#define MODULES_C

#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>
#include "std.h"
#include "inter_mcu.h"
#include "autopilot.h"
#include "estimator.h"
#include "subsystems/gps.h"
#include "subsystems/navigation/traffic_info.h"
#include "generated/settings.h"
#include "subsystems/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "commands.h"
#include "firmwares/fixedwing/main_ap.h"
#include "ap_downlink.h"
#include "sim_uart.h"
#include "subsystems/datalink/datalink.h"
#include "generated/flight_plan.h"

#include "generated/modules.h"

#include <caml/mlvalues.h>
#include <caml/memory.h>


/* Dummy definitions to replace the ones from the files not compiled in the
   simulator */
uint8_t ir_estim_mode;
uint8_t vertical_mode;
uint8_t inflight_calib_mode;
bool_t rc_event_1, rc_event_2;
bool_t launch;
uint8_t gps_nb_ovrn, link_fbw_fbw_nb_err, link_fbw_nb_err;
float alt_roll_pgain;
float roll_rate_pgain;
bool_t gpio1_status;
uint16_t datalink_time = 0;



uint8_t ac_id;

value sim_periodic_task(value unit) {
  sensors_task();
  attitude_loop();
  reporting_task();
  modules_periodic_task();
  periodic_task_fbw();
  event_task_ap();
  event_task_fbw();
  return unit;
}

value sim_monitor_task(value unit) {
  monitor_task();
  return unit;
}

value sim_nav_task(value unit) {
  navigation_task();
  return unit;
}


float ftimeofday(void) {
  struct timeval t;
  struct timezone z;
  gettimeofday(&t, &z);
  return (t.tv_sec + t.tv_usec/1e6);
}

value sim_init(value unit) {
  init_fbw();
  init_ap();
#ifdef SIM_UART
  /* open named pipe */
  char link_pipe_name[128];
#ifdef SIM_XBEE
  sprintf(link_pipe_name, "/tmp/pprz_xbee");
#else
  sprintf(link_pipe_name, "/tmp/pprz_link_%d", AC_ID);
#endif
  struct stat st;
  if (stat(link_pipe_name, &st)) {
    if (mkfifo(link_pipe_name, 0644) == -1) {
      perror("make pipe");
      exit (10);
    }
  }
  if ( !(pipe_stream = fopen(link_pipe_name, "w")) ) {
    perror("open pipe");
    exit (10);
  }
#endif

  return unit;
}

value update_bat(value bat) {
  electrical.vsupply = Int_val(bat);
  return Val_unit;
}


value get_commands(value val_commands) {
  int i;

  for(i=0; i < COMMANDS_NB; i++)
    Store_field(val_commands, i, Val_int(commands[i]));

  return Val_int(commands[COMMAND_THROTTLE]);
}

value set_datalink_message(value s) {
  int n = string_length(s);
  char *ss = String_val(s);
  assert(n <= MSG_SIZE);

  int i;
  for(i = 0; i < n; i++)
    dl_buffer[i] = ss[i];

  dl_parse_msg();
  return Val_unit;
}

/** Required by electrical */
void adc_buf_channel(void* a __attribute__ ((unused)),
		     void* b __attribute__ ((unused)),
		     void* c __attribute__ ((unused))) {
}

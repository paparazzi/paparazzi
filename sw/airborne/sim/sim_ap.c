/* Definitions and declarations required to compile autopilot code on a
   i386 architecture. Bindings for OCaml. */

#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <time.h>
#include "std.h"
#include "inter_mcu.h"
#include "autopilot.h"
#include "estimator.h"
#include "gps.h"
#include "traffic_info.h"
#include "flight_plan.h"
#include "nav.h"
#include "pid.h"
#include "infrared.h"
#include "cam.h"
#include "commands.h"
#include "main_ap.h"

#include <caml/mlvalues.h>
#include <caml/memory.h>

uint8_t ir_estim_mode;
uint8_t vertical_mode;
uint8_t inflight_calib_mode;
bool_t rc_event_1, rc_event_2;
bool_t launch;
uint8_t gps_nb_ovrn, modem_nb_ovrn, link_fbw_fbw_nb_err, link_fbw_nb_err;


uint8_t ac_id;

value sim_periodic_task(value _unit __attribute__ ((unused))) {
  periodic_task_ap();
  return Val_unit;
}


float ftimeofday(void) {
  struct timeval t;
  struct timezone z;
  gettimeofday(&t, &z);
  return (t.tv_sec + t.tv_usec/1e6);
}

value sim_init(value id) {
  init_ap();
  ac_id = Int_val(id);
  return Val_unit;
}

value update_bat(value bat) {
  fbw_vsupply_decivolt = Int_val(bat);  
  return Val_unit;
}


value get_commands(value val_commands) {
  int i;

  for(i=0; i < COMMANDS_NB; i++)
    Store_field(val_commands, i, Val_int(commands[i]));

  return Val_int(commands[COMMAND_THROTTLE]);
}

value set_ac_info_native(value ac_id, value ux, value uy, value course, value alt, value gspeed) {
  SetAcInfo(Int_val(ac_id), Double_val(ux), Double_val(uy), 
	    Double_val(course), Double_val(alt), Double_val(gspeed));
  return Val_unit;
}

value set_ac_info(value * argv, int argn) {
  assert (argn == 6);
  return set_ac_info_native(argv[0], argv[1], argv[2], argv[3],argv[4], argv[5]);
}



value move_waypoint(value wp_id, value ux, value uy, value a) {
  MoveWaypoint(Int_val(wp_id), Double_val(ux), Double_val(uy), Double_val(a));
  return Val_unit;
}

value goto_block(value block_id) {
  nav_goto_block(Int_val(block_id));
  return Val_unit;
}

value send_event(value event_id) {
  uint8_t event = Int_val(event_id);
  switch (event) {
    case 1 : rc_event_1 = TRUE; break; // FIXME !
    case 2 : rc_event_2 = TRUE; break;
    default: ;
  }
  return Val_unit;
}

value dl_setting(value index __attribute__ ((unused)), 
		 value val __attribute__ ((unused))) {
  /** DlSetting macro may be empty: unused attr to get rif of the warning */
  DlSetting(Int_val(index), Double_val(val));
  return Val_unit;
}

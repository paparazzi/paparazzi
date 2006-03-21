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

#include <caml/mlvalues.h>
#include <caml/memory.h>

uint8_t ir_estim_mode;
uint8_t vertical_mode;
uint8_t inflight_calib_mode;
bool_t rc_event_1, rc_event_2;
bool_t launch;
bool_t link_fbw_receive_valid;
uint8_t gps_nb_ovrn, modem_nb_ovrn, link_fbw_fbw_nb_err, link_fbw_nb_err;


uint8_t ac_id;

inter_mcu_msg from_fbw, from_ap;

static int16_t values_from_ap[RADIO_CTL_NB];

uint16_t ppm_pulses[ PPM_NB_PULSES ]; /** From ppm_hw.c */

value sim_periodic_task(value _unit) {
  periodic_task();
  return Val_unit;
}

static int radio_status = 1;
static int radio_really_lost;

value set_radio_status(value on) {
  radio_status = Int_val(on);
  if (! radio_status) radio_really_lost = FALSE;
  return Val_unit;
}


value set_really_lost(value on) {
  radio_really_lost = Int_val(on);
  return Val_unit;
}

value sim_rc_task(value _unit) {
  NormalizePpm(); /** -> rc_values */
  /***  printf("update: %d : %f (%d)\n", Int_val(c), Double_val(v), rc_values[COMMAND_GAIN1]); ***/
  int i;
  for(i = 0; i < COMMANDS_NB; i++)
    from_fbw.from_fbw.channels[i] = rc_values[i];

  from_fbw.from_fbw.status = (radio_status << STATUS_RADIO_OK) | (radio_really_lost << RADIO_REALLY_LOST) | (1 << AVERAGED_CHANNELS_SENT);
  link_fbw_receive_valid = TRUE;
  telecommand_task();
  return Val_unit;
}


float ftimeofday(void) {
  struct timeval t;
  struct timezone z;
  gettimeofday(&t, &z);
  return (t.tv_sec + t.tv_usec/1e6);
}

value sim_init(value id) {
  pprz_mode = PPRZ_MODE_MANUAL;
  estimator_init();
  ir_init();
  ac_id = Int_val(id);
  return Val_unit;
}

value update_bat(value bat) {
  from_fbw.from_fbw.vsupply = Int_val(bat);  
  return Val_unit;
}

value update_rc_channel(value c, value v) {
  ppm_pulses[Int_val(c)] = Double_val(v);
  return Val_unit;
}

// Defined in servo.c
#define _4017_NB_CHANNELS 10
#define SERVO_NEUTRAL_(i) SERVOS_NEUTRALS_ ## i
#define SERVO_NEUTRAL(i) (SERVO_NEUTRAL_(i))
#define SERVO_MIN (SERVO_MIN_US)
#define SERVO_MAX (SERVO_MAX_US)
#define ChopServo(x, min, max) ((x) < min ? min : ((x) > max ? max : (x)))

#ifdef SERVO_MOTOR_RIGHT
#define SERVO_GAZ SERVO_MOTOR_RIGHT
#endif

#define COMMAND(i) servo_widths[i]

value set_servos(value servos) {
  int i;

  /** Get values computed by the autopilot */
  for(i = 0; i < RADIO_CTL_NB; i++) {
    values_from_ap[i] =  US_OF_CLOCK(from_ap.from_ap.channels[i]);
    /***printf("%d:%d\n", i, values_from_ap[i]); ***/
  }
  

  uint16_t servo_widths[_4017_NB_CHANNELS];
  CommandsSet(values_from_ap);

  for(i=0; i < _4017_NB_CHANNELS; i++)
    Store_field(servos, i, Val_int(servo_widths[i]));

  return Val_int(servo_widths[SERVO_GAZ]);
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

value dl_setting(value index, value val) {
#if defined DlSetting
  DlSetting(Int_val(index), Double_val(val));
#endif
  return Val_unit;
}

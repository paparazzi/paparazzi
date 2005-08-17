/* Definitions and declarations required to compile autopilot code on a
   i386 architecture. Binding for OCaml. */

#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "std.h"
#include "link_autopilot.h"
#include "autopilot.h"
#include "estimator.h"
#include "gps.h"

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

struct inter_mcu_msg from_fbw, to_fbw;

static int16_t values_from_ap[RADIO_CTL_NB];

void inflight_calib(void) { }

void link_fbw_send(void) {
  int i;
  for(i = 0; i < RADIO_CTL_NB; i++)
    values_from_ap[i] =  to_fbw.channels[i] / CLOCK;
}

value sim_periodic_task(value unit) {
  periodic_task();
  return Val_unit;
}


value sim_rc_task(value unit) {
  from_fbw.status = (1 << STATUS_RADIO_OK) | (1 << AVERAGED_CHANNELS_SENT);
  link_fbw_receive_valid = TRUE;
  radio_control_task();
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
  ac_id = Int_val(id);
  return Val_unit;
}

value update_bat(value bat) {
  from_fbw.vsupply = Int_val(bat);  
  return Val_unit;
}

value update_rc_channel(value c, value v) {
  from_fbw.channels[Int_val(c)] = Double_val(v)*MAX_PPRZ;
  return Val_unit;
}

// Defined in servo.c
#define _4017_NB_CHANNELS 10
#define SERVO_NEUTRAL_(i) SERVOS_NEUTRALS_ ## i
#define SERVO_NEUTRAL(i) (SERVO_NEUTRAL_(i))
#define SERVO_MIN (SERVO_MIN_US)
#define SERVO_MAX (SERVO_MAX_US)
#define ChopServo(x) ((x) < SERVO_MIN ? SERVO_MIN : ((x) > SERVO_MAX ? SERVO_MAX : (x)))

value set_servos(value servos) {
  int i;

  uint16_t servo_widths[_4017_NB_CHANNELS];
  ServoSet(values_from_ap);

  for(i=0; i < _4017_NB_CHANNELS; i++)
    Store_field(servos, i, Val_int(servo_widths[i]));

  return Val_int(servo_widths[SERVO_GAZ]);
}

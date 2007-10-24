#include <glib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "booz_flight_model.h"
#include "booz_flightgear.h"



char* fg_host = "127.0.0.1";
//char* fg_host = "10.31.4.107";
unsigned int fg_port = 5501;

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;


double foo_commands[] = {0., 0., 0., 0.};


static gboolean timeout_callback(gpointer data);

static void airborne_init(void);
static void airborne_periodic_task(void);
static void airborne_event_task(void);

#include "booz_estimator.h"

static gboolean timeout_callback(gpointer data) {

  


  booz_flight_model_run(DT, foo_commands);
  sim_time += DT;

  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    booz_flightgear_send();
  }

  booz_estimator_p = bfm.state->ve[BFMS_P];
  booz_estimator_q = bfm.state->ve[BFMS_Q];
  booz_estimator_r = bfm.state->ve[BFMS_R];

  booz_estimator_phi = bfm.state->ve[BFMS_PHI];
  booz_estimator_theta = bfm.state->ve[BFMS_THETA];
  booz_estimator_psi = bfm.state->ve[BFMS_PSI];

  airborne_event_task();
  airborne_periodic_task();


  return TRUE;
}



int main ( int argc, char** argv) {

  sim_time = 0.;
  disp_time = 0.;
  
  booz_flight_model_init();

  booz_flightgear_init(fg_host, fg_port);


  airborne_init();


  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, timeout_callback, NULL);
  
  IvyInit ("BoozSim", "BoozSim READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);
 
  return 0;
}


#include "radio_control.h"
#include "actuators.h"
#include "commands.h"

#include "booz_telemetry.h"

#include "booz_estimator.h"
#include "booz_control.h"

// FIXME grrrrr!
uint32_t link_imu_nb_err;
uint8_t  link_imu_status;

int16_t trim_p = 0;
int16_t trim_q = 0;
int16_t trim_r = 0;

uint16_t ppm_pulses[PPM_NB_PULSES];


static void airborne_init(void) {

  ppm_pulses[RADIO_THROTTLE] = 1223 + 0.615 * (2050-1223);
  ppm_pulses[RADIO_ROLL] = 1500;
  ppm_pulses[RADIO_PITCH] = 1498;
  ppm_pulses[RADIO_YAW] = 1493;

  booz_estimator_init();
  booz_control_init();
  
}



static void airborne_periodic_task(void) {

  int foo = sim_time / 10;
  if (!(foo%2))
    ppm_pulses[RADIO_PITCH] = 1600;
  else
    ppm_pulses[RADIO_PITCH] = 1400;

  booz_autopilot_periodic_task();

  SetActuatorsFromCommands(commands);
  
  static uint8_t _50hz = 0;
  _50hz++;
  if (_50hz > 5) _50hz = 0;
  switch (_50hz) {
  case 1:
    radio_control_periodic_task();
    break;
  case 2:
    booz_telemetry_periodic_task();
   break;
  case 3:
    break;
  case 4:
    break;
  }


  foo_commands[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK] - 1200)/(double)(1850-1200);
  foo_commands[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 1200)/(double)(1850-1200);
  foo_commands[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 1200)/(double)(1850-1200);
  foo_commands[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT] - 1200)/(double)(1850-1200);

}


static void airborne_event_task(void) {
  //   if (ppm_valid) {
  //    ppm_valid = FALSE;
  radio_control_event_task();
  booz_autopilot_mode = BOOZ_AP_MODE_ATTITUDE;
  if (booz_autopilot_mode == BOOZ_AP_MODE_RATE)
    booz_control_rate_compute_setpoints();
  else if (booz_autopilot_mode == BOOZ_AP_MODE_ATTITUDE)
    booz_control_attitude_compute_setpoints();
  //}
}


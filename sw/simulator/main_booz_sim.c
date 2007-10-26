#include <glib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "booz_flight_model.h"
#include "booz_sensors_model.h"

#include "booz_flightgear.h"
#include "booz_joystick.h"
#include "main_booz.h"


//char* fg_host = "127.0.0.1";
char* fg_host = "10.31.4.107";
unsigned int fg_port = 5501;
char* joystick_dev = "/dev/input/js0";

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

double foo_commands[] = {0., 0., 0., 0.};

static gboolean timeout_callback(gpointer data);

#ifdef SIM_UART
static void sim_uart_init(void);
#endif
#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport
static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app, void *user_data, int argc, char *argv[]);
#endif


#include "booz_estimator.h"
#include "radio_control.h"
#include "actuators.h"
volatile bool_t ppm_valid;
#define RPM_OF_RAD_S(a) ((a)*60./M_PI)

static gboolean timeout_callback(gpointer data) {

  booz_flight_model_run(DT, foo_commands);
  booz_sensors_model_run();
  sim_time += DT;

  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    booz_flightgear_send();
    IvySendMsg("148 BOOZ_RPMS %f %f %f %f",  
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_F]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_B]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_L]),
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_R]) );
  }

  booz_estimator_p = bfm.state->ve[BFMS_P];
  booz_estimator_q = bfm.state->ve[BFMS_Q];
  booz_estimator_r = bfm.state->ve[BFMS_R];

  booz_estimator_phi = bfm.state->ve[BFMS_PHI];
  booz_estimator_theta = bfm.state->ve[BFMS_THETA];
  booz_estimator_psi = bfm.state->ve[BFMS_PSI];

  /* post a radio control event */
  ppm_pulses[RADIO_THROTTLE] = 1223 + booz_joystick_value[JS_THROTTLE] * (2050-1223);
  ppm_pulses[RADIO_PITCH]    = 1498 + booz_joystick_value[JS_PITCH]    * (2050-950);
  ppm_pulses[RADIO_ROLL]     = 1500 + booz_joystick_value[JS_ROLL]     * (2050-950);
  ppm_pulses[RADIO_YAW]      = 1500 + booz_joystick_value[JS_YAW]      * (2050-950);
  ppm_pulses[RADIO_MODE]     = 1500 + booz_joystick_value[JS_MODE]     * (2050-950);
  ppm_valid = TRUE;
  booz_main_event_task();
  /* run periodic tak */
  booz_main_periodic_task();
  /* get actuators values */
#if 0
  foo_commands[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK] - 1200)/(double)(1850-1200);
  foo_commands[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 1200)/(double)(1850-1200);
  foo_commands[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 1200)/(double)(1850-1200);
  foo_commands[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT] - 1200)/(double)(1850-1200);
#endif

  foo_commands[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK]   - 0)/(double)(255);
  foo_commands[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 0)/(double)(255);
  foo_commands[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 0)/(double)(255);
  foo_commands[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT]   - 0)/(double)(255);


  return TRUE;
}

int main ( int argc, char** argv) {

  sim_time = 0.;
  disp_time = 0.;
  
  booz_flight_model_init();

  booz_flightgear_init(fg_host, fg_port);

#ifdef SIM_UART
  sim_uart_init();
#endif 

#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport
  ivy_transport_init();
#endif

  booz_joystick_init(joystick_dev);

  booz_main_init();
  

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, timeout_callback, NULL);

  g_main_loop_run(ml);
 
  return 0;
}


/////////////////////
// Helpers
////////////////////

#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport

static void ivy_transport_init(void) {
  IvyInit ("BoozSim", "BoozSim READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

#include "std.h"
#include "settings.h"
#include "booz_telemetry.h"
static void on_DL_SETTING(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(&index, &value);
  printf("setting %d %f\n", index, value);
}

#endif



#ifdef SIM_UART
#define AC_ID 148
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "sim_uart.h"

static void sim_uart_init(void) {
  char link_pipe_name[128];
  sprintf(link_pipe_name, "/tmp/pprz_link_%d", AC_ID);
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
}
#endif /* SIM_UART */









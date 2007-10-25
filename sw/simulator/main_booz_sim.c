#include <glib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <math.h>

#include "booz_flight_model.h"
#include "booz_flightgear.h"



//char* fg_host = "127.0.0.1";
char* fg_host = "10.31.4.107";
unsigned int fg_port = 5501;

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

double foo_commands[] = {0., 0., 0., 0.};

static gboolean timeout_callback(gpointer data);
static gboolean on_js_data_received(GIOChannel *source, GIOCondition condition, gpointer data);

static void joystick_init(void);
#ifdef SIM_UART
static void sim_uart_init(void);
#endif
#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport
static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app, void *user_data, int argc, char *argv[]);
#endif

static void airborne_init(void);
static void airborne_periodic_task(void);
static void airborne_event_task(void);

#include "booz_estimator.h"
#include "radio_control.h"
volatile bool_t ppm_valid;
#define RPM_OF_RAD_S(a) ((a)*60./M_PI)

static gboolean timeout_callback(gpointer data) {

  booz_flight_model_run(DT, foo_commands);
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
  ppm_valid = TRUE;
  airborne_event_task();

  airborne_periodic_task();


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

  joystick_init();

  airborne_init();


  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, timeout_callback, NULL);
  

  g_main_loop_run(ml);
 
  return 0;
}


/////////////////////
// Helpers
////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

static void joystick_init(void) {
  const char* device = "/dev/input/js0";
  int fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
     printf("opening joystick serial device %s : %s\n", device, strerror(errno));
     return;
  }
  //  if( ioctl( fd, JSIOCSCORR, corr ) != 0 )
  //    perror( "Unable to turn off deadband correction" );

  GIOChannel* channel = g_io_channel_unix_new(fd);
  g_io_channel_set_encoding(channel, NULL, NULL);
  g_io_add_watch (channel, G_IO_IN , on_js_data_received, NULL);

}










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







//////////////////
// AIRBORNE CODE
//////////////////


#include "ppm.h"
#include "radio_control.h"
#include "actuators.h"
#include "commands.h"

#include "booz_telemetry.h"
//#include "datalink.h"

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
  ppm_pulses[RADIO_THROTTLE] = 1223 + 0.62 * (2050-1223);
  ppm_pulses[RADIO_PITCH]    = 1498 + 0.   * (2050-950);
  ppm_pulses[RADIO_ROLL]     = 1500 + 0.   * (2050-950);
  ppm_pulses[RADIO_YAW]      = 1500 + 0.   * (2050-950);
  ppm_pulses[RADIO_MODE]     = 1500 + 0.25 * (2050-950);

  ppm_init();
  radio_control_init();

  booz_estimator_init();
  booz_control_init();
  
}



static void airborne_periodic_task(void) {

#if 0
  int foo = sim_time/10; 
  if (!(foo%2)) {
    ppm_pulses[RADIO_YAW]    = 1500 + 0.1  * (2050-950);
    ppm_pulses[RADIO_PITCH]  = 1498 + 0.  * (2050-950);
  }
  else {
    ppm_pulses[RADIO_YAW]    = 1500 - 0.  * (2050-950);
    ppm_pulses[RADIO_PITCH]  = 1498 + 0.1 * (2050-950);
  }    
#endif

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

  //  DlEventCheckAndHandle();

  if (ppm_valid) {
    ppm_valid = FALSE;
    radio_control_event_task();
    booz_autopilot_mode = BOOZ_AP_MODE_ATTITUDE;
    if (rc_values_contains_avg_channels) {
      booz_autopilot_mode = BOOZ_AP_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    }
    booz_autopilot_event_task();
  }
  
}








#define JS_THROTTLE 6
#define JS_ROLL     0
#define JS_PITCH    1
#define JS_YAW      5
#define JS_MODE     2

static gboolean on_js_data_received(GIOChannel *source, GIOCondition condition, gpointer data) {

  struct js_event js;
  gsize len;
  GError *err = NULL;
  g_io_channel_read_chars(source, (void*)(&js), sizeof(struct js_event), &len, &err);
  
  if (js.type == JS_EVENT_AXIS) {
    //printf("%d %d\n", js.number, js.value);
    switch (js.number) {
    case JS_THROTTLE:
      ppm_pulses[RADIO_THROTTLE] = 1223 + (js.value - 30) * (float)(2050-1223) / (float)(223 - 30);
      break;
    case JS_PITCH:
      ppm_pulses[RADIO_PITCH] = 1498 + (js.value - 113) * (float)(2050-950) / (float)(224 - 1);
      break;
    case JS_ROLL:
      ppm_pulses[RADIO_ROLL] = 1500 + (js.value - 112) * (float)(2050-950) / (float)(1 - 224);
      break;
    case JS_YAW:
      ppm_pulses[RADIO_YAW] = 1500 + (js.value - 112) * (float)(2050-950) / (float)(224 - 1);
      break;
    case JS_MODE:
      ppm_pulses[RADIO_MODE] = 1500 + (js.value - 112) * (float)(2050-950) / (float)(224 - 1);
      rc_values_contains_avg_channels = TRUE;
      break;
    }
  }

  return TRUE;
}



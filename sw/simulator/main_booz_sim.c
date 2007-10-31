#include <glib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "booz_flight_model.h"
#include "booz_sensors_model.h"

#include "booz_flightgear.h"
#include "booz_joystick.h"

#include "booz_controller_main.h"
#include "booz_filter_main.h"


//char* fg_host = "127.0.0.1";
//char* fg_host = "10.31.4.107";
char* fg_host = "192.168.1.191";
unsigned int fg_port = 5501;
char* joystick_dev = "/dev/input/js0";

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

double booz_sim_actuators_values[] = {0., 0., 0., 0.};

static gboolean booz_sim_periodic(gpointer data);
static inline void booz_sim_display(void);

static void booz_sim_set_ppm_from_joystick( void );
static void booz_sim_read_actuators( void );

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
#include "imu_v3.h"
#include "booz_inter_mcu.h"

volatile bool_t ppm_valid;

static gboolean booz_sim_periodic(gpointer data) {
  /* read actuators positions */
  booz_sim_read_actuators();

  /* run our models */
  if (sim_time > 3.)
    /* no fdm at start to allow for filter initialisation */
    /* it sucks, I know */
    booz_flight_model_run(DT, booz_sim_actuators_values);

  booz_sensors_model_run(DT);
  
  sim_time += DT;

  /* call the filter periodic task to read sensors                      */
  /* the sim implementation of max1167_read ( called by ImuPeriodic() ) */
  /* will post a ImuEvent                                               */
  booz_filter_main_periodic_task();
  
  /* process the ImuEvent */
  /* it will run the filter and the inter-process communication which   */
  /* will post a BoozLinkMcuEvent in the Controller process             */
  booz_filter_main_event_task();

  /* process the BoozLinkMcuEvent                                       */
  /* this will update the controller estimator                          */
  booz_controller_main_event_task();
  /* cheat in simulation : psi not available from filter yet */
  booz_estimator_set_psi(bfm.state->ve[BFMS_PSI]);
  /* in simulation compute dcm as a helper for for nav */
  booz_estimator_compute_dcm();
  /* in simulation feed speed and pos estimations ( with a pos sensor :( ) */
  booz_estimator_set_speed_and_pos(bsm.speed_sensor->ve[AXIS_X], 
				   bsm.speed_sensor->ve[AXIS_Y], 
				   bsm.speed_sensor->ve[AXIS_Z],
				   bsm.pos_sensor->ve[AXIS_X], 
				   bsm.pos_sensor->ve[AXIS_Y], 
				   bsm.pos_sensor->ve[AXIS_Z] );


  /* post a radio control event */
  booz_sim_set_ppm_from_joystick();
  ppm_valid = TRUE;
  /* and let the controller process it */
  booz_controller_main_event_task();

  /* call the controller periodic task to run control loops            */
  booz_controller_main_periodic_task();


  booz_sim_display();

  return TRUE;
}

int main ( int argc, char** argv) {

  sim_time = 0.;
  disp_time = 0.;
  
  booz_flight_model_init();

  booz_sensors_model_init();

  booz_flightgear_init(fg_host, fg_port);

#ifdef SIM_UART
  sim_uart_init();
#endif 

#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport
  ivy_transport_init();
#endif

  booz_joystick_init(joystick_dev);

  booz_controller_main_init();

  booz_filter_main_init();
  

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, booz_sim_periodic, NULL);

  g_main_loop_run(ml);
 
  return 0;
}


/////////////////////
// Helpers
////////////////////

#define RPM_OF_RAD_S(a) ((a)*60./M_PI)
static inline void booz_sim_display(void) {
  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    booz_flightgear_send();
    IvySendMsg("148 BOOZ_SIM_RPMS %f %f %f %f",  
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_F]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_B]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_L]),
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_R]) );
    IvySendMsg("148 BOOZ_SIM_RATE_ATTITUDE %f %f %f %f %f %f",  
	       DegOfRad(bfm.state->ve[BFMS_P]), 
	       DegOfRad(bfm.state->ve[BFMS_Q]), 
	       DegOfRad(bfm.state->ve[BFMS_R]),
	       DegOfRad(bfm.state->ve[BFMS_PHI]), 
	       DegOfRad(bfm.state->ve[BFMS_THETA]), 
	       DegOfRad(bfm.state->ve[BFMS_PSI]));
    IvySendMsg("148 BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",  
	       (bfm.state->ve[BFMS_U]), 
	       (bfm.state->ve[BFMS_V]), 
	       (bfm.state->ve[BFMS_W]),
	       (bfm.state->ve[BFMS_X]), 
	       (bfm.state->ve[BFMS_Y]), 
	       (bfm.state->ve[BFMS_Z]));
    /* one good reason to store the bias in real unit !! */
    IvySendMsg("148 BOOZ_SIM_GYRO_BIAS %f %f %f",  
	       DegOfRad(bsm.gyro_bias_random_walk_value->ve[AXIS_P]),
	       DegOfRad(bsm.gyro_bias_random_walk_value->ve[AXIS_Q]), 
	       DegOfRad(bsm.gyro_bias_random_walk_value->ve[AXIS_R]));
  }
}


#define BREAK_MTT() {				\
    int foo = sim_time/10;			\
    switch(foo%4) {							\
    case 0:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.2 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.0 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.4 * (2050-950);		\
      break;								\
    case 2:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.2 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 3:								\
      ppm_pulses[RADIO_PITCH]    = 1500 - 0.0 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.4 * (2050-950);		\
      break;								\
    }									\
}

#define WALK_OVAL() {							\
    ppm_pulses[RADIO_ROLL]       = 1500 + 0. * (2050-950);		\
    int foo = sim_time/10;						\
    switch(foo%4) {							\
    case 0:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0. * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.2 * (2050-950);		\
      break;								\
    case 2:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0. * (2050-950);		\
      break;								\
    case 3:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.2 * (2050-950);		\
      break;								\
    }									\
}

#define STATIONARY() { \
    ppm_pulses[RADIO_ROLL]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_PITCH]    = 1498 + 0. * (2050-950);		\
    ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
  }


#define TOUPIE() {							\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_PITCH]    = 1498 + 0. * (2050-950);		\
    int foo = sim_time/20;						\
    switch(foo%4) {							\
    case 0:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.2 * (2050-950);		\
      break;								\
    case 2:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 3:								\
      ppm_pulses[RADIO_YAW]      = 1493 + -0.2 * (2050-950);		\
      break;								\
    }									\
  }


#define CIRCLE() {				                        \
    ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
    double alpha = 0.01 * sim_time;					\
    ppm_pulses[RADIO_PITCH]    = 1498 + cos(alpha) * 250.;		\
    ppm_pulses[RADIO_ROLL]     = 1500 + sin(alpha) * 250.;		\
  }

#define FAKE_JOYSTICK
static void booz_sim_set_ppm_from_joystick( void ) {
#ifndef FAKE_JOYSTICK
  ppm_pulses[RADIO_THROTTLE] = 1223 + booz_joystick_value[JS_THROTTLE] * (2050-1223);
  ppm_pulses[RADIO_PITCH]    = 1498 + booz_joystick_value[JS_PITCH]    * (2050-950);
  ppm_pulses[RADIO_ROLL]     = 1500 + booz_joystick_value[JS_ROLL]     * (2050-950);
  ppm_pulses[RADIO_YAW]      = 1493 + booz_joystick_value[JS_YAW]      * (2050-950);
  ppm_pulses[RADIO_MODE]     = 1500 + booz_joystick_value[JS_MODE]     * (2050-950);
  //  printf("joystick mode     %f %d\n", booz_joystick_value[JS_MODE], ppm_pulses[RADIO_MODE]);
  //  printf("joystick throttle %f %d\n", booz_joystick_value[JS_THROTTLE], ppm_pulses[RADIO_THROTTLE]);
  //  printf("joystick yaw %f %d\n", booz_joystick_value[JS_YAW], ppm_pulses[RADIO_YAW]);
  //  printf("joystick pitch %f %d\n", booz_joystick_value[JS_PITCH], ppm_pulses[RADIO_PITCH]);
  //  printf("joystick roll %f %d\n", booz_joystick_value[JS_ROLL], ppm_pulses[RADIO_ROLL]);
#else
  ppm_pulses[RADIO_THROTTLE] = 1223 + 0.6 * (2050-1223);
  //BREAK_MTT();
  //WALK_OVAL();
  // CIRCLE();
  STATIONARY();
  // TOUPIE();
#endif
}


static void booz_sim_read_actuators( void ) {
#if 0
  booz_sim_actuators_values[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK] - 1200)/(double)(1850-1200);
  booz_sim_actuators_values[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 1200)/(double)(1850-1200);
  booz_sim_actuators_values[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 1200)/(double)(1850-1200);
  booz_sim_actuators_values[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT] - 1200)/(double)(1850-1200);
#else
  booz_sim_actuators_values[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK]   - 0)/(double)(255);
  booz_sim_actuators_values[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 0)/(double)(255);
  booz_sim_actuators_values[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 0)/(double)(255);
  booz_sim_actuators_values[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT]   - 0)/(double)(255);
#endif

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









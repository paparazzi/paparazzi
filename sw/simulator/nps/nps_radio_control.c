#include "nps_radio_control.h"

#include <glib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <glib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define RADIO_CONTROL_DT (1./40.)

struct NpsRadioControl nps_radio_control;

static gboolean on_js_data_received(GIOChannel *source, GIOCondition condition, gpointer data);

void nps_radio_control_init(enum NpsRadioControlType type, int num_script, char* js_dev) {

  nps_radio_control.next_update = 0.;
  nps_radio_control.type = type;
  nps_radio_control.num_script = num_script;

  if (type == JOYSTICK) {
    int fd = open(js_dev, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
      printf("opening joystick device %s : %s\n", js_dev, strerror(errno));
      return;
    }
    GIOChannel* channel = g_io_channel_unix_new(fd);
    g_io_channel_set_encoding(channel, NULL, NULL);
    g_io_add_watch (channel, G_IO_IN , on_js_data_received, NULL);
  }

}


typedef void (*rc_script)(double);

static void radio_control_script_takeoff(double time);
static void radio_control_script_step_roll(double time);
static void radio_control_script_step_pitch(double time);
static void radio_control_script_step_yaw(double time);
static void radio_control_script_ff(double time);

static rc_script scripts[] = {
  radio_control_script_step_roll,
  radio_control_script_step_pitch,
  radio_control_script_step_yaw,
  radio_control_script_ff
};


#define MODE_SWITCH_MANUAL -1.0
#define MODE_SWITCH_AUTO1   0.0
#define MODE_SWITCH_AUTO2   1.0

#define RADIO_CONTROL_TAKEOFF_TIME 8


bool_t nps_radio_control_available(double time) {
  if (time >=  nps_radio_control.next_update) {
    nps_radio_control.next_update += RADIO_CONTROL_DT;
    if (time < RADIO_CONTROL_TAKEOFF_TIME)
      radio_control_script_takeoff(time);
    else if (nps_radio_control.type == SCRIPT)
      scripts[nps_radio_control.num_script](time);
    return TRUE;
  }
  return FALSE;
}


#define JS_ROLL     0
#define JS_PITCH    1
#define JS_MODE     2
#define JS_YAW      5
#define JS_THROTTLE 6
#define JS_NB_AXIS  7


static gboolean on_js_data_received(GIOChannel *source, 
				    GIOCondition condition __attribute__ ((unused)), 
				    gpointer data __attribute__ ((unused))) {

  struct js_event js;
  gsize len;
  GError *err = NULL;
  g_io_channel_read_chars(source, (gchar*)(&js), sizeof(struct js_event), &len, &err);
  
  if (js.type == JS_EVENT_AXIS) {
    if (js.number < JS_NB_AXIS) {
      switch (js.number) {
      case JS_THROTTLE:
	nps_radio_control.throttle = ((float)js.value + 28000.)/56000.; 
	//printf("joystick throttle %d\n",js.value);
	break;
      case JS_ROLL: 
	nps_radio_control.roll = (float)js.value/-28000.; 
	//printf("joystick roll %d %f\n",js.value, nps_radio_control.roll);
	break;
      case JS_PITCH:
	nps_radio_control.pitch = (float)js.value/-28000.; 
	//printf("joystick pitch %d\n",js.value);
	break;
      case JS_YAW:
	nps_radio_control.yaw = (float)js.value/-28000.; 
	//printf("joystick yaw %d\n",js.value);
	break;
      case JS_MODE:
	nps_radio_control.mode = (float)js.value/-32000.; 
	//printf("joystick mode %d\n",js.value);
	break;
      }
    }
  }
  return TRUE;
}


/*
 * Scripts
 *
 *
 */

void radio_control_script_takeoff(double time) {
  
  nps_radio_control.roll = 0.;
  nps_radio_control.pitch = 0.;
  nps_radio_control.yaw = 0.;
  nps_radio_control.throttle = 0.;
  nps_radio_control.mode = MODE_SWITCH_MANUAL;
  /* starts motors */
  if (time < 1.)
    nps_radio_control.yaw = 1.;
  else
    nps_radio_control.yaw = 0.;

}

void radio_control_script_step_roll(double time) {
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  
  if (((int32_t)rint((time*0.5)))%2) {
    nps_radio_control.roll = 0.2;
    nps_radio_control.yaw = 0.5;
  }
  else {
    nps_radio_control.roll = -0.2;
    nps_radio_control.yaw = 0.;
  }
}

void radio_control_script_step_pitch(double time) {
  nps_radio_control.roll = 0.;
  nps_radio_control.yaw = 0.;
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  if (((int32_t)rint((time*0.5)))%2) {
    nps_radio_control.pitch = 0.2;
  }
  else {
    nps_radio_control.pitch = -0.2;
  }
}

void radio_control_script_step_yaw(double time) {
  nps_radio_control.roll = 0.;
  nps_radio_control.pitch = 0.;
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  
  if (((int32_t)rint((time*0.5)))%2) {
    nps_radio_control.yaw = 0.5;
  }
  else {
    nps_radio_control.yaw = 0.;
  }
}

void radio_control_script_ff(double time __attribute__ ((unused))) {
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  if (time < RADIO_CONTROL_TAKEOFF_TIME+3)
    nps_radio_control.pitch = -1.;
  else if (time < RADIO_CONTROL_TAKEOFF_TIME+6) {
    //    nps_radio_control.roll = 0.5;
    nps_radio_control.pitch = -1.;
  }
  else {
    nps_radio_control.pitch = 0.;
    nps_radio_control.roll = 0.;
  }
}



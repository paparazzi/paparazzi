#include "booz_joystick.h"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <glib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

double booz_joystick_value[JS_NB_AXIS];
//const double booz_joystick_neutral[JS_NB_AXIS] = { 112., 113., 112., 112., 112., 112.,  30.};
//const double booz_joystick_max[JS_NB_AXIS] =     {   1., 224., 224., 224., 224., 224., 223.};
//const double booz_joystick_min[JS_NB_AXIS] =     { 224.,   1.,   1.,   1.,   1.,   1.,  30.};

const double booz_joystick_neutral[JS_NB_AXIS] = { 113., 113., 112., 112., 112., 112.,  30.};
const double booz_joystick_max[JS_NB_AXIS] =     {  46., 178., 224., 224., 224., 179., 223.};
const double booz_joystick_min[JS_NB_AXIS] =     { 179.,  46.,   1.,   1.,   1.,  46.,  30.};



static gboolean on_data_received(GIOChannel *source, GIOCondition condition, gpointer data);

void booz_joystick_init(const char* device) {
  int i;
  for (i=0; i<JS_NB_AXIS; i++)
    booz_joystick_value[i] = 0.;
  booz_joystick_value[JS_MODE] = -0.7; 

  int fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
    printf("opening joystick serial device %s : %s\n", device, strerror(errno));
    return;
  }
  GIOChannel* channel = g_io_channel_unix_new(fd);
  g_io_channel_set_encoding(channel, NULL, NULL);
  g_io_add_watch (channel, G_IO_IN , on_data_received, NULL);
  
}


static gboolean on_data_received(GIOChannel *source, 
				 GIOCondition condition __attribute__ ((unused)), 
				 gpointer data __attribute__ ((unused))) {

  struct js_event js;
  gsize len;
  GError *err = NULL;
  g_io_channel_read_chars(source, (void*)(&js), sizeof(struct js_event), &len, &err);
  
  if (js.type == JS_EVENT_AXIS) {
    if (js.number < JS_NB_AXIS) {
      //      if (js.number == JS_THROTTLE) printf("joystick value %d\n",js.value); 
      booz_joystick_value[js.number] = (double)(js.value - booz_joystick_neutral[js.number]) / 
	(booz_joystick_max[js.number] - booz_joystick_min[js.number]);
    }
  }

  return TRUE;
}


#if 0
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
#endif

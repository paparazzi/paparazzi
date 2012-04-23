#include "nps_radio_control.h"
#include "nps_radio_control_joystick.h"

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

NpsJoystick nps_joystick;

static gboolean on_js_data_received(GIOChannel *source, GIOCondition condition, gpointer data);

int nps_radio_control_joystick_init(const char* device) {

  nps_joystick.throttle = 0.5;
  nps_joystick.roll = 0.;
  nps_joystick.pitch = 0.;
  nps_joystick.yaw = 0.;
  nps_joystick.mode = MODE_SWITCH_AUTO2;

  int fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
    printf("opening joystick device %s : %s\n", device, strerror(errno));
    return -1;
  }
  GIOChannel* channel = g_io_channel_unix_new(fd);
  g_io_channel_set_encoding(channel, NULL, NULL);
  g_io_add_watch (channel, G_IO_IN , on_js_data_received, NULL);
  return 0;
}


#define JS_ROLL     0
#define JS_PITCH    1
#define JS_YAW      2
#define JS_THROTTLE 3
#define JS_NB_AXIS  7

// buttons to switch modes
#define JS_MODE_MANUAL 4
#define JS_MODE_AUTO1  5
#define JS_MODE_AUTO2  6

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
        nps_joystick.throttle = ((float)js.value - 32767.)/-65534.;
        //printf("joystick throttle %d\n",js.value);
        break;
      case JS_ROLL:
        nps_joystick.roll = (float)js.value/32767.;
        //printf("joystick roll %d %f\n",js.value, nps_joystick.roll);
        break;
      case JS_PITCH:
        nps_joystick.pitch = (float)js.value/32767.;
        //printf("joystick pitch %d %f\n",js.value, nps_joystick.pitch);
        break;
      case JS_YAW:
        //nps_joystick.yaw = 0.;
        nps_joystick.yaw = (float)js.value/32767.;
        //printf("joystick yaw %d %f\n",js.value, nps_joystick.yaw);
        break;
      }
    }
  }
  if (js.type == JS_EVENT_BUTTON) {
    switch (js.number) {
    case JS_MODE_MANUAL:
      nps_joystick.mode = MODE_SWITCH_MANUAL;
      break;
    case JS_MODE_AUTO1:
      nps_joystick.mode = MODE_SWITCH_AUTO1;
      break;
    case JS_MODE_AUTO2:
      nps_joystick.mode = MODE_SWITCH_AUTO2;
      break;
    }
  }

  return TRUE;
}

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

static gboolean on_js_data_received(GIOChannel *source, GIOCondition condition, gpointer data);

int nps_radio_control_joystick_init(const char* device) {

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

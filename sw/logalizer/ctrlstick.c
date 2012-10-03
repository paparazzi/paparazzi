/*
 * ctrlstick.c
 *
 * send joystick control to paparazzi through ivy
 *
 * based on Force Feedback: Constant Force Stress Test
 * Copyright (C) 2001 Oliver Hamann
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define DBG 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <glib.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#ifdef DBG
#define dbgprintf fprintf
#else
#define dbgprintf(x ...)
#endif

#define Bound(_x, _min, _max) { if (_x > _max) _x = _max; else if (_x < _min) _x = _min; }

#define TIMEOUT_PERIOD 20

#define MB_ID 1

#define INPUT_DEV_MAX   15

#define AXIS_COUNT      4

#if 0
// rumblepad 2
#define AXIS_YAW        ABS_X
#define AXIS_PITCH      ABS_Y
#define AXIS_ROLL       ABS_Z
#define AXIS_THROTTLE   ABS_RZ
#endif

#if 1
// e-sky 0905A simulator fms
//#define AXIS_YAW        ABS_X
//#define AXIS_ROLL       ABS_Z
//#define AXIS_THROTTLE   ABS_RY
//#define AXIS_PITCH      ABS_Y
#define AXIS_YAW        ABS_RY
#define AXIS_ROLL       ABS_Y
#define AXIS_THROTTLE   ABS_X
#define AXIS_PITCH      ABS_Z

#define THROTTLE_MIN        (-90)
#define THROTTLE_NEUTRAL    (0)
#define THROTTLE_MAX        (90)
#define ROLL_MIN            (-122)
#define ROLL_NEUTRAL        (-13)
#define ROLL_MAX            (113)
#define PITCH_MIN           (-109)
#define PITCH_NEUTRAL       (-2)
#define PITCH_MAX           (109)
#define YAW_MIN             (125)
#define YAW_NEUTRAL         (-13
#define YAW_MAX             (115)
#endif

#if 0
// wingman force 3d
#define AXIS_YAW        ABS_RZ
#define AXIS_PITCH      ABS_Y
#define AXIS_ROLL       ABS_X
#define AXIS_THROTTLE   ABS_THROTTLE
#endif

/* Helper for testing large bit masks */
#define TEST_BIT(bit,bits) (((bits[bit>>5]>>(bit&0x1f))&1)!=0)

/* Default values for the options */
#define DEVICE_NAME         "/dev/input/event"
#define DEFAULT_DEVICE_NAME "/dev/input/event4"
#define DEFAULT_AC_ID       1

/* Options */
const char * device_name_default = DEFAULT_DEVICE_NAME;
int aircraft_id                  = DEFAULT_AC_ID;

/* Global variables about the initialized device */
int device_handle;
int axis_code[AXIS_COUNT] = {AXIS_ROLL, AXIS_PITCH, AXIS_THROTTLE, AXIS_YAW};
int axis_min[AXIS_COUNT], axis_max[AXIS_COUNT];
int position[AXIS_COUNT] = {0, 0, 0};
struct ff_effect effect;
GMainLoop *ml;

void parse_args(int argc, char * argv[])
{
    /* nothing to do any more */
}

int init_hid_device(char* device_name)
{
    int cnt;
	unsigned long key_bits[32],abs_bits[32];
	int valbuf[16];
	char name[256] = "Unknown";
#ifdef USE_FORCE_FEEDBACK
	unsigned long ff_bits[32];
	struct input_event event;
#endif

	/* Open event device read only (with write permission for ff) */
	device_handle = open(device_name,O_RDONLY|O_NONBLOCK);
	if (device_handle<0) {
		dbgprintf(stderr,"ERROR: can not open %s (%s) [%s:%d]\n",
		        device_name,strerror(errno),__FILE__,__LINE__);
		return(1);
	}

	/* Which buttons has the device? */
	memset(key_bits,0,32*sizeof(unsigned long));
	if (ioctl(device_handle,EVIOCGBIT(EV_KEY,32*sizeof(unsigned long)),key_bits)<0) {
		dbgprintf(stderr,"ERROR: can not get key bits (%s) [%s:%d]\n",
		        strerror(errno),__FILE__,__LINE__);
        close(device_handle);
		return(1);
	}

	/* Which axes has the device? */
	memset(abs_bits,0,32*sizeof(unsigned long));
	if (ioctl(device_handle,EVIOCGBIT(EV_ABS,32*sizeof(unsigned long)),abs_bits)<0) {
		dbgprintf(stderr,"ERROR: can not get abs bits (%s) [%s:%d]\n",
		        strerror(errno),__FILE__,__LINE__);
        close(device_handle);
		return(1);
	}

#if 0
	printf("0x%08lX\n",abs_bits[0]);
	printf("0x%08lX\n",abs_bits[1]);
	printf("0x%08lX\n",abs_bits[2]);
	printf("0x%08lX\n",abs_bits[3]);
#endif

	/* Which axis? */
	if  (!TEST_BIT(AXIS_YAW ,abs_bits)) {
		dbgprintf(stderr,"ERROR: no suitable yaw axis found [%s:%d]\n",
		        __FILE__,__LINE__);
        close(device_handle);
		return(1);
	}
	if (!TEST_BIT(AXIS_PITCH ,abs_bits)) {
		dbgprintf(stderr,"ERROR: no suitable pitch axis found [%s:%d]\n",
		        __FILE__,__LINE__);
        close(device_handle);
		return(1);
	}
	if (!TEST_BIT(AXIS_ROLL ,abs_bits)) {
		dbgprintf(stderr,"ERROR: no suitable roll axis found [%s:%d]\n",
		        __FILE__,__LINE__);
        close(device_handle);
		return(1);
	}
	if (!TEST_BIT(AXIS_THROTTLE ,abs_bits)) {
		dbgprintf(stderr,"ERROR: no suitable throttle axis found [%s:%d]\n",
		        __FILE__,__LINE__);
        close(device_handle);
		return(1);
	}

    for (cnt=0; cnt<AXIS_COUNT; cnt++)
    {
    	/* get axis value range */
    	if (ioctl(device_handle,EVIOCGABS(axis_code[cnt]),valbuf)<0) {
    		dbgprintf(stderr,"ERROR: can not get axis value range (%s) [%s:%d]\n",
    		        strerror(errno),__FILE__,__LINE__);
            close(device_handle);
    		return(1);
    	}
    	axis_min[cnt]=valbuf[1];
    	axis_max[cnt]=valbuf[2];
    	if (axis_min[cnt]>=axis_max[cnt]) {
    		dbgprintf(stderr,"ERROR: bad axis value range (%d,%d) [%s:%d]\n",
    		        axis_min[cnt],axis_max[cnt],__FILE__,__LINE__);
            close(device_handle);
    		return(1);
    	}

        /* get last data */
        position[cnt]=(((valbuf[0])-axis_min[cnt]))*254/(axis_max[cnt]-axis_min[cnt])-127;
	}

#if 0
	/* Now get some information about force feedback */
	memset(ff_bits,0,32*sizeof(unsigned long));
	if (ioctl(device_handle,EVIOCGBIT(EV_FF ,32*sizeof(unsigned long)),ff_bits)<0) {
		dbgprintf(stderr,"ERROR: can not get ff bits (%s) [%s:%d]\n",
		        strerror(errno),__FILE__,__LINE__);
        close(device_handle);
		return(1);
	}

	/* force feedback supported? */
	if (!TEST_BIT(FF_CONSTANT,ff_bits)) {
		dbgprintf(stderr,"ERROR: device (or driver) has no force feedback support [%s:%d]\n",
		        __FILE__,__LINE__);
        close(device_handle);
		return(1);
	}

	/* Switch off auto centering */
	memset(&event,0,sizeof(event));
	event.type=EV_FF;
	event.code=FF_AUTOCENTER;
	event.value=0;
	if (write(device_handle,&event,sizeof(event))!=sizeof(event)) {
		dbgprintf(stderr,"ERROR: failed to disable auto centering (%s) [%s:%d]\n",
		        strerror(errno),__FILE__,__LINE__);
        close(device_handle);
		return(1);
	}
#endif

#ifdef USE_FORCE_FEEDBACK // turn on for ff effect
	/* Initialize constant force effect */
	memset(&effect,0,sizeof(effect));
	effect.type=FF_CONSTANT;
	effect.id=-1;
	effect.trigger.button=0;
	effect.trigger.interval=0;
	effect.replay.length=0xffff;
	effect.replay.delay=0;
	effect.u.constant.level=0;
	effect.direction=0xC000;
	effect.u.constant.envelope.attack_length=0;
	effect.u.constant.envelope.attack_level=0;
	effect.u.constant.envelope.fade_length=0;
	effect.u.constant.envelope.fade_level=0;

	/* Upload effect */
	if (ioctl(device_handle,EVIOCSFF,&effect)==-1) {
		dbgprintf(stderr,"ERROR: uploading effect failed (%s) [%s:%d]\n",
		        strerror(errno),__FILE__,__LINE__);
        close(device_handle);
		return(1);
	}

	/* Start effect */
	memset(&event,0,sizeof(event));
	event.type=EV_FF;
	event.code=effect.id;
	event.value=1;
	if (write(device_handle,&event,sizeof(event))!=sizeof(event)) {
		dbgprintf(stderr,"ERROR: starting effect failed (%s) [%s:%d]\n",
		        strerror(errno),__FILE__,__LINE__);
        close(device_handle);
		return(1);
	}
#endif

	ioctl(device_handle, EVIOCGNAME(sizeof(name)), name);
	printf("Input device name: \"%s\"\n", name);

    return(0);
}

static gboolean joystick_periodic(gpointer data __attribute__ ((unused))) {

  int res, cnt, changed = 0, mode=0;
  int throttle, pitch, roll;
  int throttle_mode;

  struct input_event event;
  static struct timeval time_now;

  /* Get events */
  do {
    res = read(device_handle,&event,sizeof(event));

    if (res == sizeof(event))
    {
	  for (cnt=0; cnt<AXIS_COUNT; cnt++) {
        if (event.type==EV_ABS && event.code==axis_code[cnt]) {
          position[cnt]=(((event.value)-axis_min[cnt]))*254/(axis_max[cnt]-axis_min[cnt])-127;
          changed = 1;
        }
      }
    }
  } while (res == sizeof(event));

  if (errno != EAGAIN)
  {
    printf("device removed\n");
    g_main_loop_quit(ml);
    return 0;
  }

//    if (changed)
    {
        dbgprintf(stdout, "pos  %d %d %d %d\n", position[0], position[1], position[2], position[3]);

        if (position[3] > 125) mode = 2;
        else if (position[3] < -125) mode = 1;
        else mode = 0;

        throttle = ((position[0] - THROTTLE_NEUTRAL -THROTTLE_MIN) * 63) / (THROTTLE_MAX-THROTTLE_MIN);
        Bound(throttle, 0, 63)
        roll = position[2] - ROLL_NEUTRAL;
        Bound(roll, -128, 127)
        pitch = position[1] - PITCH_NEUTRAL;
        Bound(pitch, -128, 127)

        throttle_mode = (throttle << 2) | mode;

        gettimeofday(&time_now, 0);

//        if (mode != 2)
        {
            dbgprintf(stdout,"mode: %d, throttle: %d, roll: %d, pitch: %d\n", throttle_mode & 3, throttle_mode >> 2, roll, pitch);
	        IvySendMsg("dl RC_3CH %d %d %d", throttle_mode, roll, pitch);
        }
    }

	return 1;
}

int main ( int argc, char** argv) {

  char devname[256];
  int cnt;
  ml =  g_main_loop_new(NULL, FALSE);

  parse_args(argc, argv);

  IvyInit ("IvyCtrlJoystick", "IvyCtrlJoystick READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  while(1)
  {
    for (cnt=0; cnt<INPUT_DEV_MAX; cnt++){
      sprintf(devname, DEVICE_NAME "%d", cnt);
      if (init_hid_device(devname) == 0) break;
    }

    if (cnt != INPUT_DEV_MAX)
    {
      g_timeout_add(TIMEOUT_PERIOD, joystick_periodic, NULL);
      g_main_loop_run(ml);
     }
     sleep(1);
  }

  return 0;
}

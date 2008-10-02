/*
 * $Id$
 *
 * joystick lib
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

//#define STICK_DBG 1

#include "usb_stick.h"

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

#ifdef STICK_DBG
#define dbgprintf fprintf
#else
#define dbgprintf(x ...)
#endif

#define MIN_BUTTON_CODE   BTN_JOYSTICK
#define MAX_BUTTON_CODE   BTN_THUMBR+1
#define MIN_ABS_CODE      ABS_X
#define MAX_ABS_CODE      ABS_MAX+1

#define ABS_MAX_VALUE     254
#define ABS_MID_VALUE     127

#define BUTTON_COUNT      STICK_BUTTON_COUNT
#define AXIS_COUNT        STICK_AXIS_COUNT

/* Helper for testing large bit masks */
#define TEST_BIT(bit,bits) (((bits[bit>>5]>>(bit&0x1f))&1)!=0)

/* Global variables about the initialized device */
int stick_device_handle;

int8_t stick_axis_values[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
int16_t stick_button_values = 0;

int axis_code[AXIS_COUNT];
int axis_count = 0;
int button_code[BUTTON_COUNT];
int button_count = 0;

int32_t axis_min[AXIS_COUNT], axis_max[AXIS_COUNT];

struct stick_code_param_ stick_init_param = {
  0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  0, {0, 0, 0, 0, 0, 0}
};

//struct ff_effect effect;


int init_hid_device(char* device_name)
{
  int cnt;
  unsigned long key_bits[32],abs_bits[32];
  //	unsigned long ff_bits[32];
  int valbuf[16];
  char name[256] = "Unknown";

  button_count = 0;
  axis_count = 0;

  /* Open event device read only (with write permission for ff) */
  stick_device_handle = open(device_name,O_RDONLY|O_NONBLOCK);
  if (stick_device_handle<0) {
    dbgprintf(stderr,"ERROR: can not open %s (%s) [%s:%d]\n",
        device_name,strerror(errno),__FILE__,__LINE__);
    return(1);
  }

  /* Which buttons has the device? */
  memset(key_bits,0,32*sizeof(unsigned long));
  if (ioctl(stick_device_handle,EVIOCGBIT(EV_KEY,32*sizeof(unsigned long)),key_bits)<0) {
    dbgprintf(stderr,"ERROR: can not get key bits (%s) [%s:%d]\n",
        strerror(errno),__FILE__,__LINE__);
    return(1);
  }

  /* Store buttons */
  if (stick_init_param.button_count > 0) {
    for (cnt = 0; cnt < MIN(stick_init_param.button_count,BUTTON_COUNT); cnt++) {
      if (!TEST_BIT(stick_init_param.button_code[cnt], key_bits)) {
        dbgprintf(stderr,"ERROR: no suitable custom button %d found [%s:%d]\n",cnt,__FILE__,__LINE__);
        return 1;
      }
    }
    button_count = stick_init_param.button_count;
    memcpy(button_code,stick_init_param.button_code,BUTTON_COUNT*sizeof(int));
  }
  else {
    for (cnt = MIN_BUTTON_CODE; cnt < MAX_BUTTON_CODE; cnt++) {
      if (TEST_BIT(cnt, key_bits)) {
        button_code[button_count++] = cnt;
        dbgprintf(stderr,"Available button: %d (0x%x)\n",cnt,cnt);
      }
      if (button_count == BUTTON_COUNT) break;
    }
    if (button_count == 0) {
      dbgprintf(stderr,"ERROR: no suitable buttons found [%s:%d]\n",__FILE__,__LINE__);
    }
  }

  /* Which axis has the device? */
  memset(abs_bits,0,32*sizeof(unsigned long));
  if (ioctl(stick_device_handle,EVIOCGBIT(EV_ABS,32*sizeof(unsigned long)),abs_bits)<0) {
    dbgprintf(stderr,"ERROR: can not get abs bits (%s) [%s:%d]\n",
        strerror(errno),__FILE__,__LINE__);
    return(1);
  }

  /* Store axis */
  if (stick_init_param.axis_count > 0) {
    for (cnt = 0; cnt < MIN(stick_init_param.axis_count,AXIS_COUNT); cnt++) {
      if (!TEST_BIT(stick_init_param.axis_code[cnt], abs_bits)) {
        dbgprintf(stderr,"ERROR: no suitable custom axis %d found [%s:%d]\n",cnt,__FILE__,__LINE__);
        return 1;
      }
    }
    axis_count = stick_init_param.axis_count;
    memcpy(axis_code,stick_init_param.axis_code,AXIS_COUNT*sizeof(int));
  }
  else {
    for (cnt = MIN_ABS_CODE; cnt < MAX_ABS_CODE; cnt++) {
      if (TEST_BIT(cnt, abs_bits)) {
        axis_code[axis_count++] = cnt;
        dbgprintf(stderr,"Available axis: %d (0x%x)\n",cnt,cnt);
      }
      if (axis_count == AXIS_COUNT) break;
    }
    // at least 2 axis are needed in auto detection
    if (axis_count < 2) {
      dbgprintf(stderr,"ERROR: no suitable axis found [%s:%d]\n",__FILE__,__LINE__);
      return(1);
    }
  }

  /* Axis param */
  for (cnt = 0; cnt < axis_count; cnt++)
  {
    /* get axis value range */
    if (ioctl(stick_device_handle,EVIOCGABS(axis_code[cnt]),valbuf)<0) {
      dbgprintf(stderr,"ERROR: can not get axis %d value range (%s) [%s:%d]\n",
          cnt,strerror(errno),__FILE__,__LINE__);
      return(1);
    }
    axis_min[cnt]=valbuf[1];
    axis_max[cnt]=valbuf[2];
    if (axis_min[cnt]>=axis_max[cnt]) {
      dbgprintf(stderr,"ERROR: bad axis %d value range (%d,%d) [%s:%d]\n",
          cnt,axis_min[cnt],axis_max[cnt],__FILE__,__LINE__);
      return(1);
    }
  }

#if 0
  /* Now get some information about force feedback */
  memset(ff_bits,0,32*sizeof(unsigned long));
  if (ioctl(device_handle,EVIOCGBIT(EV_FF ,32*sizeof(unsigned long)),ff_bits)<0) {
    dbgprintf(stderr,"ERROR: can not get ff bits (%s) [%s:%d]\n",
        strerror(errno),__FILE__,__LINE__);
    return(1);
  }

  /* force feedback supported? */
  if (!TEST_BIT(FF_CONSTANT,ff_bits)) {
    dbgprintf(stderr,"ERROR: device (or driver) has no force feedback support [%s:%d]\n",
        __FILE__,__LINE__);
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
    return(1);
  }

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
    return(1);
  }
#endif	

  ioctl(stick_device_handle, EVIOCGNAME(sizeof(name)), name);
  printf("Input device name: \"%s\" on device \"%s\"\n", name, device_name);

  return(0);
}


int stick_read( void ) {

  int cnt;
  struct input_event event;

  /* Get events */
  while (read(stick_device_handle,&event,sizeof(event))==sizeof(event)) {

    switch (event.type) {
      case EV_KEY:
        for (cnt = 0; cnt < button_count; cnt++) {
          if (event.code == button_code[cnt]) {
            if (event.value) stick_button_values |= (1 << cnt); // Set bit
            else stick_button_values &= ~(1 << cnt); // Clear bit
            break;
          }
        }
        break;
      case EV_ABS:
        for (cnt = 0; cnt < axis_count; cnt++) {
          if (event.code == axis_code[cnt]) {
            stick_axis_values[cnt] = (((event.value) - axis_min[cnt]))*ABS_MAX_VALUE / (axis_max[cnt] - axis_min[cnt]) - ABS_MID_VALUE;
            break;
          }
        }
        break;
      default: break;
    }

  }

  dbgprintf(stderr,"buttons %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d | ",
      (stick_button_values >> 0) & 1,
      (stick_button_values >> 1) & 1,
      (stick_button_values >> 2) & 1,
      (stick_button_values >> 3) & 1,
      (stick_button_values >> 4) & 1,
      (stick_button_values >> 5) & 1,
      (stick_button_values >> 6) & 1,
      (stick_button_values >> 7) & 1,
      (stick_button_values >> 8) & 1,
      (stick_button_values >> 9) & 1,
      (stick_button_values >> 10) & 1,
      (stick_button_values >> 11) & 1,
      (stick_button_values >> 12) & 1,
      (stick_button_values >> 13) & 1,
      (stick_button_values >> 14) & 1,
      (stick_button_values >> 15) & 1);
  dbgprintf(stderr,"axis %d %d %d %d %d %d\n",
      stick_axis_values[0],
      stick_axis_values[1],
      stick_axis_values[2],
      stick_axis_values[3],
      stick_axis_values[4],
      stick_axis_values[5]);

  return 0;
}


int stick_init( char * device_name ) {

  char devname[256];
  int cnt = 0;

  /* test device_name, else look for a suitable device */
  if (device_name != NULL) {
    if (init_hid_device(device_name) != 0) cnt = STICK_INPUT_DEV_MAX;
  }
  else {
    for (cnt = 0; cnt < STICK_INPUT_DEV_MAX; cnt++) {
      sprintf(devname, STICK_DEVICE_NAME "%d", cnt);
      if (init_hid_device(devname) == 0) break;
    }
  }

  /* return 1 if no device found */
  if (cnt == STICK_INPUT_DEV_MAX) {
    fprintf(stderr,"ERROR: no suitable joystick found [%s:%d]\n",
        __FILE__,__LINE__);
    return(1);
  }  

  return 0;
}


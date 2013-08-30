/*
 * Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

/**
 * @file boards/ardrone/gpio_ardrone.c
 * ardrone GPIO driver
 */

#include <stdio.h>
#include <stdlib.h>
#include "gpio_ardrone.h"

//val=0 -> set gpio output lo
//val=1 -> set gpio output hi
//val=-1 -> set gpio as input (output hi-Z)
int gpio_set(int nr,int val)
{
  char cmdline[200];
  if(val<0) sprintf(cmdline,"/usr/sbin/gpio %d -d i",nr);
  else if(val>0) sprintf(cmdline,"/usr/sbin/gpio %d -d ho 1",nr);
  else sprintf(cmdline,"/usr/sbin/gpio %d -d ho 0",nr);
  return system(cmdline);
}

// Option 1:
//#define WE_HAVE_NO_CLUE_YET

// Option 2:
#define WE_MUST_TO_USE_THE_TERRIBLE_HACK

// Option 3:
//#define WE_KNOW_HOW_ARDRONE_IOCTL_WORKS_ON_DEV_GPIO



#ifdef WE_HAVE_NO_CLUE_YET

int gpio_get(int nr)
{
  return 0;
}

#endif




#ifdef WE_MUST_TO_USE_THE_TERRIBLE_HACK

FILE* ardrone_system_pipe = 0;

int gpio_get(int nr)
{

  if (ardrone_system_pipe == 0)
  {
	  char cmdline[200];
	  sprintf(cmdline,"/usr/sbin/gpio %d -r",nr);
	  ardrone_system_pipe = popen(cmdline,"r");
	  if (!ardrone_system_pipe)
	  {
		  return -1;
	  }
  }
  else
  {
	  // TODO: we now call this with a large delay expecting that all data is present
	  // if (!feof(pipe)) // Still busy

	  char buff[128];
	  char* ret = fgets(buff, 128, ardrone_system_pipe);
	  ret = fgets(buff, 128, ardrone_system_pipe);
	  pclose(ardrone_system_pipe);
	  ardrone_system_pipe = 0;

	  if (ret == NULL)
	  {
	    return -2;
	  }

	  int pin = ret[25] - '0';

	  printf("GPIO_GET: %d '%d' \n", nr, pin);

	  return pin;
  }
  return -3;
}

#endif





#ifdef WE_KNOW_HOW_ARDRONE_IOCTL_WORKS_ON_DEV_GPIO

#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <sys/ioctl.h>


#define GPIO_IOCTL_COUNT 0
#define GPIO_IOCTL_GET 2

int gpiofp = 0;
int gpio_get(int nr)
{
	if (gpiofp == 0)
	{
		gpiofp = open("/dev/gpio",O_RDWR);
		printf("GPIO open %d\n", gpiofp);
		// printf("%s", errno() );
	}
	else
	{
		int gpio = nr;
		int ret = ioctl(gpiofp, GPIO_IOCTL_GET, &gpio);
		printf("GPIO_ %d = %d %d \n",nr,gpio, ret);
	}

	// We don't know yet
	return 0;
}

#endif

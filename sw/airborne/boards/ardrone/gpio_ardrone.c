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

#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <sys/ioctl.h>
#include "gpio_ardrone.h"

#define GPIO_MAGIC 'p'
#define GPIO_DIRECTION _IOW(GPIO_MAGIC, 0, struct gpio_direction)
#define GPIO_READ _IOWR(GPIO_MAGIC, 1, struct gpio_data)
#define GPIO_WRITE _IOW(GPIO_MAGIC, 2, struct gpio_data)
int gpiofp = 0;

struct gpio_data {
  int pin;
  int value;
};

enum gpio_mode {
    GPIO_INPUT = 0,             //!< Pin configured for input
    GPIO_OUTPUT_LOW,            //!< Pin configured for output with low level
    GPIO_OUTPUT_HIGH,           //!< Pin configured for output with high level
};

struct gpio_direction {
    int pin;
    enum gpio_mode mode;
};

//val=0 -> set gpio output lo
//val=1 -> set gpio output hi
void gpio_set(int nr, int val)
{
  struct gpio_data data;
  // Open the device if not open
  if (gpiofp == 0)
    gpiofp = open("/dev/gpio",O_RDWR);

  // Read the GPIO value
  data.pin = nr;
  data.value = val;
  ioctl(gpiofp, GPIO_WRITE, &data);
}

void gpio_set_input(int nr)
{
  struct gpio_direction dir;
  // Open the device if not open
  if (gpiofp == 0)
    gpiofp = open("/dev/gpio",O_RDWR);

  // Read the GPIO value
  dir.pin = nr;
  dir.mode = GPIO_INPUT;
  ioctl(gpiofp, GPIO_DIRECTION, &dir);
}

int gpio_get(int nr)
{
  struct gpio_data data;
  // Open the device if not open
  if (gpiofp == 0)
    gpiofp = open("/dev/gpio",O_RDWR);

  // Read the GPIO value
  data.pin = nr;
  ioctl(gpiofp, GPIO_READ, &data);
  return data.value;
}

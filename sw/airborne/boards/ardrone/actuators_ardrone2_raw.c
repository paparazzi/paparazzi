/*
 * Original Code from:
 * Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com
 *
 * Adapated for Paparazzi by:
 * Copyright (C) 2012 Dino Hensen <dino.hensen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/ardrone/actuators_ardrone2_raw.c
 * Actuator driver for ardrone2-raw version
 */

#include "subsystems/actuators.h"
#include "actuators_ardrone2_raw.h"
#include "gpio_ardrone.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>

/**
 * Power consumption @ 11V all 4 motors running
 * PWM   A
 * 0    0.2
 * 80   1.3
 * 100  1.5
 * 150  2.0
 * 190  2.5
 * 130  3.0
 */
int mot_fd; /**< File descriptor for the port */

void actuators_ardrone_init(void)
{
  //open mot port
  mot_fd = open("/dev/ttyO0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (mot_fd == -1)
  {
    perror("open_port: Unable to open /dev/ttyO0 - ");
    return;
  }
  fcntl(mot_fd, F_SETFL, 0); //read calls are non blocking
  fcntl(mot_fd, F_GETFL, 0);

  //set port options
  struct termios options;
  //Get the current options for the port
  tcgetattr(mot_fd, &options);
  //Set the baud rates to 115200
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
  options.c_iflag = 0; //clear input options
  options.c_lflag=0; //clear local options
  options.c_oflag &= ~OPOST; //clear output options (raw output)

  //Set the new options for the port
  tcsetattr(mot_fd, TCSANOW, &options);

  //reset IRQ flipflop - on error 106 read 1, this code resets 106 to 0
  gpio_set(106,-1);
  gpio_set(107,0);
  gpio_set(107,1);

  //all select lines inactive
  gpio_set(68,1);
  gpio_set(69,1);
  gpio_set(70,1);
  gpio_set(71,1);

  //configure motors
  uint8_t reply[256];
  for(int m=0;m<4;m++) {
    gpio_set(68+m,-1);
    actuators_ardrone_cmd(0xe0,reply,2);
    if(reply[0]!=0xe0 || reply[1]!=0x00)
    {
      printf("motor%d cmd=0x%02x reply=0x%02x\n",m+1,(int)reply[0],(int)reply[1]);
    }
    actuators_ardrone_cmd(m+1,reply,1);
    gpio_set(68+m,1);
  }

  //all select lines active
  gpio_set(68,-1);
  gpio_set(69,-1);
  gpio_set(70,-1);
  gpio_set(71,-1);

  //start multicast
  actuators_ardrone_cmd(0xa0,reply,1);
  actuators_ardrone_cmd(0xa0,reply,1);
  actuators_ardrone_cmd(0xa0,reply,1);
  actuators_ardrone_cmd(0xa0,reply,1);
  actuators_ardrone_cmd(0xa0,reply,1);

  //reset IRQ flipflop - on error 106 read 1, this code resets 106 to 0
  gpio_set(106,-1);
  gpio_set(107,0);
  gpio_set(107,1);

  //all leds green
//  actuators_ardrone_set_leds(MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN);
}

int actuators_ardrone_cmd(uint8_t cmd, uint8_t *reply, int replylen) {
  write(mot_fd, &cmd, 1);
  return read(mot_fd, reply, replylen);
}

void actuators_ardrone_commit(void)
{
  actuators_ardrone_set_pwm(actuators_pwm_values[0], actuators_pwm_values[1], actuators_pwm_values[2], actuators_pwm_values[3]);
}

/**
 * Write motor speed command
 * cmd = 001aaaaa aaaabbbb bbbbbccc ccccccdd ddddddd0
 */
void actuators_ardrone_set_pwm(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
  uint8_t cmd[5];
  cmd[0] = 0x20 | ((pwm0&0x1ff)>>4);
  cmd[1] = ((pwm0&0x1ff)<<4) | ((pwm1&0x1ff)>>5);
  cmd[2] = ((pwm1&0x1ff)<<3) | ((pwm2&0x1ff)>>6);
  cmd[3] = ((pwm2&0x1ff)<<2) | ((pwm3&0x1ff)>>7);
  cmd[4] = ((pwm3&0x1ff)<<1);
  write(mot_fd, cmd, 5);
}

/**
 * Write LED command
 * cmd = 011grgrg rgrxxxxx (this is ardrone1 format, we need ardrone2 format)
 */
void actuators_ardrone_set_leds(uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3)
{
  uint8_t cmd[2];
  cmd[0]=0x60 | ((led0&3)<<3) | ((led1&3)<<1) | ((led2&3)>>1);
  cmd[1]=((led2&3)<<7) | ((led3&3)<<5);
  write(mot_fd, cmd, 2);
}

void actuators_ardrone_close(void)
{
  close(mot_fd);
}

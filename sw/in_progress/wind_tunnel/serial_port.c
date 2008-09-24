/* 
   pc2rc serial port functions
   Copyright (C) 2001 Antoine Drouin

   This file is part of paparazzi.

   paparazzi is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   paparazzi is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with paparazzi; see the file COPYING.  If not, write to
   the Free Software Foundation, 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  

*/

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "serial_port.h"

#define TRACE(type,fmt,args...)
#define TRACE_ERROR 1

struct SerialPort* serial_port_new() {
  struct SerialPort* this = malloc(sizeof(struct SerialPort));
  return this;
}

/*
 * opens serial port and setup the terminal
 */
guint 
serial_port_open(struct SerialPort* this, const char* device, 
		 void(*term_conf_callback)(struct termios*, speed_t*)) {
  speed_t speed;
  if ((this->fd = open(device, O_RDWR)) < 0) {
    TRACE(TRACE_ERROR,"opening %s (%s)\n", device, strerror(errno));
    return -1;
  }
  if (tcgetattr(this->fd, &this->orig_termios) < 0) {
    TRACE(TRACE_ERROR,"getting term settings (%s)\n", strerror(errno));
    return -1;
  }   
  this->cur_termios = this->orig_termios;
  term_conf_callback(&this->cur_termios, &speed);
  if (cfsetispeed(&this->cur_termios, speed)) {
    TRACE(TRACE_ERROR,"setting term speed (%s)\n", strerror(errno));
    return -1;
  }
  if (tcsetattr(this->fd, TCSADRAIN, &this->cur_termios)) {
    TRACE(TRACE_ERROR,"setting term attributes (%s)\n", strerror(errno));
    return -1;
  }
  return 0;
}

/*
 * closes serial port and restore term settings 
 */
guint 
serial_port_close(struct SerialPort* this) {
  if (tcflush(this->fd, TCIOFLUSH)) {
    TRACE(TRACE_ERROR,"flushing (%s)\n", strerror(errno));
    return -1; 
  }
  if (tcsetattr(this->fd, TCSADRAIN, &this->orig_termios)) {        // Restore modes.
    TRACE(TRACE_ERROR,"restoring term attributes (%s)\n", strerror(errno));
    return -1; 
  }
  if (close(this->fd)) {
    TRACE(TRACE_ERROR,"closing (%s)\n", strerror(errno));
    return -1; 
  }
  return 0;
}

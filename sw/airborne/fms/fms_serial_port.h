/*
 * $Id$
 *
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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
 *
 */

#ifndef FMS_SERIAL_PORT_H
#define FMS_SERIAL_PORT_H

#include <termios.h>

struct FmsSerialPort {
  int fd;                        /* serial device fd          */
  struct termios orig_termios;   /* saved tty state structure */
  struct termios cur_termios;    /* tty state structure       */
};

extern struct FmsSerialPort* serial_port_new(void);
extern void serial_port_free(struct FmsSerialPort* me);
extern void serial_port_flush(struct FmsSerialPort* me);
extern void serial_port_flush_output(struct FmsSerialPort* me);
extern int  serial_port_open_raw(struct FmsSerialPort* me, const char* device, speed_t speed);
extern int  serial_port_open(struct FmsSerialPort* me, const char* device,
			     void(*term_conf_callback)(struct termios*, speed_t*));
extern void serial_port_close(struct FmsSerialPort* me);

#endif /* FMS_SERIAL_PORT_H */

/*
   $Id$
   Copyright (C) 2004 Pascal Brisset, Antoine Drouin

 Ocaml bindings for handling serial ports

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

#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/ioctl.h>

#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>
#include <caml/memory.h>

static int baudrates[] = { B0, B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400 };


/****************************************************************************/
/* Open serial device for requested protocoll */
/****************************************************************************/
value c_init_serial(value device, value speed, value hw_flow_control)
{
  CAMLparam3 (device, speed, hw_flow_control);
  struct termios orig_termios, cur_termios;

  int br = baudrates[Int_val(speed)];

  int fd = open(String_val(device), O_RDWR|O_NOCTTY|O_NONBLOCK);

  if (fd == -1) failwith("opening modem serial device : fd < 0");

  if (tcgetattr(fd, &orig_termios)) failwith("getting modem serial device attr");
  cur_termios = orig_termios;

  /* input modes - turn off input processing */
  cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
			    |ICRNL |IXON|IXANY|IXOFF|IMAXBEL);
  /* pas IGNCR sinon il vire les 0x0D */
  cur_termios.c_iflag |= BRKINT;

  /* output_flags - turn off output processing */
  cur_termios.c_oflag  &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);

  /* control modes */
  cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL);
  cur_termios.c_cflag |= CREAD|CS8|CLOCAL;
  if (Bool_val(hw_flow_control)) {
    cur_termios.c_cflag |= CRTSCTS;
  }
  else {
    cur_termios.c_cflag &= ~(CRTSCTS);
  }

  /* local modes */
  cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
  cur_termios.c_lflag |= NOFLSH;

  if (cfsetspeed(&cur_termios, br)) failwith("setting modem serial device speed");

  if (tcsetattr(fd, TCSADRAIN, &cur_termios)) failwith("setting modem serial device attr");

  CAMLreturn (Val_int(fd));
}

value c_set_dtr(value val_fd, value val_bit)
{
  CAMLparam2 (val_fd, val_bit);
  int status;
  int fd = Int_val(val_fd);

  ioctl(fd, TIOCMGET, &status);
  if (Bool_val(val_bit))
    status |= TIOCM_DTR;
  else
    status &= ~TIOCM_DTR;
  ioctl(fd, TIOCMSET, &status);
  CAMLreturn (Val_unit);
}


/* From the gPhoto I/O library */
value c_serial_set_baudrate(value val_fd, value speed)
{
  CAMLparam2 (val_fd, speed);
  struct termios tio;
  int fd = Int_val(val_fd);

  if (tcgetattr(fd, &tio) < 0) {
    failwith("tcgetattr");
  }
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 5;

  tio.c_lflag &= ~(ICANON | ISIG | ECHO | ECHONL | ECHOE | ECHOK);

  int br = baudrates[Int_val(speed)];

  cfsetispeed(&tio, br);
  cfsetospeed(&tio, br);
  if (tcsetattr(fd, TCSANOW | TCSAFLUSH, &tio) < 0) {
    failwith("tcsetattr");
  }
  CAMLreturn (Val_unit);
}

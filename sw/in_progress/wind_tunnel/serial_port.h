#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <glib.h>

struct SerialPort {
  int fd;                         /* serial device fd          */
  struct termios  orig_termios;   /* saved tty state structure */
  struct termios  cur_termios;    /* tty state structure       */
};


struct SerialPort* serial_port_new();
guint serial_port_open(struct SerialPort* this, const char* device,
                       void(*term_conf_callback)(struct termios*, speed_t*));
guint serial_port_close(struct SerialPort* this);


#endif

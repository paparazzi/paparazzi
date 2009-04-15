#ifndef RDYB_SERIAL_PORT_H
#define RDYB_SERIAL_PORT_H

#include <termios.h>

struct RdybSerialPort {
  int fd;                         /* serial device fd          */
  struct termios  orig_termios;   /* saved tty state structure */
  struct termios  cur_termios;    /* tty state structure       */
};

extern struct RdybSerialPort* serial_port_new( void );
extern void serial_port_free(struct RdybSerialPort* me);
extern int  serial_port_open(struct RdybSerialPort* me, const char* device, 
			     void(*term_conf_callback)(struct termios*, speed_t*));
extern void serial_port_close(struct RdybSerialPort* me);

#endif /* RDYB_SERIAL_PORT_H */

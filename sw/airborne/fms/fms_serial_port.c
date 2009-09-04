#include "fms_serial_port.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#define TRACE(type,fmt,args...)
#define TRACE_ERROR 1

struct FmsSerialPort* serial_port_new(void) {
  struct FmsSerialPort* me = malloc(sizeof(struct FmsSerialPort));
  return me;
}

void serial_port_free(struct FmsSerialPort* me) {
  free(me);
}

int  serial_port_open(struct FmsSerialPort* me, const char* device,
		      void(*term_conf_callback)(struct termios*, speed_t*)) {
  
  speed_t speed;
  if ((me->fd = open(device, O_RDWR | O_NONBLOCK)) < 0) {
    TRACE(TRACE_ERROR,"opening %s (%s)\n", device, strerror(errno));
    return -1;
  }
  if (tcgetattr(me->fd, &me->orig_termios) < 0) {
    TRACE(TRACE_ERROR,"getting term settings (%s)\n", strerror(errno));
    return -1;
  }   
  me->cur_termios = me->orig_termios;
  term_conf_callback(&me->cur_termios, &speed);
  if (cfsetispeed(&me->cur_termios, speed)) {
    TRACE(TRACE_ERROR,"setting term speed (%s)\n", strerror(errno));
    return -1;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->cur_termios)) {
    TRACE(TRACE_ERROR,"setting term attributes (%s)\n", strerror(errno));
    return -1;
  }
  return 0;
  
}

void serial_port_close(struct FmsSerialPort* me) {

  if (tcflush(me->fd, TCIOFLUSH)) {
    TRACE(TRACE_ERROR,"flushing (%s)\n", strerror(errno));
    return; 
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->orig_termios)) {        // Restore modes.
    TRACE(TRACE_ERROR,"restoring term attributes (%s)\n", strerror(errno));
    return; 
  }
  if (close(me->fd)) {
    TRACE(TRACE_ERROR,"closing (%s)\n", strerror(errno));
    return; 
  }
  return;


}

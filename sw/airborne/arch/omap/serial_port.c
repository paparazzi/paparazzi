#include "serial_port.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

//#define TRACE(type,fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(type,fmt,args...)
#define TRACE_ERROR 1

struct SerialPort* serial_port_new(void) {
  struct SerialPort* me = malloc(sizeof(struct SerialPort));
  return me;
}

void serial_port_free(struct SerialPort* me) {
  free(me);
}


void serial_port_flush(struct SerialPort* me) {
  /*
   * flush any input that might be on the port so we start fresh.
   */
  if (tcflush(me->fd, TCIFLUSH)) {
    TRACE(TRACE_ERROR,"%s, set term attr failed: %s (%d)\n", "", strerror(errno), errno);
    fprintf(stderr, "flush (%d) failed: %s (%d)\n", me->fd, strerror(errno), errno);
  }
}

void serial_port_flush_output(struct SerialPort* me) {
  /*
   * flush any input that might be on the port so we start fresh.
   */
  if (tcflush(me->fd, TCOFLUSH)) {
    TRACE(TRACE_ERROR,"%s, set term attr failed: %s (%d)\n", "", strerror(errno), errno);
    fprintf(stderr, "flush (%d) failed: %s (%d)\n", me->fd, strerror(errno), errno);
  }
}

int  serial_port_open_raw(struct SerialPort* me, const char* device, speed_t speed) {
  if ((me->fd = open(device, O_RDWR | O_NONBLOCK | O_NOCTTY)) < 0) {
    TRACE(TRACE_ERROR,"%s, open failed: %s (%d)\n", device, strerror(errno), errno);
    return -1;
  }
  if (tcgetattr(me->fd, &me->orig_termios) < 0) {
    TRACE(TRACE_ERROR,"%s, get term settings failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  me->cur_termios = me->orig_termios;
  /* input modes  */
  me->cur_termios.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
                 |ICRNL |IUCLC|IXON|IXANY|IXOFF|IMAXBEL);
  me->cur_termios.c_iflag |= IGNPAR;
  /* control modes*/
  me->cur_termios.c_cflag &= ~(CSIZE|PARENB|CRTSCTS|PARODD|HUPCL|CSTOPB);
  me->cur_termios.c_cflag |= CREAD|CS8|CLOCAL;
  /* local modes  */
  me->cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
  me->cur_termios.c_lflag |= NOFLSH;
  if (cfsetispeed(&me->cur_termios, speed)) {
    TRACE(TRACE_ERROR,"%s, set term speed failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->cur_termios)) {
    TRACE(TRACE_ERROR,"%s, set term attr failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  serial_port_flush(me);
  return 0;
}

int  serial_port_open(struct SerialPort* me, const char* device,
          void(*term_conf_callback)(struct termios*, speed_t*)) {

  speed_t speed;
  if ((me->fd = open(device, O_RDWR | O_NONBLOCK)) < 0) {
    TRACE(TRACE_ERROR,"%s, open failed: %s (%d)\n", device, strerror(errno), errno);
    return -1;
  }
  if (tcgetattr(me->fd, &me->orig_termios) < 0) {
    TRACE(TRACE_ERROR,"%s, get term settings failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  me->cur_termios = me->orig_termios;
  term_conf_callback(&me->cur_termios, &speed);
  if (cfsetispeed(&me->cur_termios, speed)) {
    TRACE(TRACE_ERROR,"%s, set term speed failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->cur_termios)) {
    TRACE(TRACE_ERROR,"%s, set term attr failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  serial_port_flush(me);
  return 0;

}

void serial_port_close(struct SerialPort* me) {

  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0)
    return;
  if (tcflush(me->fd, TCIOFLUSH)) {
    TRACE(TRACE_ERROR,"flushing (%s) (%d)\n", strerror(errno), errno);
    close(me->fd);
    return;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->orig_termios)) {        // Restore modes.
    TRACE(TRACE_ERROR,"restoring term attributes (%s) (%d)\n", strerror(errno), errno);
    close(me->fd);
    return;
  }
  if (close(me->fd)) {
    TRACE(TRACE_ERROR,"closing %s (%d)\n", strerror(errno), errno);
    return;
  }
  return;


}

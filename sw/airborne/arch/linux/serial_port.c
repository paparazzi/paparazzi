#include "serial_port.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/**
 * some definitions from PPRZ uart driver as we can't include it directly
 */

// for definition of baud rates
#if !USE_ARBITRARY_BAUDRATE
#include <termios.h>
#else
#include <linux/termios.h>
#endif

// strange speed for SBUS
#ifndef B100000
#define B100000 100000
#endif

// for conversion between linux baud rate definition and actual speed
static inline int uart_speed(int def)
{
  switch (def) {
    case B1200: return 1200;
    case B2400: return 2400;
    case B4800: return 4800;
    case B9600: return 9600;
    case B19200: return 19200;
    case B38400: return 38400;
    case B57600: return 57600;
    case B100000: return 100000;
    case B115200: return 115200;
    case B230400: return 230400;
#ifdef B921600
    case B921600: return 921600;
#endif
    default: return 9600;
  }
}

#define UBITS_7 7
#define UBITS_8 8

#define USTOP_1 1
#define USTOP_2 2

#define UPARITY_NO    0
#define UPARITY_ODD   1
#define UPARITY_EVEN  2

///////////////////////




// IUCLC flag translates upper case to lower case (actually needed here?)
// but is not in POSIX and OSX
#ifndef IUCLC
#define IUCLC 0
#endif

//#define TRACE(type,fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(type,fmt,args...)
#define TRACE_ERROR 1

struct SerialPort *serial_port_new(void)
{
  struct SerialPort *me = malloc(sizeof(struct SerialPort));
  return me;
}

void serial_port_free(struct SerialPort *me)
{
  free(me);
}


#if !USE_ARBITRARY_BAUDRATE
// classic set baud function

void serial_port_flush(struct SerialPort *me)
{
  /*
   * flush any input that might be on the port so we start fresh.
   */
  if (tcflush(me->fd, TCIFLUSH)) {
    TRACE(TRACE_ERROR, "%s, set term attr failed: %s (%d)\n", "", strerror(errno), errno);
    fprintf(stderr, "flush (%d) failed: %s (%d)\n", me->fd, strerror(errno), errno);
  }
}

void serial_port_flush_output(struct SerialPort *me)
{
  /*
   * flush any input that might be on the port so we start fresh.
   */
  if (tcflush(me->fd, TCOFLUSH)) {
    TRACE(TRACE_ERROR, "%s, set term attr failed: %s (%d)\n", "", strerror(errno), errno);
    fprintf(stderr, "flush (%d) failed: %s (%d)\n", me->fd, strerror(errno), errno);
  }
}

int  serial_port_open_raw(struct SerialPort *me, const char *device, speed_t speed)
{
  if ((me->fd = open(device, O_RDWR | O_NONBLOCK | O_NOCTTY)) < 0) {
    TRACE(TRACE_ERROR, "%s, open failed: %s (%d)\n", device, strerror(errno), errno);
    return -1;
  }
  if (tcgetattr(me->fd, &me->orig_termios) < 0) {
    TRACE(TRACE_ERROR, "%s, get term settings failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  me->cur_termios = me->orig_termios;
  /* input modes  */
  me->cur_termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | IGNCR
                               | ICRNL | IUCLC | IXON | IXANY | IXOFF | IMAXBEL);
  me->cur_termios.c_iflag |= IGNPAR;
  /* control modes*/
  me->cur_termios.c_cflag &= ~(CSIZE | PARENB | CRTSCTS | PARODD | HUPCL | CSTOPB);
  me->cur_termios.c_cflag |= CREAD | CS8 | CLOCAL;
  /* local modes  */
  me->cur_termios.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | FLUSHO | PENDIN);
  me->cur_termios.c_lflag |= NOFLSH;
  /* output modes */
  me->cur_termios.c_oflag &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);
  /* no buffering */
  me->cur_termios.c_cc[VTIME] = 0;
  me->cur_termios.c_cc[VMIN] = 0;

  if (cfsetispeed(&me->cur_termios, speed)) {
    TRACE(TRACE_ERROR, "%s, set term speed failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->cur_termios)) {
    TRACE(TRACE_ERROR, "%s, set term attr failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  serial_port_flush(me);
  return 0;
}

int  serial_port_open(struct SerialPort *me, const char *device,
                      void(*term_conf_callback)(struct termios *, speed_t *))
{

  speed_t speed;
  if ((me->fd = open(device, O_RDWR | O_NONBLOCK)) < 0) {
    TRACE(TRACE_ERROR, "%s, open failed: %s (%d)\n", device, strerror(errno), errno);
    return -1;
  }
  if (tcgetattr(me->fd, &me->orig_termios) < 0) {
    TRACE(TRACE_ERROR, "%s, get term settings failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  me->cur_termios = me->orig_termios;
  term_conf_callback(&me->cur_termios, &speed);
  if (cfsetispeed(&me->cur_termios, speed)) {
    TRACE(TRACE_ERROR, "%s, set term speed failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->cur_termios)) {
    TRACE(TRACE_ERROR, "%s, set term attr failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  serial_port_flush(me);
  return 0;

}

void serial_port_close(struct SerialPort *me)
{

  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0) {
    return;
  }
  if (tcflush(me->fd, TCIOFLUSH)) {
    TRACE(TRACE_ERROR, "flushing (%s) (%d)\n", strerror(errno), errno);
    close(me->fd);
    return;
  }
  if (tcsetattr(me->fd, TCSADRAIN, &me->orig_termios)) {        // Restore modes.
    TRACE(TRACE_ERROR, "restoring term attributes (%s) (%d)\n", strerror(errno), errno);
    close(me->fd);
    return;
  }
  if (close(me->fd)) {
    TRACE(TRACE_ERROR, "closing %s (%d)\n", strerror(errno), errno);
    return;
  }
  return;


}

int serial_port_set_baudrate(struct SerialPort *me, speed_t speed)
{
  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0) {
    return -1;
  }
  if (cfsetispeed(&me->cur_termios, speed)) {
    TRACE(TRACE_ERROR, "%s, set term speed failed: %s (%d)\n", device, strerror(errno), errno);
    close(me->fd);
    return -1;
  }
  return 0;
}

int serial_port_set_bits_stop_parity(struct SerialPort *me, const int bits, const int stop, const int parity)
{
  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0) {
    return -1;
  }
  // clear data bits
  me->cur_termios.c_cflag &= ~CSIZE;
  if (bits == UBITS_7) {
    me->cur_termios.c_cflag |= CS7;
  } else {
    me->cur_termios.c_cflag |= CS8;
  }
  // set stops
  if (stop == USTOP_1) {
    me->cur_termios.c_cflag &= ~CSTOPB;
  } else {
    me->cur_termios.c_cflag |= CSTOPB;
  }
  // set parity
  if (parity == UPARITY_EVEN) {
    me->cur_termios.c_cflag |= PARENB;
    me->cur_termios.c_cflag &= ~PARODD;
  } else if (parity == UPARITY_ODD) {
    me->cur_termios.c_cflag |= PARENB;
    me->cur_termios.c_cflag |= PARODD;
  } else {
    me->cur_termios.c_cflag &= ~PARENB;
  }
  // set new parameters
  if (tcsetattr(me->fd, TCSADRAIN, &me->cur_termios)) {
    TRACE(TRACE_ERROR, "setting term attributes (%s) (%d)\n", strerror(errno), errno);
    return -2;
  }
  return 0;
}

#else // USE_ARBITRARY_BAUDRATE
// use termios2 interface
// needed to set arbitrary speed
// it is the case for the SBUS RC input

int ioctl(int d, int request, ...); // avoid warning
int tcflush (int __fd, int __queue_selector); // avoid warning

void serial_port_flush(struct SerialPort *me)
{
  /*
   * flush any input that might be on the port so we start fresh.
   */
  if (tcflush(me->fd, TCIFLUSH)) {
    TRACE(TRACE_ERROR, "%s, set term attr failed: %s (%d)\n", "", strerror(errno), errno);
    fprintf(stderr, "flush (%d) failed: %s (%d)\n", me->fd, strerror(errno), errno);
  }
}

void serial_port_flush_output(struct SerialPort *me)
{
  /*
   * flush any input that might be on the port so we start fresh.
   */
  if (tcflush(me->fd, TCOFLUSH)) {
    TRACE(TRACE_ERROR, "%s, set term attr failed: %s (%d)\n", "", strerror(errno), errno);
    fprintf(stderr, "flush (%d) failed: %s (%d)\n", me->fd, strerror(errno), errno);
  }
}

int  serial_port_open_raw(struct SerialPort *me, const char *device, speed_t speed)
{
  if ((me->fd = open(device, O_RDWR | O_NONBLOCK | O_NOCTTY)) < 0) {
    TRACE(TRACE_ERROR, "%s, open failed: %s (%d)\n", device, strerror(errno), errno);
    return -1;
  }

  if (ioctl(me->fd, TCGETS2, &me->orig_termios)) {
    perror("TCGETS2");
    close(me->fd);
    return -1;
  }

  me->cur_termios = me->orig_termios;
  /* input modes  */
  me->cur_termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | IGNCR
                               | ICRNL | IUCLC | IXON | IXANY | IXOFF | IMAXBEL);
  me->cur_termios.c_iflag |= IGNPAR;
  /* control modes*/
  me->cur_termios.c_cflag &= ~(CSIZE | PARENB | CRTSCTS | PARODD | HUPCL | CSTOPB);
  me->cur_termios.c_cflag |= CREAD | CS8 | CLOCAL;
  /* local modes  */
  me->cur_termios.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | FLUSHO | PENDIN);
  me->cur_termios.c_lflag |= NOFLSH;
  /* output modes */
  me->cur_termios.c_oflag &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);
  /* no buffering */
  me->cur_termios.c_cc[VTIME] = 0;
  me->cur_termios.c_cc[VMIN] = 0;

  me->cur_termios.c_cflag &= ~CBAUD;
  me->cur_termios.c_cflag |= BOTHER;
  me->cur_termios.c_ispeed = uart_speed(speed); // set real speed
  me->cur_termios.c_ospeed = uart_speed(speed); // set real speed

  if (ioctl(me->fd, TCSETS2, &me->cur_termios)) {
    perror("TCSETS2");
    close(me->fd);
    return -1;
  }

  serial_port_flush(me);

  if (ioctl(me->fd, TCGETS2, &me->cur_termios))
  {
    perror("TCGETS2");
    return 5;
  }

  return 0;
}

int  serial_port_open(struct SerialPort *me, const char *device,
                      void(*term_conf_callback)(struct termios *, speed_t *))
{
  (void)me;
  (void)device;
  (void)term_conf_callback;
  return 0;
}

void serial_port_close(struct SerialPort *me)
{

  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0) {
    return;
  }
  if (tcflush(me->fd, TCIOFLUSH)) {
    TRACE(TRACE_ERROR, "flushing (%s) (%d)\n", strerror(errno), errno);
    close(me->fd);
    return;
  }
  if (ioctl(me->fd, TCSETS2, &me->orig_termios)) {        // Restore modes.
    TRACE(TRACE_ERROR, "restoring term attributes (%s) (%d)\n", strerror(errno), errno);
    perror("TCSETS2");
    close(me->fd);
    return;
  }
  if (close(me->fd)) {
    TRACE(TRACE_ERROR, "closing %s (%d)\n", strerror(errno), errno);
    return;
  }
  return;

}

int serial_port_set_baudrate(struct SerialPort *me, speed_t speed)
{
  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0) {
    return -1;
  }

  me->cur_termios.c_cflag &= ~CBAUD;
  me->cur_termios.c_cflag |= BOTHER;
  me->cur_termios.c_ispeed = uart_speed(speed); // set real speed
  me->cur_termios.c_ospeed = uart_speed(speed); // set real speed

  // set new parameters
  if (ioctl(me->fd, TCSETS2, &me->cur_termios)) {
    perror("TCSETS2");
    close(me->fd);
    return -1;
  }
  return 0;
}


int serial_port_set_bits_stop_parity(struct SerialPort *me, const int bits, const int stop, const int parity)
{
  /* if null pointer or file descriptor indicates error just bail */
  if (!me || me->fd < 0) {
    return -1;
  }
  // clear data bits
  me->cur_termios.c_cflag &= ~CSIZE;
  if (bits == UBITS_7) {
    me->cur_termios.c_cflag |= CS7;
  } else {
    me->cur_termios.c_cflag |= CS8;
  }
  // set stops
  if (stop == USTOP_1) {
    me->cur_termios.c_cflag &= ~CSTOPB;
  } else {
    me->cur_termios.c_cflag |= CSTOPB;
  }
  // set parity
  if (parity == UPARITY_EVEN) {
    me->cur_termios.c_cflag |= PARENB;
    me->cur_termios.c_cflag &= ~PARODD;
    me->cur_termios.c_iflag |= INPCK;
  } else if (parity == UPARITY_ODD) {
    me->cur_termios.c_cflag |= PARENB;
    me->cur_termios.c_cflag |= PARODD;
    me->cur_termios.c_iflag |= INPCK;
  } else {
    me->cur_termios.c_cflag &= ~PARENB;
    me->cur_termios.c_iflag &= ~INPCK;
  }
  // set new parameters
  if (ioctl(me->fd, TCSETS2, &me->cur_termios)) {
    perror("TCSETS2");
    close(me->fd);
    return -1;
  }
  return 0;
}

#endif


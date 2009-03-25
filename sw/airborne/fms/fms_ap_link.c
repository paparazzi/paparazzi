#include "fms_ap_link.h"

#include <stdlib.h>

#include "fms_debug.h"
#include "fms_serial_port.h"


static void configure_term(struct termios *termios, speed_t *speed);


struct FmsApLink* ap_link_new(const char* device) {

  struct FmsApLink* me = malloc(sizeof(struct FmsApLink));
  me->sp = serial_port_new();
  if (serial_port_open(me->sp, device, configure_term)) {
    ap_link_free(me);
    return NULL;
  }
  


  return me;

}

void ap_link_free(struct FmsApLink* me) {
  if (me->sp) {
    serial_port_close(me->sp);
    serial_port_free(me->sp);
  }
  free(me);
}


void ap_link_parse(struct FmsApLink* me, int nb_bytes) {

  TRACE(TRACE_DEBUG, "ap link parsing %d bytes \n", nb_bytes);

}



static void configure_term(struct termios *termios, speed_t *speed) {
  /* input modes  */
  termios->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
                 |ICRNL |IUCLC|IXON|IXANY|IXOFF|IMAXBEL);
  termios->c_iflag |= IGNPAR;
  /* control modes*/
  termios->c_cflag &= ~(CSIZE|PARENB|CRTSCTS|PARODD|HUPCL);
  termios->c_cflag |= CREAD|CS8|CSTOPB|CLOCAL;
  /* local modes  */
  termios->c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
  termios->c_lflag |= NOFLSH;
  /* speed        */
  *speed = B115200;
}




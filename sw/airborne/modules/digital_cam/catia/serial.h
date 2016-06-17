

extern int fd;

#include "std.h"

int serial_init(char *port_name);

static inline int ttyUSB0ChAvailable(void)
{
  return false;
}

#define ttyUSB0Transmit(_char)     \
  {                                  \
    char c = _char;                  \
    int __attribute__((unused)) ret = write(fd,&c,1);        \
  }

#define ttyUSB0Getch() ({char c;int ret=read(fd, &c,1);c;})

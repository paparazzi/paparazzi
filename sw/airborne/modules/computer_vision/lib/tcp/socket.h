#ifndef SOCKET_H
#define SOCKET_H

#include <unistd.h>             /*  for ssize_t data type  */
#include "./video/video.h"

#define LISTENQ        (1024)   /*  Backlog for listen()   */


/*  Function declarations  */
int initSocket(void) ;
ssize_t Readline(void *vptr, size_t maxlen);
ssize_t Writeline(struct img_struct *img, size_t maxlen);
int closeSocket(void);

#endif  /*  SOCKET_H  */


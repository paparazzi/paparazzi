#include "socket.h"
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */
#include <errno.h>
#include <string.h>     /* memset */

#include <stdlib.h>
#include <stdio.h>




/*  Global constants  */

#define ECHO_PORT          (2002)
#define MAX_LINE           (1000)

/*  Global variables  */
int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */

int closeSocket(void)
{
  return close(conn_s);
}

int initSocket(void)
{

  /*  Create the listening socket  */

  if ((list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    fprintf(stderr, "ECHOSERV: Error creating listening socket.\n");
    return -1;
  }


  /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family      = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port        = htons(ECHO_PORT);

  /*  Bind our socket addresss to the
  listening socket, and call listen()  */

  if (bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0) {
    fprintf(stderr, "ECHOSERV: Error calling bind()\n");
    exit(EXIT_FAILURE);
  }

  if (listen(list_s, LISTENQ) < 0) {
    fprintf(stderr, "ECHOSERV: Error calling listen()\n");
    exit(EXIT_FAILURE);
  }



  /*  Wait for a connection, then accept() it  */
  if ((conn_s = accept(list_s, NULL, NULL)) < 0) {
    fprintf(stderr, "ECHOSERV: Error calling function accept()\n");
    return -1;
  }
  printf("Connected!\n");
  return 1;
}

/*  Read a line from a socket  */

ssize_t Readline_socket(void *vptr, size_t maxlen)
{
  ssize_t n, rc;
  char    c, *buffer;

  buffer = vptr;

  for (n = 1; n < maxlen; n++) {

    if ((rc = read(conn_s, &c, 1)) == 1) {
      *buffer++ = c;
      if (c == '\n') {
        break;
      }
    } else if (rc == 0) {
      if (n == 1) {
        return 0;
      } else {
        break;
      }
    } else {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
  }

  *buffer = 0;
  return n;
}


/*  Write a line to a socket  */

ssize_t Writeline_socket(struct img_struct *img, size_t n)
{
  size_t      nleft;
  ssize_t     nwritten;
  unsigned char *buf1 = img->buf;
  nleft  = n;

  //printf("socket buf1 %d, %d, %d, %d, %d, %d\n",buf1[0],buf1[1],buf1[2],buf1[3],buf1[4],buf1[5]);

  while (nleft > 0) {
    if ((nwritten = write(conn_s, buf1, nleft)) <= 0) {
      if (errno == EINTR) {
        nwritten = 0;
      } else {
        return -1;
      }
    }
    nleft  -= nwritten;
    buf1 += nwritten;
  }

  return n;

}





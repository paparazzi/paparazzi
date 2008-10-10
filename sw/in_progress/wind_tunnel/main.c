#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "serial_port.h"

#define BUF_SIZE 512
#define AC_ID 2

static struct SerialPort* sp;
static GIOChannel* ioc;

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
  *speed = B9600;
}

static void dump_buf ( int len, char* buf) {
  
  static gchar buf2[BUF_SIZE];
  static int cur_len = 0;
  int i;

  for (i=0; i<len; i++) {
    if (buf[i] != '\r') {
      if (buf[i] != 'T' && buf[i] != '.'){
	buf2[cur_len] = buf[i];
	cur_len++;
      }
    }
    else {
      int force;
      buf2[cur_len] = 0;
      g_message("buf2: %s",buf2);
      sscanf(buf2,"%d",&force);
      IvySendMsg("%d X_FORCE %d", AC_ID, force);
      cur_len = 0;
    }
  }

  //g_message("read %s and %s",buf,buf2);
}

static gboolean on_serial_data_received(GIOChannel *source,
                                        GIOCondition condition,
                                        gpointer data) {

  static gchar buf[BUF_SIZE];
  gsize len;
  //int fd = open("data", O_RDWR | O_APPEND | O_CREAT);
  
  g_io_channel_read_chars(source, buf, BUF_SIZE, &len, NULL);
  dump_buf(len, buf);
  
  return TRUE;
}

gboolean timeout_callback(gpointer data) {

  const char* msg = "GT\n";
  gsize bw;
  
  g_io_channel_write_chars(ioc, msg, 3, &bw, NULL); 
  g_io_channel_flush(ioc, NULL);
  
  return TRUE;
  
}

int main ( int argc, char** argv) {

  sp = serial_port_new();
  serial_port_open(sp, "/dev/ttyUSB1", configure_term);
  
  ioc = g_io_channel_unix_new(sp->fd);
  g_io_channel_set_encoding(ioc, NULL, NULL);
  g_io_channel_set_flags (ioc,G_IO_FLAG_NONBLOCK, NULL );
  g_io_add_watch (ioc, G_IO_IN, on_serial_data_received, NULL);


  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("Wind_Tunnel", "Wind_Tunnel READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  g_timeout_add(500, timeout_callback, NULL);
  
  g_main_loop_run(ml);

  return 0;
}


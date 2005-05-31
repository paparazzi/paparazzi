#include "wavecard_glib_utils.h"
#include "wavecard.h"
#include <signal.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>



struct WavecardAddr hosts[] = {
  { { 0x01, 0x18, 0x04, 0xc0, 0x01, 0x34 }, "ground_station"},
  { { 0x01, 0x18, 0x04, 0xc0, 0x01, 0x35 }, "twinstar3"},
  { { 0x01, 0x18, 0x04, 0xc0, 0x01, 0x2d }, "microjet4"},
  { { 0x01, 0x18, 0x04, 0xc0, 0x01, 0x36 }, "plaster1"},
  { { 0x01, 0x18, 0x04, 0xc0, 0x01, 0x37 }, "gorazoptere"}
};
uint8_t  nb_hosts = 5;

struct WavecardAddr* get_host_by_addr ( uint8_t* addr) {
  uint8_t i;
  while (i<nb_hosts) {
    if (!memcmp(addr, hosts[i].addr, WC_ADDR_LEN))
      return &hosts[i];
    i++;
  }
  return NULL;
}

struct WavecardAddr* get_host_by_id ( uint8_t id) {
  return (id < nb_hosts) ? &hosts[id] : NULL; 
}

uint8_t              get_nb_hosts     () { return nb_hosts;}

GIOChannel* open_serial_port( const gchar* serial_dev) {
  GIOChannel* input_channel;
  struct termios  orig_termios;
  struct termios  cur_termios;
  int fd = open(serial_dev, O_RDWR | O_NONBLOCK);

   if (fd == -1) {
     g_message("opening serial device %s : %s", serial_dev, strerror(errno));
     return NULL;
   } 
   
   if (tcgetattr(fd, &orig_termios)) {
     g_message("getting serial device attr (%s) : %s", serial_dev, strerror(errno));
     return NULL;
   }
   cur_termios = orig_termios;
 
   /* input modes */
   cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
			    |ICRNL |IUCLC|IXON|IXANY|IXOFF|IMAXBEL);
   /* pas IGNCR sinon il vire les 0x0D */
   cur_termios.c_iflag |= BRKINT;
   
   /* output_flags */
   cur_termios.c_oflag  &=~(OPOST|OLCUC|ONLCR|OCRNL|ONOCR|ONLRET|OFILL|OFDEL);

   /* control modes */
   cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL|CRTSCTS);
   cur_termios.c_cflag |= CREAD|CS8|CLOCAL;
   
   /* local modes */
   cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
   cur_termios.c_lflag |= NOFLSH;
   
   if (cfsetispeed(&cur_termios, B9600)) {
     g_message("setting serial device speed (%s) : %s", serial_dev, strerror(errno));
     return NULL;
  }

  if (tcsetattr(fd, TCSADRAIN, &cur_termios)) {
    g_message("setting serial device attr (%s) : %s", serial_dev, strerror(errno));
    return NULL;
  }
  
  input_channel = g_io_channel_unix_new(fd);
  g_io_channel_set_encoding(input_channel, NULL, NULL);
  return input_channel;
}

static void print_hex(gsize len, guchar* buf) {
  const char d2h[] = "0123456789ABCDEF";
  int i=0;
  for (i=0; i<len; i++)
    printf("%c%c ", d2h[(int)buf[i]/16], d2h[(int)buf[i]%16]);
  printf("\n");
}

#define LINE_LEN 8
void print_both(gsize len, guchar* buf) {
  const char d2h[] = "0123456789ABCDEF";
  int i,j;
  for (i=0; i<len && i < LINE_LEN; i++) {
    printf("%c%c ", d2h[(int)buf[i]/16], d2h[(int)buf[i]%16]);
  }
  if (i<LINE_LEN)
    for (j=0; j<LINE_LEN-i; j++)
      printf("   ");
  printf("\t");
  for (i=0; i<len && i < LINE_LEN; i++)
    if (isalnum(buf[i])) 
      printf("%c", buf[i]);
    else
      printf(".");
  printf("\n");
  if (i<len) print_both(len-i, buf+i);
}

void wc_glib_print_addr(uint8_t* addr) {
  int i;
  for(i = 0; i < WC_ADDR_LEN ; i++)
    printf("%02x ", addr[i]);
}

void wc_glib_append_addr(GString* str, uint8_t* addr) {
  int i;
  for(i = 0; i < WC_ADDR_LEN ; i++)
    g_string_append_printf(str, "%02x ", addr[i]);
}

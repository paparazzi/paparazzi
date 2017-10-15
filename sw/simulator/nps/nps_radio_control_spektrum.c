#include "nps_radio_control_spektrum.h"
#include "nps_radio_control.h"

#include <glib.h>
#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

// IUCLC flag translates upper case to lower case (actually needed here?)
// but is not in POSIX and OSX
#ifndef IUCLC
#define IUCLC 0
#endif

#define CHANNEL_OF_FRAME(i) ((((frame_buf[2*i]<<8) + frame_buf[2*i+1])&0x03FF)-512)

static int sp_fd;

static gboolean on_serial_data_received(GIOChannel *source,
                                        GIOCondition condition,
                                        gpointer data);
static void parse_data(char *buf, int len);
static void handle_frame(void);


int nps_radio_control_spektrum_init(const char *device)
{

  if ((sp_fd = open(device, O_RDWR | O_NONBLOCK)) < 0) {
    printf("opening %s (%s)\n", device, strerror(errno));
    return -1;
  }
  struct termios termios;
  termios.c_iflag = 0; // properly initialize variable
  /* input modes  */
  termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | IGNCR
                       | ICRNL | IUCLC | IXON | IXANY | IXOFF | IMAXBEL);
  termios.c_iflag |= IGNPAR;

  termios.c_cflag = 0; // properly initialize variable
  /* control modes*/
  termios.c_cflag &= ~(CSIZE | PARENB | CRTSCTS | PARODD | HUPCL);
  termios.c_cflag |= CREAD | CS8 | CSTOPB | CLOCAL;

  termios.c_lflag = 0; // properly initialize variable
  /* local modes  */
  termios.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | FLUSHO | PENDIN);
  termios.c_lflag |= NOFLSH;
  /* speed        */
  speed_t speed = B115200;

  if (cfsetispeed(&termios, speed)) {
    printf("setting term speed (%s)\n", strerror(errno));
    return -1;
  }
  if (tcsetattr(sp_fd, TCSADRAIN, &termios)) {
    printf("setting term attributes (%s)\n", strerror(errno));
    return -1;
  }
  GIOChannel *channel = g_io_channel_unix_new(sp_fd);
  g_io_channel_set_encoding(channel, NULL, NULL);
  g_io_add_watch(channel, G_IO_IN , on_serial_data_received, NULL);
  return 0;
}




static gboolean on_serial_data_received(GIOChannel *source,
                                        GIOCondition condition __attribute__((unused)),
                                        gpointer data __attribute__((unused)))
{
  char buf[255];
  gsize bytes_read;
  GError *_err = NULL;
  GIOStatus st = g_io_channel_read_chars(source, buf, 255, &bytes_read, &_err);
  if (!_err) {
    if (st == G_IO_STATUS_NORMAL) {
      parse_data(buf, bytes_read);
    }
  } else {
    printf("error reading serial: %s\n", _err->message);
    g_error_free(_err);
  }
  return TRUE;
}


#define SYNC_1 0x03
#define SYNC_2 0x12

#define STA_UNINIT     0
#define STA_GOT_SYNC_1 1
#define STA_GOT_SYNC_2 2

uint8_t status = STA_UNINIT;

#define FRAME_LEN 14
static uint8_t frame_buf[FRAME_LEN];
static uint32_t idx = 0;

static void parse_data(char *buf, int len)
{
  int i;
  for (i = 0; i < len; i++) {
    int8_t c = buf[i ];
    switch (status) {
      case STA_UNINIT:
        if (c == SYNC_1) {
          status = STA_GOT_SYNC_1;
        }
        break;
      case STA_GOT_SYNC_1:
        if (c == SYNC_2) {
          status = STA_GOT_SYNC_2;
          idx = 0;
        } else {
          status = STA_UNINIT;
        }
        break;
      case STA_GOT_SYNC_2:
        frame_buf[idx] = c;
        idx++;
        if (idx == FRAME_LEN) {
          status = STA_UNINIT;
          handle_frame();
        }
        break;
      default:
        status = STA_UNINIT;
        break;
    }
  }
}

static void handle_frame(void)
{
  nps_radio_control.roll = (float)CHANNEL_OF_FRAME(0) / -340.;
  nps_radio_control.throttle = (float)(CHANNEL_OF_FRAME(1) + 340) / 680.;
  nps_radio_control.pitch = (float)CHANNEL_OF_FRAME(2) / -340.;
  nps_radio_control.yaw = (float)CHANNEL_OF_FRAME(3) / -340.;
  nps_radio_control.mode = (float)CHANNEL_OF_FRAME(5) / 340.;
  //  printf("%f %f %f %f %f \n", nps_radio_control.roll, nps_radio_control.throttle, nps_radio_control.pitch, nps_radio_control.yaw, nps_radio_control.mode);
}

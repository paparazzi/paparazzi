#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <Ivy/version.h>

//////////////////////////////////////////////////////////////////////////////////
// SETTINGS
//////////////////////////////////////////////////////////////////////////////////

// Serial Repeat Rate
long delay = 1000;

// local_uav Number
unsigned char send_ac_id = 5;

//////////////////////////////////////////////////////////////////////////////////
// local_uav DATA
//////////////////////////////////////////////////////////////////////////////////


struct _uav_type_
{
  // Header
  unsigned char header;

  // Data
  unsigned char ac_id;
  short int phi, theta, psi;
  int utm_east,utm_north,utm_z;
  unsigned char utm_zone;
  
  // Footer
  unsigned char footer;
} 
__attribute__((packed))

local_uav, remote_uav;

//////////////////////////////////////////////////////////////////////////////////
// IVY Reader
//////////////////////////////////////////////////////////////////////////////////


static void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  unsigned char id = atoi(argv[0]);

  if (id != send_ac_id)
  {
    printf("NEGLECT: %d\n",id);
    return;
  }

/*
   <message name="ATTITUDE" id="6">
     <field name="phi"   type="float" unit="rad" alt_unit="deg"/>
     <field name="psi"   type="float" unit="rad" alt_unit="deg"/>
     <field name="theta" type="float" unit="rad" alt_unit="deg"/>
   </message>
*/

  local_uav.ac_id = id;
  local_uav.phi   = (short int) (atof(argv[1]) * 1000.0);
  local_uav.theta = (short int) (atof(argv[3]) * 1000.0);
  local_uav.psi   = (short int) (atof(argv[2]) * 1000.0);

}

static void on_Gps(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  unsigned char id = atoi(argv[0]);

  if (id != send_ac_id)
  {
    printf("NEGLECT: %d\n",id);
    return;
  }

/*
   <message name="GPS" id="8">
     <field name="mode"       type="uint8"  unit="byte_mask"/>
     <field name="utm_east"   type="int32"  unit="cm" alt_unit="m"/>
     <field name="utm_north"  type="int32"  unit="cm" alt_unit="m"/>
     <field name="course"     type="int16"  unit="decideg" alt_unit="deg"/>
     <field name="alt"        type="int32"  unit="mm" alt_unit="m"/>
     <field name="speed"      type="uint16" unit="cm/s" alt_unit="m/s"/>
     <field name="climb"      type="int16"  unit="cm/s" alt_unit="m/s"/>
     <field name="week"       type="uint16" unit="weeks"/>
     <field name="itow"       type="uint32" unit="ms"/>
     <field name="utm_zone"   type="uint8"/>
     <field name="gps_nb_err" type="uint8"/>
   </message>
*/

  local_uav.ac_id = id;
  local_uav.utm_east = atoi(argv[2]);
  local_uav.utm_north = atoi(argv[3]);
  local_uav.utm_z = atoi(argv[5]);
  local_uav.utm_zone = atoi(argv[10]);

  printf("ATTITUDE ac=%d phi=%d theta=%d psi=%d ",local_uav.ac_id, local_uav.phi, local_uav.theta, local_uav.psi);
  printf("GPS ac=%d %d %d %d %d\n",local_uav.ac_id, local_uav.utm_east, local_uav.utm_north, local_uav.utm_z, local_uav.utm_zone);

}

//////////////////////////////////////////////////////////////////////////////////
// SERIAL PORT
//////////////////////////////////////////////////////////////////////////////////

// pointer
int fd;

/// Open
void open_port(const char* device) {
  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "open_port: unable to open device %s - ", device);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  // setup connection options
  struct termios options;

  // get the current options
  tcgetattr(fd, &options);

  // set local mode, enable receiver, set comm. options:
  // 8 data bits, 1 stop bit, no parity, 9600 Baud
  options.c_cflag = CLOCAL | CREAD | CS8 | B9600;

  // write options back to port
  tcsetattr(fd, TCSANOW, &options);
}

void send_port(void)
{
  int bytes = write(fd, &local_uav, sizeof(local_uav));
  printf("SENT: %d bytes\n",bytes);
}

void read_port(void)
{
}

void close_port(void)
{
  close(fd);
}

//////////////////////////////////////////////////////////////////////////////////
// TIMER
//////////////////////////////////////////////////////////////////////////////////

// Timer
void handle_timer (TimerId id, void *data, unsigned long delta) {
  printf("TIMER\n");
  send_port();
}

TimerId tid;

/// Handler for Ctrl-C, exits the main loop
void sigint_handler(int sig) {
  printf("\nCLEAN STOP\n");
  IvyStop();
  TimerRemove(tid);
  close_port(); 
}

//////////////////////////////////////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////////////////////////////////////

int main ( int argc, char** argv) 
{
  int s = sizeof(local_uav);
 
  if (argc < 3)
  {
    printf("Use: ivy2serial ac_id serial_device\n");
    return -1;
  }
  
  send_ac_id = atoi(argv[1]);

  printf("Listening to AC=%d, \nSending Size of Data = %d \n",send_ac_id, s);

  // make Ctrl-C stop the main loop and clean up properly
  signal(SIGINT, sigint_handler);

  open_port(argv[2]);

  // create timer (Ivy)
  tid = TimerRepeatAfter (0, delay, handle_timer, 0);


  IvyInit ("IVY <-> Serial", "IVY <-> Serial READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Attitude, NULL, "^(\\S*) ATTITUDE (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_Gps, NULL, "^(\\S*) GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
  
  IvyMainLoop ();

  return 0;
}


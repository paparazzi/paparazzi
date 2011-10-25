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

//////////////////////////////////////////////////////////////////////////////////
// local_uav DATA
//////////////////////////////////////////////////////////////////////////////////


struct _uav_type_
{
  // Header
  unsigned char header;

  // Data
  unsigned char ac_id;
  short int phi, theta, psi, speed;
  int utm_east,utm_north,utm_z;
  unsigned char utm_zone;
  unsigned char pprz_mode;
  float desired_alt;
  unsigned char block;

  // Footer
  unsigned char footer;
} 
__attribute__((packed))

local_uav, remote_uav;

volatile unsigned char new_ivy_data = 0;
volatile unsigned char new_serial_data = 0;

//////////////////////////////////////////////////////////////////////////////////
// IVY Reader
//////////////////////////////////////////////////////////////////////////////////


static void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
/*
   <message name="ATTITUDE" id="6">
     <field name="phi"   type="float" unit="rad" alt_unit="deg"/>
     <field name="psi"   type="float" unit="rad" alt_unit="deg"/>
     <field name="theta" type="float" unit="rad" alt_unit="deg"/>
   </message>
*/

  local_uav.phi   = (short int) (atof(argv[0]) * 1000.0);
  local_uav.psi   = (short int) (atof(argv[1]) * 1000.0);
  local_uav.theta = (short int) (atof(argv[2]) * 1000.0);

  //printf("ATTITUDE ac=%d phi=%d theta=%d psi=%d ",local_uav.ac_id, local_uav.phi, local_uav.theta, local_uav.psi);
}

static void on_Desired(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
/*
   <message name="DESIRED" id="16">
     <field name="roll" type="float" format="%.2f" unit="rad" alt_unit="deg" alt_unit_coef="57.3"/>
     <field name="pitch" type="float" format="%.2f" unit="rad" alt_unit="deg" alt_unit_coef="57.3"/>
     <field name="course" type="float" format="%.1f" unit="rad" alt_unit="deg" alt_unit_coef="57.3"/>
     <field name="x" type="float" format="%.0f"  unit="m"/>
     <field name="y" type="float" format="%.0f" unit="m"/>
     <field name="altitude" type="float" format="%.0f"  unit="m"/>
     <field name="climb" type="float" format="%.1f"  unit="m/s"></field>
   </message>
*/

  local_uav.desired_alt = atof(argv[5]);
}

static void on_Gps(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
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

  local_uav.utm_east = atoi(argv[1]);
  local_uav.utm_north = atoi(argv[2]);
  local_uav.utm_z = atoi(argv[4]);
  local_uav.utm_zone = atoi(argv[9]);
  local_uav.speed = atoi(argv[5]);

  //printf("ATTITUDE ac=%d phi=%d theta=%d psi=%d ",local_uav.ac_id, local_uav.phi, local_uav.theta, local_uav.psi);
  //printf("GPS ac=%d %d %d %d %d\n",local_uav.ac_id, local_uav.utm_east, local_uav.utm_north, local_uav.utm_z, local_uav.utm_zone);

  new_ivy_data = 1;
}

//////////////////////////////////////////////////////////////////////////////////
// IVY Writer
//////////////////////////////////////////////////////////////////////////////////

void send_ivy(void)
{
  float phi,theta,psi,z,zdot;

  if (new_serial_data == 0)
    return;
 
  new_serial_data = 0;

  phi = ((float) remote_uav.phi) / 1000.0f;
  theta = ((float) remote_uav.theta) / 1000.0f;
  psi = ((float) remote_uav.psi) / 1000.0f;

  IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n", remote_uav.ac_id);

  IvySendMsg("%d ATTITUDE %f %f %f\n", remote_uav.ac_id, phi, psi, theta);
  
/*
  remote_uav.utm_east = local_uav.utm_east;
  remote_uav.utm_north = local_uav.utm_north + 5000;
  remote_uav.utm_z = local_uav.utm_z + 1000;
  remote_uav.utm_zone = local_uav.utm_zone;
  remote_uav.speed = local_uav.speed * 4;
  remote_uav.psi += 30.;
*/

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

  IvySendMsg("%d GPS 3 %d %d 0 %d %d 0 0 0 %d 0\n", remote_uav.ac_id, remote_uav.utm_east, remote_uav.utm_north, remote_uav.utm_z, remote_uav.speed, remote_uav.utm_zone);
  
/*
  <message name="FBW_STATUS" id="103">
    <field name="rc_status" type="uint8" values="OK|LOST|REALLY_LOST"/>
    <field name="frame_rate" type="uint8" unit="Hz"/>
    <field name="mode" type="uint8" values="MANUAL|AUTO|FAILSAFE"/>
    <field name="vsupply" type="uint8" unit="decivolt"/>
    <field name="current" type="int32" unit="mA"/>
  </message>
*/

  IvySendMsg("%d FBW_STATUS 2 0 1 81 0 \n", remote_uav.ac_id);

  z = ((float)remote_uav.utm_z) / 1000.0f;
  zdot = 0.0f;
  IvySendMsg("%d ESTIMATOR %f %f \n", remote_uav.ac_id, z, zdot);

/*
   <message name="DESIRED" id="16">
     <field name="roll" type="float" format="%.2f" unit="rad" alt_unit="deg" alt_unit_coef="57.3"/>
     <field name="pitch" type="float" format="%.2f" unit="rad" alt_unit="deg" alt_unit_coef="57.3"/>
     <field name="course" type="float" format="%.1f" unit="rad" alt_unit="deg" alt_unit_coef="57.3"/>
     <field name="x" type="float" format="%.0f"  unit="m"/>
     <field name="y" type="float" format="%.0f" unit="m"/>
     <field name="altitude" type="float" format="%.0f"  unit="m"/>
     <field name="climb" type="float" format="%.1f"  unit="m/s"></field>
   </message>
*/
  IvySendMsg("%d DESIRED 0 0 0 0 0 %f 0 \n", remote_uav.ac_id, remote_uav.desired_alt);

  printf("IVY %d\n",remote_uav.ac_id);
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

unsigned char* buf_tx = (unsigned char*) &local_uav;
unsigned char* buf_rx = (unsigned char*) &remote_uav;

void send_port(void)
{
  int bytes;
  int i = 0;

  if (new_ivy_data == 0)
    return;
 
  new_ivy_data = 0;


  local_uav.header = '@';
  local_uav.footer = 0;
  // Checksum 
  for (i=0;i<(sizeof(local_uav)-1);i++)
  {
    local_uav.footer += buf_tx[i];
    printf("%x ", buf_tx[i]);
  }
  bytes = write(fd, &local_uav, sizeof(local_uav));
  printf("SENT: %d (%d bytes)\n",local_uav.ac_id, bytes);
}

void read_port(void)
{
  int i;
  static int counter = 0;
  int readsize = sizeof(remote_uav) - counter;
  int bytes = read(fd, buf_rx + counter, readsize);
  unsigned char crc = 0;

  // printf("READ: %d bytes",bytes);

  if (bytes <= 0)
    return;

  counter += bytes;

  if (counter >= sizeof(remote_uav)) 
  {
    if (buf_rx[0] != '@')
    {
      printf("Protocol Error\n");
    }
    for (i=0;i<(sizeof(remote_uav)-1);i++)
    {
      crc += buf_rx[i];
      printf("%x ", buf_rx[i]);
    }
    if (buf_rx[sizeof(remote_uav)-1] != crc)
    {
      printf("Checksum Error\n");
    }
    printf("RECEIVED %d (%d bytes)\n",remote_uav.ac_id, counter);
    counter -= sizeof(remote_uav);

    new_serial_data = 1;
    remote_uav.ac_id = 6;

    send_ivy();
  }
}

void close_port(void)
{
  close(fd);
}

//////////////////////////////////////////////////////////////////////////////////
// TIMER
//////////////////////////////////////////////////////////////////////////////////

// Timer
void handle_timer (TimerId id, void *data, unsigned long delta) 
{
  static unsigned char dispatch = 0;
  
  // Every Time
  read_port();

  // One out of 2
  if (dispatch > 0)
  {
    send_port();
    dispatch = 0;
  }
  else
  {
    dispatch ++;
  }
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
  
  local_uav.ac_id = atoi(argv[1]);

  printf("Listening to AC=%d, \nSending Size of Data = %d \n",local_uav.ac_id, s);

  // make Ctrl-C stop the main loop and clean up properly
  signal(SIGINT, sigint_handler);

  // Open Serial or Die
  open_port(argv[2]);

  // Init UAV
  remote_uav.ac_id = 6;

  remote_uav.phi = 1000;
  remote_uav.theta = 200;
  remote_uav.psi = -3140;


  // create timer (Ivy)
  tid = TimerRepeatAfter (0, delay / 2, handle_timer, 0);


  IvyInit ("IVY <-> Serial", "IVY <-> Serial READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Desired, NULL, "^%d DESIRED (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",local_uav.ac_id);
  IvyBindMsg(on_Attitude, NULL, "^%d ATTITUDE (\\S*) (\\S*) (\\S*)", local_uav.ac_id);
  IvyBindMsg(on_Gps, NULL, "^%d GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",local_uav.ac_id);
  IvyStart("127.255.255.255");
  
  IvyMainLoop ();

  return 0;
}


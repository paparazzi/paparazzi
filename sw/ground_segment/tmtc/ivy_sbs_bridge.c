#include <glib.h>
#include <gtk/gtk.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <Ivy/version.h>
//#include <Ivy/ivyglibloop.h>


#define Dprintf(X, ... )
//#define Dprintf printf

#define DEBUG_INPUT 0
#define DEBUG_INTR 0

#define MAX_INTRUDER 10
#define MAX_AGE_INTR 80   //Scaled by timer, stepsize 1/4s


///////////
//Mostly copied geometric liraries
/////////////

/* Computation for the WGS84 geoid only */
#define E 0.08181919106
#define K0 0.9996
#define DELTA_EAST  500000.
#define DELTA_NORTH 0.
#define A 6378137.0
#define N (K0*A)

#define LambdaOfUtmZone(utm_zone) RadOfDeg((utm_zone-1)*6-180+3)

static const float serie_coeff_proj_mercator[5] = {
  0.99832429842242842444,
  0.00083632803657738403,
  0.00000075957783563707,
  0.00000000119563131778,
  0.00000000000241079916
};

static const float serie_coeff_proj_mercator_inverse[5] = {
  0.998324298422428424,
  0.000837732168742475825,
  5.90586914811817062e-08,
  1.6734091890305064e-10,
  2.13883575853313883e-13
};

struct complex { float re; float im; };
#define CScal(k, z) { z.re *= k;  z.im *= k; }
#define CAdd(z1, z2) { z2.re += z1.re;  z2.im += z1.im; }
#define CSub(z1, z2) { z2.re -= z1.re;  z2.im -= z1.im; }
#define CI(z) { float tmp = z.re; z.re = - z.im; z.im = tmp; }
#define CExp(z) { float e = exp(z.re); z.re = e*cos(z.im); z.im = e*sin(z.im); }

#define CSin(z) { CI(z); struct complex _z = {-z.re, -z.im}; float e = exp(z.re); float cos_z_im = cos(z.im); z.re = e*cos_z_im; float sin_z_im = sin(z.im); z.im = e*sin_z_im; _z.re = cos_z_im/e; _z.im = -sin_z_im/e; CSub(_z, z); CScal(-0.5, z); CI(z); }


#if DEBUG_INPUT == 1
#define INPUT_PRINT(...) { printf( __VA_ARGS__);};
#else
#define INPUT_PRINT(...) {  };
#endif


#if DEBUG_INTR == 1
#define INTR_PRINT(...) { printf( __VA_ARGS__);};
#else
#define INTR_PRINT(...) {  };
#endif

#define INPUT_MAXLEN 255

void parse_msg4(void);
void parse_msg3(void);

void handle_intruders(void);

void sbs_parse_msg(void);
void sbs_parse_char(unsigned char c);

float DegOfRad(float i);
float RadOfDeg(float i);

void close_port(void);
int open_port(void);
void read_port(void);


struct MsgBuf {
  int msg_available;
  int pos_available;
  int nb_ovrn;        // number if incomplete messages
  char msg_buf[INPUT_MAXLEN];  ///< buffer for storing one line
  int msg_len;
};

struct MsgBuf in_data;

struct LlaCoor_f {
  float lon; ///< in radians
  float lat; ///< in radians
  float alt; ///< in meters above WGS84 reference ellipsoid
};

/**
 * @brief position in UTM coordinates
 * Units: meters */
struct UtmCoor_f {
  float north; ///< in meters
  float east; ///< in meters
  float alt; ///< in meters above WGS84 reference ellipsoid
  int zone; ///< UTM zone number
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 * @details Units lat,lon: radians*1e7
 * Unit alt: centimeters above MSL
 */
struct LlaCoor_i {
  int lon; ///< in radians*1e7
  int lat; ///< in radians*1e7
  int alt; ///< in millimeters above WGS84 reference ellipsoid
};

/**
 * @brief position in UTM coordinates
 */
struct UtmCoor_i {
  int north; ///< in centimeters
  int east; ///< in centimeters
  int alt; ///< in millimeters above WGS84 reference ellipsoid
  int zone; ///< UTM zone number
};

// data structure for intruders
struct Intruder {
  int id;
  struct UtmCoor_i utm_pos;
  int lastalt;
  int gspeed;
  int course;
  int climb;
  float dist;
  uint time4;
  uint time3;
  int used;
};

struct Intruder Intr[MAX_INTRUDER + 1];

//Flags
int sendivyflag = 0;

int num_intr;

uint timer = 100;
uint lastivyrcv = 0;
uint lastivytrx = 0;

int portstat = 0;

void utm_of_lla_f(struct UtmCoor_f *utm, struct LlaCoor_f *lla);
static inline float isometric_latitude_f(float phi, float e);
static inline float isometric_latitude_fast_f(float phi);

int dist(struct UtmCoor_i *utmi);


//TCP Port variables
int sockfd, portno;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[256];
fd_set readSet;
struct timeval selTimeout;


//////////////////////////////////////////////////////////////////////////////////
// SETTINGS
//////////////////////////////////////////////////////////////////////////////////

// Serial Repeat Rate
long delay = 1000;

GtkWidget *status_ivy;
GtkWidget *status_out_ivy;
GtkWidget *status;
GtkWidget *status_sbs;

char status_str[256];
char status_ivy_str[256];
char status_ivy_out[256];
char status_sbs_str[256];

long int count_ivy = 0;
long int count_serial = 0;

//////////////////////////////////////////////////////////////////////////////////
// local_uav DATA
//////////////////////////////////////////////////////////////////////////////////

struct _uav_type_ {
  // Header
  unsigned char header;

  // Data
  unsigned char ac_id;
  short int phi, theta, psi, speed;
  int utm_east, utm_north, utm_z;
  unsigned char utm_zone;
  unsigned char pprz_mode;
  float desired_alt;
  float climb;
  float course;
  unsigned char block;

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
  /*
     <message name="ATTITUDE" id="6">
       <field name="phi"   type="float" unit="rad" alt_unit="deg"/>
       <field name="psi"   type="float" unit="rad" alt_unit="deg"/>
       <field name="theta" type="float" unit="rad" alt_unit="deg"/>
     </message>
  */

  local_uav.phi   = (short int)(atof(argv[0]) * 1000.0);
  local_uav.psi   = (short int)(atof(argv[1]) * 1000.0);
  local_uav.theta = (short int)(atof(argv[2]) * 1000.0);

  //Dprintf("ATTITUDE ac=%d phi=%d theta=%d psi=%d ",local_uav.ac_id, local_uav.phi, local_uav.theta, local_uav.psi);
}

static void on_Estimator(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  /*
    <message name="ESTIMATOR" id="42">
      <field name="z" type="float" unit="m"/>
      <field name="z_dot" type="float" unit="m/s"/>
    </message>
  */

  local_uav.utm_z = ((atof(argv[0])) * 1000.0f);
  local_uav.climb   = atof(argv[1]);

}

static void on_Navigation(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  /*
     <message name="NAVIGATION" id="10">
       <field name="cur_block" type="uint8"/>
       <field name="cur_stage" type="uint8"/>
       <field name="pos_x" type="float" unit="m" format="%.1f"/>
       <field name="pos_y" type="float" unit="m" format="%.1f"/>
       <field name="dist2_wp" type="float" format="%.1f" unit="m^2"/>
       <field name="dist2_home" type="float" format="%.1f" unit="m^2"/>
       <field name="circle_count" type="uint8"/>
       <field name="oval_count" type="uint8"/>
     </message>
  */

  local_uav.block   = atoi(argv[0]);
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
  local_uav.utm_zone = atoi(argv[9]);
  local_uav.speed = atoi(argv[5]);

  count_ivy++;
  lastivyrcv = timer;

  //Print to window
  sprintf(status_ivy_str, "Received from IVY: [%ld]", count_ivy);
  gtk_label_set_text(GTK_LABEL(status_ivy), status_ivy_str);
}

//////////////////////////////////////////////////////////////////////////////////
// IVY Writer
//////////////////////////////////////////////////////////////////////////////////

void send_ivy(void)
{
  float phi, theta, psi, z, zdot;

  phi = ((float) remote_uav.phi) / 1000.0f;
  theta = ((float) remote_uav.theta) / 1000.0f;
  psi = (RadOfDeg(remote_uav.course));

  IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n", remote_uav.ac_id);

  IvySendMsg("%d ATTITUDE %f %f %f\n", remote_uav.ac_id, phi, psi, theta);


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

  IvySendMsg("%d GPS 3 %d %d %d %d %d %d 0 0 %d 0\n", remote_uav.ac_id, remote_uav.utm_east, remote_uav.utm_north,
             (int)remote_uav.course, remote_uav.utm_z, remote_uav.speed, (int)remote_uav.climb, remote_uav.utm_zone);

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
  zdot = remote_uav.climb / 100.0f;
  IvySendMsg("%d ESTIMATOR %f %f \n", remote_uav.ac_id, z, zdot);

  count_serial++;
  lastivytrx = timer;

  sprintf(status_ivy_out, "Intruder ID: %d; forwarding to IVY [%ld]", remote_uav.ac_id, count_serial);
  gtk_label_set_text(GTK_LABEL(status_out_ivy), status_ivy_out);
}



//////////////////////////////////////////////////////////////////////////////////
// TIMER
//////////////////////////////////////////////////////////////////////////////////

// Main functions called every 1/4s
gboolean timeout_callback(gpointer data)
{
  static unsigned char dispatch = 0;

  // Every Time
  if (portstat == 1) {
    read_port();
  }
  sendivyflag = 1;
  handle_intruders();
  timer++;

  // One out of 4
  if (dispatch > 2) {
    dispatch = 0;

    if ((timer - lastivyrcv) > 10) {
      sprintf(status_ivy_str, "--");
      gtk_label_set_text(GTK_LABEL(status_ivy), status_ivy_str);
    }
    if ((timer - lastivytrx) > 10) {
      sprintf(status_ivy_out, "--");
      gtk_label_set_text(GTK_LABEL(status_out_ivy), status_ivy_out);
    }
    if (portstat == 0) {
      portstat = open_port();
    }

  } else {
    dispatch ++;
  }
  return TRUE;
}

//////////////////////////////////////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////////////////////////////////////

gint delete_event(GtkWidget *widget,
                  GdkEvent  *event,
                  gpointer   data)
{
  g_print("CLEAN STOP\n");

  close_port();
  IvyStop();

  exit(0);

  return (FALSE); // false = delete window, FALSE = keep active
}


int main(int argc, char **argv)
{


  gtk_init(&argc, &argv);

  if (argc < 2) {
    printf("Use: ivy2serial ac_id \n");
    return -1;
  }


  local_uav.ac_id = atoi(argv[1]);

  sprintf(status_str, "Listening to AC=%d", local_uav.ac_id);
  sprintf(status_ivy_str, "--");
  sprintf(status_ivy_out, "--");
  printf("%s\n", status_str);

  // Try open port

  if (argc > 2) { //dedicated port
    if (argc < 3) {
      printf("Server and Port please\n");
    } else {
      server = gethostbyname(argv[2]);
      portno = atoi(argv[3]);
    }
  } else {
    printf("Standard localhost 30003 used!\n");
    server = gethostbyname("localhost");
    portno = 30003;
  }
  portstat = open_port();

  // Init UAV

  remote_uav.phi = 1000;
  remote_uav.theta = 200;
  remote_uav.psi = -3140;

  // Start IVY
  IvyInit("IVY <-> Serial", "IVY <-> Serial READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Desired, NULL, "^%d DESIRED (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", local_uav.ac_id);
  IvyBindMsg(on_Estimator, NULL, "^%d ESTIMATOR (\\S*) (\\S*)", local_uav.ac_id);
  IvyBindMsg(on_Navigation, NULL, "^%d NAVIGATION (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",
             local_uav.ac_id);
  IvyBindMsg(on_Attitude, NULL, "^%d ATTITUDE (\\S*) (\\S*) (\\S*)", local_uav.ac_id);
  IvyBindMsg(on_Gps, NULL, "^%d GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",
             local_uav.ac_id);
  IvyStart("127.255.255.255");

  // Add Timer
  gtk_timeout_add(delay / 4, timeout_callback, NULL);

  // GTK Window
  GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(window), "IVY_SBS_Bridge");

  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(delete_event), NULL);

  GtkWidget *box = gtk_vbox_new(TRUE, 1);
  gtk_container_add(GTK_CONTAINER(window), box);

  GtkWidget *hbox = gtk_hbox_new(FALSE, 1);
  gtk_container_add(GTK_CONTAINER(box), hbox);
  status = gtk_label_new("Status:");
  gtk_box_pack_start(GTK_BOX(hbox), status, FALSE, FALSE, 1);
  gtk_label_set_justify((GtkLabel *) status, GTK_JUSTIFY_LEFT);
  status = gtk_label_new(status_str);
  gtk_box_pack_start(GTK_BOX(hbox), status, FALSE, FALSE, 1);
  gtk_label_set_justify((GtkLabel *) status, GTK_JUSTIFY_LEFT);

  hbox = gtk_hbox_new(FALSE, 1);
  gtk_container_add(GTK_CONTAINER(box), hbox);
  status_ivy = gtk_label_new("From IVY:");
  gtk_box_pack_start(GTK_BOX(hbox), status_ivy, FALSE, FALSE, 1);
  gtk_label_set_justify((GtkLabel *) status_ivy, GTK_JUSTIFY_LEFT);
  status_ivy = gtk_label_new(status_ivy_str);
  gtk_box_pack_start(GTK_BOX(hbox), status_ivy, FALSE, FALSE, 1);
  gtk_label_set_justify((GtkLabel *) status_ivy, GTK_JUSTIFY_LEFT);

  hbox = gtk_hbox_new(FALSE, 1);
  gtk_container_add(GTK_CONTAINER(box), hbox);
  status_out_ivy = gtk_label_new("To IVY:");
  gtk_box_pack_start(GTK_BOX(hbox), status_out_ivy, FALSE, FALSE, 1);
  gtk_label_set_justify((GtkLabel *) status_out_ivy, GTK_JUSTIFY_LEFT);
  status_out_ivy = gtk_label_new(status_ivy_out);
  gtk_label_set_justify(GTK_LABEL(status_out_ivy), GTK_JUSTIFY_LEFT);
  gtk_box_pack_start(GTK_BOX(hbox), status_out_ivy, FALSE, FALSE, 1);

  hbox = gtk_hbox_new(FALSE, 1);
  gtk_container_add(GTK_CONTAINER(box), hbox);
  status_sbs = gtk_label_new("SBS:");
  gtk_box_pack_start(GTK_BOX(hbox), status_sbs, FALSE, FALSE, 1);
  gtk_label_set_justify((GtkLabel *) status_sbs, GTK_JUSTIFY_LEFT);
  status_sbs = gtk_label_new(status_sbs_str);
  gtk_label_set_justify(GTK_LABEL(status_sbs), GTK_JUSTIFY_LEFT);
  gtk_box_pack_start(GTK_BOX(hbox), status_sbs, FALSE, FALSE, 1);


  gtk_widget_show_all(window);

  gtk_main();

  // Clean up
  fprintf(stderr, "Stopping\n");

  return 0;
}

///
//Subroutines
/////

//////////////////////////////////////////////////////////////////////////////////
// TCP PORT
//////////////////////////////////////////////////////////////////////////////////

/// Open
int open_port(void)
{

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if ((sockfd < 0) && DEBUG_INPUT) {
    perror("ERROR opening socket");
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(portno);
  if ((connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) && DEBUG_INPUT) {
    perror("ERROR connecting socket");
    return 0;
  }

//set values for select()
  FD_ZERO(&readSet);
  FD_SET(sockfd, &readSet);//tcp socket

  selTimeout.tv_sec = 0;       /* timeout (secs.) */
  selTimeout.tv_usec = 10;     /* 10 microseconds */

  return 1;
}

///Close
void close_port(void)
{
  close(sockfd);
}

/////////////////////////////////////
//Read ADSB data from TCP port
////////////////////////////////////////////

void read_port(void)
{
  unsigned char readbuf;
  int sel_ret;

  FD_SET(sockfd, &readSet);//set tcp socket for select

  sel_ret = select(sockfd + 1, &readSet, NULL, NULL, &selTimeout);

  if (sel_ret == -1) {
    perror("select"); // error occurred in select()
  } else if (sel_ret == 0) {
    INPUT_PRINT(".");  //Debug for no message
    fflush(stdout);
  } else {
    FD_SET(sockfd, &readSet);//re-set
    while (select(sockfd + 1, &readSet, NULL, NULL, &selTimeout) > 0) {
      FD_SET(sockfd, &readSet);//re-set
      if (read(sockfd, &readbuf, 1) == 1) {
        sbs_parse_char(readbuf);
      } else { //port closed at server, go to reconnect
        portstat = 0;
        close_port();
        break;
      }
      if (in_data.msg_available) {
        sbs_parse_msg();
      }
    }
  }

}

/**
 * This is the actual parser.
 * It reads one character at a time
 * setting in_data.msg_available to TRUE
 * after a full line.
 */
void sbs_parse_char(unsigned char c)
{
  //reject empty lines
  if (in_data.msg_len == 0) {
    if (c == '\r' || c == '\n' || c == '$') {
      return;
    }
  }

  // fill the buffer, unless it's full
  if (in_data.msg_len < INPUT_MAXLEN - 1) {

    // messages end with a linefeed
    //AD: TRUNK:       if (c == '\r' || c == '\n')
    if (c == '\r' || c == '\n') {
      in_data.msg_available = 1;
    } else {
      in_data.msg_buf[in_data.msg_len] = c;
      in_data.msg_len ++;
      in_data.msg_available = 0;
    }
  }

  if (in_data.msg_len >= INPUT_MAXLEN - 1) {
    in_data.msg_available = 1;
  }
}


/**
 * sbs_parse_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
MSG,4,,,495224,,,,,,,,404,54,,,64,,0,0,0,0
MSG,1,,,495224,,,,,,TAP594  ,,,,,,,,0,0,0,0
MSG,8,,,495224,,,,,,,,,,,,,,,,,
MSG,3,,,495224,,,,,,,37000,,,48.27750,11.68840,,,0,0,0,0
 */
void sbs_parse_msg(void)
{

  if (in_data.msg_len > 5 && !strncmp(in_data.msg_buf , "MSG,4", 5)) {
    in_data.msg_buf[in_data.msg_len] = 0;
    INPUT_PRINT("parsing MSG 4: \"%s\" \n\r", in_data.msg_buf);
    //INPUT_PRINT("MSG 4 ");
    parse_msg4();
  } else {
    if (in_data.msg_len > 5 && !strncmp(in_data.msg_buf , "MSG,3", 5)) {
      in_data.msg_buf[in_data.msg_len] = 0;
      INPUT_PRINT("parsing MSG 3: \"%s\" \n\r", in_data.msg_buf);
      //INPUT_PRINT("MSG 3");
      parse_msg3();
    } else {
      in_data.msg_buf[in_data.msg_len] = 0;
      INPUT_PRINT("ignoring: len=%i \"%s\" \n\r", in_data.msg_len, in_data.msg_buf);
    } //ignore
  }//MSG3

  // reset message-buffer
  in_data.msg_len = 0;
}


/**
 * parse MSG 4 SBS Message
 */
void parse_msg4(void)
{
  int i = 8;     // current position in the message, start after: MSG,4,,,
  char *endptr;  // end of parsed substrings


//Reading Hex number
  in_data.msg_buf[i - 2] = '0';
  in_data.msg_buf[i - 1] = 'x';
  //get Flarm intruder ID
  int id = strtod(&in_data.msg_buf[i - 2], &endptr);
  INPUT_PRINT("MSG4 id = %i \n\r", id);

  while (in_data.msg_buf[i++] != ',') {             // next field
    if (i >= in_data.msg_len) {
      return;
    }
  }

  i += 7;           //forward to Groundspeed

//get intruder Ground speed (knots)
  int speed = strtod(&in_data.msg_buf[i], &endptr) * 51; //knots to cm/s (rough..)
  INPUT_PRINT("Speed = %i \n\r", speed);


  while (in_data.msg_buf[i++] != ',') {             // next field: Track
    if (i >= in_data.msg_len) {
      return;
    }
  }

//get intruder ground track
  int track = strtod(&in_data.msg_buf[i], &endptr);
  INPUT_PRINT("Track = %i \n\r", track);



  while (in_data.msg_buf[i++] != ',') {             // next field: empty
    if (i >= in_data.msg_len) {
      return;
    }
  }

  while (in_data.msg_buf[i++] != ',') {             // next field: empty
    if (i >= in_data.msg_len) {
      return;
    }
  }

  while (in_data.msg_buf[i++] != ',') {             // next field: Climb rate
    if (i >= in_data.msg_len) {
      return;
    }
  }

//get intruder climb rate (feet per minute)
  int climb = strtod(&in_data.msg_buf[i], &endptr) / 2; //fpm to cm/s


  INPUT_PRINT("Climb = %i \n\r", climb);

//Build table of current intruder situation

  //Check if already in list, then replace by new values
  int z ;
  int newflag = 1;

  for (z = 0; z < MAX_INTRUDER; z++) {
    if (Intr[z].id == id) {
      newflag = 0;
      //Check for positive or negative vertical speed (not signed :-( )
      if (Intr[z].utm_pos.alt < Intr[z].lastalt) {
        //sinking
        climb = -climb;
      } else if (Intr[z].utm_pos.alt == Intr[z].lastalt) {
        //equal.. take last
        if (Intr[z].climb < 0) {
          climb = -climb;
        }
      }

      Intr[z].gspeed         = speed;
      Intr[z].course         = track;
      Intr[z].climb          = climb;
      Intr[z].time4          = timer;
    }

  }

  //New intruder
  if (newflag) {
    Intr[MAX_INTRUDER].id             = id;
    Intr[MAX_INTRUDER].gspeed         = speed;
    Intr[MAX_INTRUDER].course         = track;
    Intr[MAX_INTRUDER].climb          = climb;
    Intr[MAX_INTRUDER].time4          = timer;
    Intr[MAX_INTRUDER].used           = 4;
    INPUT_PRINT("new = %i \n\r", Intr[MAX_INTRUDER].id);

  }

  handle_intruders();

  return;
}


/**
 * parse MSG 3 SBS Message
 */
void parse_msg3(void)
{

  int i = 8;     // current position in the message, start after: MSG,4,,,
  char *endptr;  // end of parsed substrings
  struct LlaCoor_f coordf;
  struct UtmCoor_f utmf;
  struct UtmCoor_i utmi;

//Reading Hex number
  in_data.msg_buf[i - 2] = '0';
  in_data.msg_buf[i - 1] = 'x';
  //get Flarm intruder ID
  int id = strtod(&in_data.msg_buf[i - 2], &endptr);
  INPUT_PRINT("MSG3 id = %i \n\r", id);

  while (in_data.msg_buf[i++] != ',') {             // next field
    if (i >= in_data.msg_len) {
      return;
    }
  }

  i += 6;           //forward to Alt

//get intruder alt (in feet)
  int alt = strtod(&in_data.msg_buf[i], &endptr) * 305; //feet to mm
  INPUT_PRINT("Alt = %i \n\r", alt);

  while (in_data.msg_buf[i++] != ',') {             // next field: empty
    if (i >= in_data.msg_len) {
      return;
    }
  }

  while (in_data.msg_buf[i++] != ',') {             // next field: empty
    if (i >= in_data.msg_len) {
      return;
    }
  }

  while (in_data.msg_buf[i++] != ',') {             // next field: Lat
    if (i >= in_data.msg_len) {
      return;
    }
  }

  //Get intruder Lat
  double lat = strtod(&in_data.msg_buf[i], &endptr);
  INPUT_PRINT("Lat = %f \n\r", lat);

// convert to radians
  coordf.lat = RadOfDeg(lat);


  while (in_data.msg_buf[i++] != ',') {             // next field: Lon
    if (i >= in_data.msg_len) {
      return;
    }
  }

  //Get intruder Lon
  double lon = strtod(&in_data.msg_buf[i], &endptr);
  INPUT_PRINT("Lon = %f \n\r", lon);

// convert to radians
  coordf.lon = RadOfDeg(lon);


  coordf.alt = alt / 1000;
  //Calculations for UTM zone of local UAV so that distance calc works even for zone borders
  utmf.zone = local_uav.utm_zone;
  utm_of_lla_f(&utmf, &coordf);

  /* copy results of utm conversion */

  utmi.east = utmf.east * 100;
  utmi.north = utmf.north * 100;
  utmi.alt = alt;
  utmi.zone = utmf.zone;

  INPUT_PRINT("UTMI E N Z = %d, %d, %d, %d \n\r", utmi.east, utmi.north, utmi.zone,  dist(&utmi));

//Build table of current intruder situation

//Check if already in list, then replace by new values
  int z ;
  int newflag = 1;

  for (z = 0; z < MAX_INTRUDER; z++) {
    if (Intr[z].id == id) {
      newflag = 0;
      Intr[z].lastalt        = Intr[z].utm_pos.alt;
      Intr[z].utm_pos.north  = utmi.north ;
      Intr[z].utm_pos.east   = utmi.east ;
      Intr[z].utm_pos.alt    = utmi.alt ;
      Intr[z].utm_pos.zone   = utmi.zone;
      Intr[z].dist           = dist(&utmi);
      Intr[z].time3          = timer;
    }

  }

  //New intruder
  //If Relative East is empty (value 0), it is a mode C/S transponder, so no position.
  if (newflag) {
    Intr[MAX_INTRUDER].id             = id;
    Intr[MAX_INTRUDER].utm_pos.north  = utmi.north ;
    Intr[MAX_INTRUDER].utm_pos.east   = utmi.east ;
    Intr[MAX_INTRUDER].utm_pos.alt    = utmi.alt ;
    Intr[MAX_INTRUDER].utm_pos.zone   = utmi.zone;
    Intr[MAX_INTRUDER].dist           = dist(&utmi);
    Intr[MAX_INTRUDER].time3          = timer;
    Intr[MAX_INTRUDER].used           = 3;
    INPUT_PRINT("new = %i \n\r", Intr[MAX_INTRUDER].id);

  }

  handle_intruders();
  return;
}

//Build up of intruder table
void handle_intruders(void)
{

  int z;

  //Analyse if Data ok
  num_intr = 0;
  for (z = 0; z < MAX_INTRUDER; z++) {
    //If data too old, mark unused
    if (((timer - Intr[z].time3) < MAX_AGE_INTR) && ((timer - Intr[z].time4) < MAX_AGE_INTR)) {
      Intr[z].used = 1 ;
      num_intr++; //count valid planes
    } else if ((timer - Intr[z].time3) < MAX_AGE_INTR) {
      Intr[z].used = 3 ;
    } else if ((timer - Intr[z].time4) < MAX_AGE_INTR) {
      Intr[z].used = 4 ;
    } else {
      Intr[z].used = 0;
    }
  }

  INTR_PRINT("unsort %i", num_intr);
  for (z = 0; z < MAX_INTRUDER + 1; z++) {
    INTR_PRINT("Nr:%i d:%.2f u:%d", z, Intr[z].dist, Intr[z].used);
  }
  INTR_PRINT("fin \n\r");

  //Sort for dist and used
  int zi;
  struct Intruder Temp_int;
  //Bubble sort
  for (z = 0; z < MAX_INTRUDER + 1; z++) {
    for (zi = 0; zi < MAX_INTRUDER + 1; zi++) {
      if (((Intr[zi].dist > Intr[zi + 1].dist) && (Intr[zi + 1].used == 1)) || (Intr[zi].used == 0)) {
        Temp_int = Intr[zi];
        Intr[zi] = Intr[zi + 1];
        Intr[zi + 1] = Temp_int;
      }
    }
  }

  if (num_intr) {
    remote_uav.ac_id = 99;
    remote_uav.phi = 0;
    remote_uav.utm_east = Intr[0].utm_pos.east;
    remote_uav.utm_north = Intr[0].utm_pos.north;
    remote_uav.utm_z = Intr[0].utm_pos.alt;
    remote_uav.speed = Intr[0].gspeed;
    remote_uav.utm_zone = Intr[0].utm_pos.zone;
    remote_uav.climb = Intr[0].climb;
    remote_uav.course = Intr[0].course;

    //Send to IVY if data valid
    if (sendivyflag == 1) {
      if (Intr[0].used) {
        sendivyflag = 0;
        send_ivy();
      }
    }

  }

  //show on window
  float dist_close = 0.0;
  if (Intr[0].used == 1) {
    dist_close = Intr[0].dist / 100000.0;
  }

  if (portstat == 1) {
    sprintf(status_sbs_str, "Connected; Intruders: %i  Min.Dist: %4.1f km", num_intr, dist_close);
    gtk_label_set_text(GTK_LABEL(status_sbs), status_sbs_str);
  } else {
    sprintf(status_sbs_str, "No SBS Data, Port closed");
    gtk_label_set_text(GTK_LABEL(status_sbs), status_sbs_str);
  }
  return;
}

//calculate distance intruder to local UAV
int dist(struct UtmCoor_i *utmi)
{

  int d = sqrtf((float)(utmi->north - local_uav.utm_north) * (float)(utmi->north - local_uav.utm_north) + (float)(
                  utmi->east - local_uav.utm_east) * (float)(utmi->east - local_uav.utm_east) + ((float)(
                        utmi->alt - local_uav.utm_z) / 10.0) * ((float)(utmi->alt - local_uav.utm_z) / 10.0));;

  return d;
}

float RadOfDeg(float i)
{
  float erg;
  erg =  i / 180 * M_PI ;
  return erg;
}

float DegOfRad(float i)
{
  float erg;
  erg =  i / M_PI * 180 ;
  return erg;
}

void utm_of_lla_f(struct UtmCoor_f *utm, struct LlaCoor_f *lla)
{
  float lambda_c = LambdaOfUtmZone(utm->zone);
  float ll = isometric_latitude_f(lla->lat , E);
  float dl = lla->lon - lambda_c;
  float phi_ = asin(sin(dl) / cosh(ll));
  float ll_ = isometric_latitude_fast_f(phi_);
  float lambda_ = atan(sinh(ll) / cos(dl));
  struct complex z_ = { lambda_,  ll_ };
  CScal(serie_coeff_proj_mercator[0], z_);
  int k;
  for (k = 1; k < 3; k++) {
    struct complex z = { lambda_,  ll_ };
    CScal(2 * k, z);
    CSin(z);
    CScal(serie_coeff_proj_mercator[k], z);
    CAdd(z, z_);
  }
  CScal(N, z_);
  utm->east = DELTA_EAST + z_.im;
  utm->north = DELTA_NORTH + z_.re;

  // copy alt above reference ellipsoid
  utm->alt = lla->alt;
}

static inline float isometric_latitude_f(float phi, float e)
{
  return log(tan(M_PI_4 + phi / 2.0)) - e / 2.0 * log((1.0 + e * sin(phi)) / (1.0 - e * sin(phi)));
}

static inline float isometric_latitude_fast_f(float phi)
{
  return log(tan(M_PI_4 + phi / 2.0));
}

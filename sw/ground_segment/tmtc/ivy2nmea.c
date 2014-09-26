/*
 * Name: IVY2NMEA
 * Author: OpenUAS (Thanks to CDW for basis and Tim for NMEA library work)
 * URL: http://ivy2nmea.openuas.org
 * License: http://www.gnu.org/licenses/lgpl.html
 * Info: IVY AC messages to NMEA GPRMC and GPGGA sentences out to serial port but clearly for *Position only*
 * (OBC2014 RULES: an NMEA 0183 serial output with GPRMC and GPGGA sentences for aircraft current location)
   Absolute Location = position?
   Location - Position Definition: A point on the earth's surface expressed by a coordinate system such as latitude and longitude.

 * Goal: This application listens on the ivy bus extracts data needed and outputs NMEA sentences on a serial port
 * Why: An UAS challenge OBC 2014 demands NMEA serial data out, the sole reason for this application
 * Version: 0.1
 * Date: 20131029
 * Notes: Alpha state
 * $Id:  $
*/

/* NMEA Introduction

NMEA 0183 is a combined electrical and data specification for communication
between marine electronic devices such as echo sounder, sonars, anemometer,
gyrocompass, autopilot, GPS receivers and many other types of instruments.

NMEA 0183 standard uses text-based (ASCII), serial communications protocol.
It defines rules for transmitting "sentences" from one "talker" to
multiple listeners.

About NMEA 0183 protocols

There are two layers in NMEA 0183:

# data link layer
# application layer protocol

In fact, data link layer defines only serial configuration:

* bit rate (typically 4800)
* 8 data bits
* no parity checking
* 1 stop bit
* no handshake

Application layer more complex, but not really to complex. Common NMEA sentence format listed below:

 $<talker ID><sentence ID,>[parameter 1],[parameter 2],...[<*checksum>]<CR><LF>

That else it is necessary to specify:

NMEA defines two types of sentences: proprietary and non-proprietary.
Non-proprietary sentences has a one of standard two-letter talker ID
(e.g. GP for GPS unit, GL for glonass unit etc.)
and one of standard three-letter sentence ID
(e.g. GLL for geographic location GPS data, DBK - depth below keel and so on)
all of this talker IDs and sentence IDs can be found in official paper.
Non-proprietary sentences has a 'P' letter instead of standard talker ID,
followed by three-letter standard manufacturer code
(GRM for Garmin, MTK for MTK etc.),
further follows any string - name of proprietary command
(depends on specific manufacturer). Maximum length for sentences– 82 characters.

Explanation via an example for (standard) 'GLL' GPS sentence:

GLL - means geographic location

 $GPGLL,1111.11,a,yyyyy.yy,a,hhmmss.ss, A*hh <CR><LF>

Parameters list description:

* llll.ll - latitude
* 'N' letter for North, 'S' - for south
* yyyyy.yy - longitude
* 'E' letter - for east, 'W' - for west
* UTC time in moment of measurement
* 'A' letter - data valid, 'V' - data not valid
* checksum

Example: $GPGLL,5532.8492,N,03729.0987,E,004241.469,A*33

and proprietary Garmin 'E' sentence:

PGRME - means Estimated Error Information

 $PGRME,x.x,M,x.x,M,x.x,M*hh <CR><LF>

Parameters list description:

* x.x - Estimated horizontal position error (HPE) 0.0 to 999.9 meters
* M - means meters
* x.x - Estimated vertical error (VPE) 0.0 to 999.9 meters
* M - means meters
* x.x - Estimated position error (EPE) 0.0 to 999.9 meters
* checksum

Types

For the OBC2014 we only need to provide two types: GPGGA and GPRMC

 $GPGGA  Global positioning system fixed data
 $GPRMC  Recommended minimum specific GNSS data

!!!GPRMC

The $GPRMC Sentence (Position and time)

* Example (signal not acquired):
  $GPRMC,235947.000,V,0000.0000,N,00000.0000,E,,,041299,,*1D
* Example (signal acquired):
  $GPRMC,092204.999,A,4250.5589,S,14718.5084,E,0.00,89.68,211200,,*25

!!!GPGGA

The $GPGGA Sentence (Fix data)

* Example (signal not acquired): $GPGGA,235947.000,0000.0000,N,00000.0000,E,0,00,0.0,0.0,M,,,,0000*00
* Example (signal acquired): $GPGGA,092204.999,4250.5589,S,14718.5084,E,1,04,24.4,19.7,M,,,,0000*1F

*/

#include <glib.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <float.h>
#include <time.h>

#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <Ivy/version.h>



/*
 * Application version
 */

#define IVY2NMEA_VERSION        ("0.1.0")
#define IVY2NMEA_VERSION_MAJOR  (0)
#define IVY2NMEA_VERSION_MINOR  (1)
#define IVY2NMEA_VERSION_MICRO  (0)

void write_port(char* buff, int len);

/*
 * Buffer
 */

#define NMEA_CONVSTR_BUF    (256)
#define NMEA_TIMEPARSE_BUF  (256)

#define NMEA_POSIX(x)  x
#define NMEA_INLINE    inline

/*
 * Distance units
 */

#define NMEA_MS_TO_KNOTS      (3.6 / 1.852)         /**< Knots, kilometer / NMEA_TUD_KNOTS = knot */

/*
 * Fixed for conversion
 */

#define NMEA_PI                     (3.141592653589793)             /**< PI value */
#define NMEA_PI180                  (NMEA_PI / 180)                 /**< PI division by 180 */
#define NMEA_EARTHRADIUS_KM         (6378)                          /**< Earth's mean radius in km */
#define NMEA_EARTHRADIUS_M          (NMEA_EARTHRADIUS_KM * 1000)    /**< Earth's mean radius in m */
#define NMEA_EARTH_SEMIMAJORAXIS_M  (6378137.0)                     /**< Earth's semi-major axis in m according WGS84 */
#define NMEA_EARTH_SEMIMAJORAXIS_KM (NMEA_EARTHMAJORAXIS_KM / 1000) /**< Earth's semi-major axis in km according WGS 84 */
#define NMEA_EARTH_FLATTENING       (1 / 298.257223563)             /**< Earth's flattening according WGS 84 */
#define NMEA_DOP_FACTOR             (5)                             /**< Factor for translating DOP to meters */

/*
 * Speed units
 */

#define NMEA_TUS_MS         (3.6)           /**< Meters per seconds, (k/h) / NMEA_TUS_MS= (m/s) */

/**
 * NMEA packets type which are generated by library
 */

enum nmeaPACKETTYPE
{
    GPNON   = 0x0000,   /**< Unknown packet type. */
    GPGGA   = 0x0001,   /**< GGA - Essential fix data which provide 3D location and accuracy data. */
    GPRMC   = 0x0002,   /**< RMC - Recommended Minimum Specific GPS/TRANSIT Data. */
};

/**
 * Date and time data
 */

typedef struct _nmeaDATETIME
{
  int     year;       /**< Years since 1900 */
  int     mon;        /**< Months since January - [0,11] */
  int     day;        /**< Day of the month - [1,31] */
  int     hour;       /**< Hours since midnight - [0,23] */
  int     min;        /**< Minutes after the hour - [0,59] */
  int     sec;        /**< Seconds after the minute - [0,59] */
  int     hsec;       /**< Hundredth part of second - [0,99] */

} nmeaDATETIME;

/**
 * GGA packet information structure (Global Positioning System Fix Data)
 */

typedef struct _nmeaGPGGA
{
  nmeaDATETIME utc;    /**< UTC of position (just time) */
  double   lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
  char     ns;         /**< [N]orth or [S]outh */
  double   lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
  char     ew;         /**< [E]ast or [W]est */
  int      sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
  int      satinuse;   /**< Number of satellites in use (not those in view) */
  double   HDOP;       /**< Horizontal dilution of precision */
  double   elv;        /**< Antenna altitude above/below mean sea level (geoid) */
  char     elv_units;  /**< [M]eters (Antenna height unit) */
  double   diff;       /**< Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level. '-' = geoid is below WGS-84 ellipsoid) */
  char     diff_units; /**< [M]eters (Units of geoidal separation) */
} nmeaGPGGA;

/**
 * RMC packet information structure (Recommended Minimum sentence C)
 */

typedef struct _nmeaGPRMC
{
  nmeaDATETIME utc;     /**< UTC of position */
  char     status;      /**< Status (A = active or V = void) */
  double   lat;         /**< Latitude in NDEG - [degree][min].[sec/60] */
  char     ns;          /**< [N]orth or [S]outh */
  double   lon;         /**< Longitude in NDEG - [degree][min].[sec/60] */
  char     ew;          /**< [E]ast or [W]est */
  double   speed;       /**< Speed over the ground in knots */
  double   direction;   /**< Track angle in degrees True */
} nmeaGPRMC;

/**
 * nmeaINFO_of_UAV_from_UA DATA
 */

typedef struct _nmeaINFO_of_UAV //Where data is stored to be saved via NMA out
{
  unsigned char ac_id; /**< the Aircraft ID of the data */
  double  lat;         /**< Latitude in deg */
  double  lon;         /**< Longitude in deg */
  double  elv;         /**< Antenna altitude above/below mean sea level (geoid) in meters */
  double  speed;       /**< Speed over the ground in kilometers/hour */
  double  direction;   /**< Track angle in degrees True */
  double  pitch;       /**< degrees */
  double  roll;
  double  heading;
  double  agl;          /**< Altitude above ground [m] */
} nmeaINFO_of_UAV;

/*************************************************************************/


/**
 * \fn nmea_degree2radian
 * \brief Convert degree to radian
 */

double nmea_degree2radian(double val) { return (val * NMEA_PI180); }

/**
 * \fn nmea_radian2degree
 * \brief Convert radian to degree
 */

double nmea_radian2degree(double val) { return (val / NMEA_PI180); }

/**
 * \brief Convert NDEG (NMEA degree) to fractional degree
 */

double nmea_ndeg2degree(double val)
{
  double deg = ((int)(val / 100));
  val = deg + (val - deg * 100) / 60;
  return val;
}

/**
 * \brief Convert fractional degree to NDEG (NMEA degree)
 */

double nmea_degree2ndeg(double val)
{
  double int_part;
  double fra_part;
  fra_part = modf(val, &int_part);
  val = int_part * 100 + fra_part * 60;
  return val;
}

/**
 * \fn nmea_ndeg2radian
 * \brief Convert NDEG (NMEA degree) to radian
 */

double nmea_ndeg2radian(double val) { return nmea_degree2radian(nmea_ndeg2degree(val)); }

/**
 * \fn nmea_radian2ndeg
 * \brief Convert radian to NDEG (NMEA degree)
 */

double nmea_radian2ndeg(double val) { return nmea_degree2ndeg(nmea_radian2degree(val)); }

/**
 * \brief Calculate control sum of binary buffer
 */

int nmea_calc_crc(const char *buff, int buff_sz)
{
  int chsum = 0,
      it;

  for(it = 0; it < buff_sz; ++it)
    chsum ^= (int)buff[it];

  return chsum;
}

/**
 * \brief Formating string (like standart printf) with CRC tail (*CRC)
 */

int nmea_printf(char *buff, int buff_sz, const char *format, ...)
{
  int retval, add = 0;
  va_list arg_ptr;

  if(buff_sz <= 0)
    return 0;

  va_start(arg_ptr, format);

  retval = NMEA_POSIX(vsnprintf)(buff, buff_sz, format, arg_ptr);

  if(retval > 0)
  {
    add = NMEA_POSIX(snprintf)(
        buff + retval, buff_sz - retval, "*%02x\r\n",
        nmea_calc_crc(buff + 1, retval - 1));
  }

  retval += add;

  if(retval < 0 || retval > buff_sz)
  {
    memset(buff, ' ', buff_sz);
    retval = buff_sz;
  }

  va_end(arg_ptr);

  return retval;
}

/**
 * \brief Get time now to nmeaTIME structure {TODO: determine UAtime or GCS time}
 */

void nmea_time_now(nmeaDATETIME *stm)
{
  time_t lt;
  struct tm *tt;

  time(&lt);
  tt = gmtime(&lt);

  stm->year = tt->tm_year;
  stm->mon = tt->tm_mon;
  stm->day = tt->tm_mday;
  stm->hour = tt->tm_hour;
  stm->min = tt->tm_min;
  stm->sec = tt->tm_sec;
  stm->hsec = 0;
}

/**
 * Clear GPGGA data
 */

void nmea_zero_GPGGA(nmeaGPGGA *pack)
{
  memset(pack, 0, sizeof(nmeaGPGGA));
  nmea_time_now(&pack->utc);
  pack->ns = 'N';
  pack->ew = 'E';
  pack->elv_units = 'M';
  pack->diff_units = 'M';
}

/**
 * Clear GPRMC data
 */

void nmea_zero_GPRMC(nmeaGPRMC *pack)
{
  memset(pack, 0, sizeof(nmeaGPRMC));
  nmea_time_now(&pack->utc);
  pack->status = 'V'; // Void
  pack->ns = 'N';
  pack->ew = 'E';
}

/**
 * \brief Generate NMEA GPGGA line
 */

int nmea_gen_GPGGA(char *buff, int buff_sz, nmeaGPGGA *pack)
{
  return nmea_printf(buff, buff_sz,
      "$GPGGA,%02d%02d%02d,%010.5f,%C,%011.5f,%C,%1d,%02d,%03.1f,%03.1f,%C,,,,,",
      pack->utc.hour, pack->utc.min, pack->utc.sec,
      pack->lat, pack->ns, pack->lon, pack->ew,
      pack->sig, pack->satinuse, pack->HDOP, pack->elv, pack->elv_units);
}

/**
 * \brief Generate NMEA GPRMC line
 */
int nmea_gen_GPRMC(char *buff, int buff_sz, nmeaGPRMC *pack)
{
  return nmea_printf(buff, buff_sz,
      "$GPRMC,%02d%02d%02d.%02d,%C,%010.5f,%C,%011.5f,%C,%03.1f,%03.1f,%02d%02d%02d,,,",
      pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
      pack->status, pack->lat, pack->ns, pack->lon, pack->ew,
      pack->speed, pack->direction,
      pack->utc.day, pack->utc.mon + 1, pack->utc.year - 100);
}

/**
 * \brief Generate from UAV GPGGA line
 */
void nmea_info2GPGGA(const nmeaINFO_of_UAV *info, nmeaGPGGA *pack)
{
  nmea_zero_GPGGA(pack);

  pack->lat = nmea_degree2ndeg(fabs(info->lat));
  pack->ns = ((info->lat > 0)?'N':'S');
  pack->lon = nmea_degree2ndeg(fabs(info->lon));
  pack->ew = ((info->lon > 0)?'E':'W');
  pack->sig = 1;
  pack->satinuse = 7;
  pack->HDOP = 1.0;
  pack->elv = info->elv;
}

/**
 * \brief Generate from UAV GPRMC line
 */
void nmea_info2GPRMC(const nmeaINFO_of_UAV *info, nmeaGPRMC *pack)
{
  nmea_zero_GPRMC(pack);

  pack->status = 'A'; // Active
  pack->lat = nmea_degree2ndeg(fabs(info->lat));
  pack->ns = ((info->lat > 0)?'N':'S');
  pack->lon = nmea_degree2ndeg(fabs(info->lon));
  pack->ew = ((info->lon > 0)?'E':'W');
  pack->speed = info->speed * NMEA_MS_TO_KNOTS;
  pack->direction = info->direction;
}



gboolean verbose;
int watchdog = 0;

/**
 * \brief IVY Read and store in UAV info structure
 */

static void on_Gps(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  // Reset Watchdog
  watchdog = 0;

  if (argc != 13)
  {
    fprintf(stderr,"ERROR: IVY2NMEA: invalid message length FLIGHT_PARAM\n");
  }

/*
  <message name="FLIGHT_PARAM" id="11">
    <field name="ac_id"  type="string"/>
    <field 0 name="roll"   type="float" unit="deg"/>
    <field 1 name="pitch"  type="float" unit="deg"/>
    <field 2 name="heading" type="float" unit="deg"/>
    <field 3 name="lat"    type="float" unit="deg"/>
    <field 4 name="long"   type="float" unit="deg"/>
    <field 5 name="speed"  type="float" unit="m/s"/>
    <field 6 name="course" type="float" unit="deg" format="%.1f"/>
    <field 7 name="alt"    type="float" unit="m"/>
    <field 8 name="climb"  type="float" unit="m/s"/>
    <field 9 name="agl"    type="float" unit="m"/>
    <field 10 name="unix_time"    type="float" unit="s (Unix time)"/>
    <field 11 name="itow"   type="uint32" unit="ms"/>
    <field 12 name="airspeed" type="float" unit="m/s"/>
  </message>
*/

  nmeaINFO_of_UAV info;
  info.roll = atof(argv[0]);  // deg
  info.pitch = atof(argv[1]);  // deg
  info.heading = atof(argv[2]);  // deg
  info.lat = atof(argv[3]);
  info.lon = atof(argv[4]);
  info.speed = atof(argv[5]);
  info.direction = atof(argv[6]);
  info.elv = atof(argv[7]);
  info.agl = atof(argv[9]);

  nmeaGPGGA gga;
  nmea_info2GPGGA(&info, &gga);
  char buff[1024];
  int len = nmea_gen_GPGGA(buff,1024, &gga);
  write_port(buff,len);
  if (verbose)
    printf("%d: %s",len, buff);



  nmeaGPRMC rmc;
  nmea_info2GPRMC(&info, &rmc);
  len = nmea_gen_GPRMC(buff,1024, &rmc);
  write_port(buff, len);
  if (verbose)
    printf("%d: %s",len, buff);
}



/**
 * \brief Periodic
 */

#define TIMEOUT_PERIOD 200

static gboolean gps_periodic(gpointer data __attribute__ ((unused)))
{
  watchdog++;
  if (watchdog == 7) {
    fprintf(stderr,"Error: ivy2nmea FLIGHT_INFO timeout\n");
  }
  else if (watchdog > 7) {
    watchdog = 7;
  }
  return TRUE;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Serial Port
//

/**
 * \brief Set Serial Port
 */

int fd = 0;

/**
 * Open port
 */

void open_port(const char* device)
{
  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "ERROR: IVY2NMEA: open_port: unable to open device %s - ", device);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  /* setup connection options */
  struct termios options;

  /* get the current options */
  tcgetattr(fd, &options);

  /*
   * set local mode, enable receiver, set comm. options:
   * NMEA Defines, where bit rate (typically 4800)
   * 8 data bits, 1 stop bit, no parity, 4800 Baud, no handshake
   */

  options.c_cflag = CLOCAL | CREAD | CS8 | B4800;

  /* write options back to port */
  tcsetattr(fd, TCSANOW, &options);
}

/**
 * Write To Port
 */

void write_port(char* buff, int len)
{
  write(fd, buff,len);
}

/**
 * Close Serial Port
 */

void close_port(void)
{
  close(fd);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Main & Parse
//

char* ac_id;
char* port;
char* ivy_bus;

gboolean parse_args(int argc, char** argv)
{
  verbose = FALSE;
  ac_id = "1";
  port = "/dev/ttyS0";
#ifdef __APPLE__
  ivy_bus = "224.255.255.255";
#else
  ivy_bus = "127.255.255.255";
#endif

  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information\n"
    "   --id <ac_id>                           e.g. 1\n"
    "   --port <gps out port>                  e.g. /dev/ttyS0\n"
    "   --ivy_bus <ivy bus>                    e.g. 127.255.255.255\n";

  while (1) {

    static struct option long_options[] = {
      {"ivy_bus", 1, NULL, 0},
      {"id", 1, NULL, 0},
      {"port", 1, NULL, 0},
      {"help", 0, NULL, 'h'},
      {"verbose", 0, NULL, 'v'},
      {0, 0, 0, 0}
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "vh",
        long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
      case 0:
        switch (option_index) {
          case 0:
            ivy_bus = strdup(optarg); break;
          case 1:
            ac_id = strdup(optarg); break;
          case 2:
            port = strdup(optarg); break;
          default:
            break;
        }
        break;

      case 'v':
        verbose = TRUE;
        break;

      case 'h':
        fprintf(stderr, usage, argv[0]);
        exit(0);

      default: /* ’?’ */
        printf("?? getopt returned character code 0%o ??\n", c);
        fprintf(stderr, usage, argv[0]);
        exit(EXIT_FAILURE);
    }
  }
  return TRUE;
}



/**
 * \brief Main
 */

int main ( int argc, char** argv)
{
  if (!parse_args(argc, argv))
    return 1;

  printf("IVY2NMEA v%s\n",IVY2NMEA_VERSION);
  printf("Listening to AC=%s, NMEA0184 out on = %s\n",ac_id, port);

  open_port(port);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  // Start IVY
  IvyInit ("IVY2NMEA", "IVY2NMEA READY", NULL, NULL, NULL, NULL);
  //IvyBindMsg(on_Gps, NULL, "^%s GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)",ac_id);
  IvyBindMsg(on_Gps, NULL, "^ground FLIGHT_PARAM %s (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", ac_id);
  //IvyBindMsg(on_Gps, NULL, "(.*)");
  IvyStart(ivy_bus);

  g_timeout_add(TIMEOUT_PERIOD, gps_periodic, NULL);

  g_main_loop_run(ml);

  return 0;
}

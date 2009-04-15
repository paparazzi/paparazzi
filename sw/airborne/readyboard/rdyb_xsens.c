#include "rdyb_xsens.h"

#include "rdyb_debug.h"
#include "rdyb_telemetry.h"

#include <stdlib.h>
#include <inttypes.h>

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

void xsens_scale(struct RdybXsens* me);

#define XSENS_DEVICE "/dev/xsens"

#include "xsens_protocol.h"

static void configure_term(struct termios *termios, speed_t *speed);

struct RdybXsens*  xsens_new(void) {

  struct RdybXsens* me = malloc(sizeof(struct RdybXsens));

  me->sp = serial_port_new();
  
  if (serial_port_open(me->sp, XSENS_DEVICE, configure_term)) {
    xsens_free(me);
    return NULL;
  }

 return me; 

}

void xsens_free(struct RdybXsens* me) {
  if (me->sp) {
    serial_port_close(me->sp);
    serial_port_free(me->sp);
  }
  free(me);
}


#define WAIT_START 0
#define WAIT_BID   1
#define WAIT_MID   2
#define WAIT_LEN   3
#define WAIT_DATA  4
#define WAIT_CK    5

static unsigned char parser_status = WAIT_START;
static unsigned int  parser_error_nb = 0;
static unsigned char parser_ck;
static unsigned char parser_msg_id;
static unsigned char parser_msg_len;
static unsigned char parser_msg_idx;
static unsigned char parser_msg_buf[255];
static unsigned char parser_msg_available = FALSE;

static inline void priv_parser(unsigned char c);


// WARNING :  xsens is outputing sensors in East North Up
// we change that to North East Down
static const struct Int32Vect3 accel_neutral = { 32862, 33105, 32412};
static const struct FloatVect3 accel_scale_fact   = { 413, -413, -412};

static const struct Int32Vect3 gyro_neutral = { 32561, 32721, 32531};
static const struct FloatVect3 gyro_scale_fact   = { 1100, -1082, -1110};

static const struct Int32Vect3 mag_neutral = { 33312, 32713, 32310};
static const struct FloatVect3 mag_scale_fact   = { 7156, -6870, -7118};





void xsens_scale(struct RdybXsens* me) {

  struct Int32Vect3 accel_signed;
  Vect3Diff(accel_signed, me->accel_raw, accel_neutral);
  Vect3EwDiv(me->accel, accel_signed, accel_scale_fact);

  struct Int32Vect3 gyro_signed;
  Vect3Diff(gyro_signed, me->gyro_raw, gyro_neutral);
  Vect3EwDiv(me->gyro, gyro_signed, gyro_scale_fact);

  struct Int32Vect3 mag_signed;
  Vect3Diff(mag_signed, me->mag_raw, mag_neutral);
  Vect3EwDiv(me->mag, mag_signed, mag_scale_fact);

}


void xsens_parse(struct RdybXsens* me, int nb_bytes, char* buf, void(msg_cb)(void)) {

  //  TRACE(TRACE_DEBUG, "got %d bytes\n", nb_bytes);
  int i = 0;
  while (i<nb_bytes) {
    priv_parser(buf[i]);
    if (parser_msg_available) {
      //      TRACE(TRACE_DEBUG, "got message %d (%d %d)\n", 
      //      	    parser_msg_id, parser_msg_len, parser_error_nb);
      if (parser_msg_id == XSENS_MTData_ID) {
	me->accel_raw.x = XSENS_DATA_RAWInertial_accX(parser_msg_buf, 0); 
	me->accel_raw.y = XSENS_DATA_RAWInertial_accY(parser_msg_buf, 0); 
	me->accel_raw.z = XSENS_DATA_RAWInertial_accZ(parser_msg_buf, 0); 
	me->gyro_raw.x  = XSENS_DATA_RAWInertial_gyrX(parser_msg_buf, 0);
	me->gyro_raw.y  = XSENS_DATA_RAWInertial_gyrY(parser_msg_buf, 0);
	me->gyro_raw.z  = XSENS_DATA_RAWInertial_gyrZ(parser_msg_buf, 0);
	me->mag_raw.x   = XSENS_DATA_RAWInertial_magX(parser_msg_buf, 0);
	me->mag_raw.y   = XSENS_DATA_RAWInertial_magY(parser_msg_buf, 0);
	me->mag_raw.z   = XSENS_DATA_RAWInertial_magZ(parser_msg_buf, 0);
	//	uint16_t temp = *(uint16_t*)(parser_msg_buf+18);
	xsens_scale(me);
	msg_cb();
      }
      parser_msg_available = FALSE;
    }
    i++;
  }
}



static inline void priv_parser(unsigned char c) {

  //  TRACE(TRACE_DEBUG, "parsing char %d (%d)\n", c, parser_status);
  parser_ck += c;
  switch (parser_status) {
  case WAIT_START:
    // Wait for start of packet
    if (c!= XSENS_START)
      goto restart;
    parser_status++;
    parser_ck = 0;
    break;
  case WAIT_BID:
    // Look for xsens Bus ID
    if (c != XSENS_BID)
      goto error;
    parser_status++;
    break;
  case WAIT_MID:
    // Save message ID
    parser_msg_id = c;
    parser_status++;
    break;
  case WAIT_LEN:
    // Save message length
    parser_msg_len = c;   
    parser_msg_idx = 0;
    parser_status++;
    break;
  case WAIT_DATA:
    // Read byte into data buffer
    parser_msg_buf[parser_msg_idx] = c;
    parser_msg_idx++;
    if (parser_msg_idx >= parser_msg_len)
      parser_status++;
    break;
  case WAIT_CK:
    if (parser_ck) 
      goto error;
    // Notification for valid message received
    parser_msg_available = TRUE;
    goto restart;
    break;
  }
  return;
 error:
  parser_error_nb++;
 restart:
  parser_status = WAIT_START;
  return;

}




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
  *speed = B230400;
}




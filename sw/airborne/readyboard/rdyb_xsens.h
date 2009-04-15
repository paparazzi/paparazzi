#ifndef RDYB_XSENS_H
#define RDYB_XSENS_H


#include "fms_serial_port.h"

#include "rdyb_algebra.h"


struct RdybXsens {
  struct FmsSerialPort* sp;
  struct Int32Vect3 accel_raw;
  struct Int32Vect3 gyro_raw;
  struct Int32Vect3 mag_raw;
  struct FloatVect3 accel;
  struct FloatVect3 gyro;
  struct FloatVect3 mag;
};

extern struct RdybXsens*  xsens_new(void);
extern void xsens_free(struct RdybXsens* me);
extern void xsens_parse(struct RdybXsens* me, int nb_bytes, char* buf, void(msg_cb)(void));


#endif /* RDYB_XSENS_H */


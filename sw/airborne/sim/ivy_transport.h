#include <stdio.h>
#include <Ivy/ivy.h>

extern char ivy_buf[];
extern char* ivy_p;

#define IvyTransportCheckFreeSpace(_) TRUE

#define IvyTransportSizeOf(x) (x)

#define IvyTransportHeader(len) ivy_p=ivy_buf;

#define IvyTransportTrailer() { *ivy_p = '\0'; IvySendMsg("%s",ivy_buf); }

#define IvyTransportPutUint8(x) { ivy_p += sprintf(ivy_p, "%u ", x); }
#define IvyTransportPutNamedUint8(_name, _x) { ivy_p += sprintf(ivy_p, "%s ", _name); }

#define Space() ivy_p += sprintf(ivy_p, " ");
#define Comma() ivy_p += sprintf(ivy_p, ",");

#define IvyTransportPutUintByAddr(x) ivy_p += sprintf(ivy_p, "%u", *x);
#define IvyTransportPutUint8ByAddr(x) IvyTransportPutUintByAddr(x) Space()
#define IvyTransportPutUint16ByAddr(x) IvyTransportPutUintByAddr(x) Space()
#define IvyTransportPutUint32ByAddr(x) IvyTransportPutUintByAddr(x) Space()

#define IvyTransportPutIntByAddr(x) ivy_p += sprintf(ivy_p, "%d", *x);
#define IvyTransportPutInt8ByAddr(x) IvyTransportPutIntByAddr(x) Space()
#define IvyTransportPutInt16ByAddr(x) IvyTransportPutIntByAddr(x) Space()
#define IvyTransportPutInt32ByAddr(x) IvyTransportPutIntByAddr(x) Space()

#define IvyTransportPutOneFloatByAddr(x) ivy_p += sprintf(ivy_p, "%f", *x);
#define IvyTransportPutFloatByAddr(x) IvyTransportPutOneFloatByAddr(x) Space()
#define IvyTransportPutDoubleByAddr(x) IvyTransportPutOneFloatByAddr(x) Space()

#define IvyTransportPutArray(_put, _n, _x) { \
  int __i; \
  for(__i = 0; __i < _n; __i++) { \
    _put(&_x[__i]); \
    Comma(); \
  } \
}

#define IvyTransportPutUint8Array(_n, _x) IvyTransportPutArray(IvyTransportPutUintByAddr, _n, _x)
#define IvyTransportPutInt16Array(_n, _x) IvyTransportPutArray(IvyTransportPutIntByAddr, _n, _x)
#define IvyTransportPutUint16Array(_n, _x) IvyTransportPutArray(IvyTransportPutUintByAddr, _n, _x)
#define IvyTransportPutUint32Array(_n, _x) IvyTransportPutArray(IvyTransportPutUintByAddr, _n, _x)
#define IvyTransportPutFloatArray(_n, _x) IvyTransportPutArray(IvyTransportPutOneFloatByAddr, _n, _x)
#define IvyTransportPutDoubleArray(_n, _x) IvyTransportPutFloatArray(_n, _x) 

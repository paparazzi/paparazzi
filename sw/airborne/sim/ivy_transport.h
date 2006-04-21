#include <stdio.h>
#include <ivy.h>

extern char ivy_buf[];
extern char* ivy_p;

#define IvyTransportCheckFreeSpace(_) TRUE

#define IvyTransportSizeOf(x) (x)

#define IvyTransportHeader(len) ivy_p=ivy_buf;

#define IvyTransportTrailer() { *ivy_p = '\0'; IvySendMsg(ivy_buf); }

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
#define IvyTransportPutFloatByAddr(x) ivy_p += sprintf(ivy_p, "%f ", *x);

#define IvyTransportPutArray(_put, _n, _x) { \
  int i; \
  for(i = 0; i < _n; i++) { \
    _put(&_x[i]); \
    Comma(); \
  } \
}

#define IvyTransportPutInt16Array(_n, _x) IvyTransportPutArray(IvyTransportPutIntByAddr, _n, _x)
#define IvyTransportPutUint16Array(_n, _x) IvyTransportPutArray(IvyTransportPutUintByAddr, _n, _x)

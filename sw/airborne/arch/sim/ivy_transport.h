#include <stdio.h>
#include <Ivy/ivy.h>

extern char ivy_buf[];
extern char* ivy_p;

#define IvyTransportCheckFreeSpace(_dev,_) TRUE

#define IvyTransportSizeOf(_dev, x) (x)

#define IvyTransportHeader(_dev,len) ivy_p=ivy_buf;

#define IvyTransportTrailer(_dev) { *ivy_p = '\0'; IvySendMsg("%s",ivy_buf); }

#define IvyTransportPutUint8(_dev,x) { ivy_p += sprintf(ivy_p, "%u ", x); }
#define IvyTransportPutNamedUint8(_dev,_name, _x) { ivy_p += sprintf(ivy_p, "%s ", _name); }

#define Space() ivy_p += sprintf(ivy_p, " ");
#define Comma() ivy_p += sprintf(ivy_p, ",");

#define IvyTransportPutUintByAddr(_dev,x) ivy_p += sprintf(ivy_p, "%u", *x);
#define IvyTransportPutUint8ByAddr(_dev,x) IvyTransportPutUintByAddr(_dev,x) Space()
#define IvyTransportPutUint16ByAddr(_dev,x) IvyTransportPutUintByAddr(_dev,x) Space()
#define IvyTransportPutUint32ByAddr(_dev,x) IvyTransportPutUintByAddr(_dev,x) Space()

#define IvyTransportPutIntByAddr(_dev,x) ivy_p += sprintf(ivy_p, "%d", *x);
#define IvyTransportPutInt8ByAddr(_dev,x) IvyTransportPutIntByAddr(_dev,x) Space()
#define IvyTransportPutInt16ByAddr(_dev,x) IvyTransportPutIntByAddr(_dev,x) Space()
#define IvyTransportPutInt32ByAddr(_dev,x) IvyTransportPutIntByAddr(_dev,x) Space()

#define IvyTransportPutOneFloatByAddr(_dev,x) ivy_p += sprintf(ivy_p, "%f", *x);
#define IvyTransportPutFloatByAddr(_dev,x) IvyTransportPutOneFloatByAddr(_dev,x) Space()
#define IvyTransportPutDoubleByAddr(_dev,x) IvyTransportPutOneFloatByAddr(_dev,x) Space()

#define IvyTransportPutArray(_dev,_put, _n, _x) { \
  int __i; \
  for(__i = 0; __i < _n; __i++) { \
    _put(_dev,&_x[__i]); \
    Comma(); \
  } \
}

#define IvyTransportPutUint8Array(_dev,_n, _x) IvyTransportPutArray(_dev,IvyTransportPutUintByAddr, _n, _x)
#define IvyTransportPutInt16Array(_dev,_n, _x) IvyTransportPutArray(_dev,IvyTransportPutIntByAddr, _n, _x)
#define IvyTransportPutUint16Array(_dev,_n, _x) IvyTransportPutArray(_dev,IvyTransportPutUintByAddr, _n, _x)
#define IvyTransportPutUint32Array(_dev,_n, _x) IvyTransportPutArray(_dev,IvyTransportPutUintByAddr, _n, _x)
#define IvyTransportPutFloatArray(_dev,_n, _x) IvyTransportPutArray(_dev,IvyTransportPutOneFloatByAddr, _n, _x)
#define IvyTransportPutDoubleArray(_dev,_n, _x) IvyTransportPutFloatArray(_dev,_n, _x)

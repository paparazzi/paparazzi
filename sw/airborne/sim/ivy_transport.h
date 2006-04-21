#include <stdio.h>
#include <ivy.h>

extern char ivy_buf[];
extern char* ivy_p;

#define IvyTransportCheckFreeSpace(_) TRUE

#define IvyTransportSizeOf(x) (x)

#define IvyTransportHeader(len) ivy_p=ivy_buf;

#define IvyTransportTrailer() { *ivy_p = '\0'; IvySendMsg(ivy_buf); }

#define IvyTransportPutUint8(x) { ivy_p += sprintf(ivy_p, "%u ", x); }

#define IvyTransportPutUint8ByAddr(x) { ivy_p += sprintf(ivy_p, "%u ", *x); }
#define IvyTransportPutUint16ByAddr(x) { ivy_p += sprintf(ivy_p, "%u ", *x); }
#define IvyTransportPutUint32ByAddr(x) { ivy_p += sprintf(ivy_p, "%u ", *x);}

#define IvyTransportPutInt8ByAddr(x) { ivy_p += sprintf(ivy_p, "%d ", *x); }
#define IvyTransportPutInt16ByAddr(x) { ivy_p += sprintf(ivy_p, "%d ", *x); }
#define IvyTransportPutInt32ByAddr(x) { ivy_p += sprintf(ivy_p, "%d ", *x); }
#define IvyTransportPutFloatByAddr(x) { ivy_p += sprintf(ivy_p, "%f ", *x); }

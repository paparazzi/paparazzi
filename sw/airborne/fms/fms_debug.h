#ifndef FMS_DEBUG_H
#define FMS_DEBUG_H

#include <stdio.h>

#define TRACE_DEBUG 1
#define TRACE_ERROR 2

#define TRACE(type,fmt,args...) {               \
    fprintf(stderr, fmt, args);                 \
  }


#endif /* FMS_DEBUG_H */

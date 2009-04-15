#ifndef RDBY_DEBUG_H
#define RDBY_DEBUG_H


#include <stdio.h>

#define TRACE(type,fmt,args...) { printf(fmt, args);}

#define TRACE_DEBUG 1
#define TRACE_ERROR 2



#endif /* RDBY_DEBUG_H */

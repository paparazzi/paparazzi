#ifndef BOOZ_TRIG_INT_H
#define BOOZ_TRIG_INT_H

#include "std.h"
#include "airframe.h"

/* Allow makefile to define BOOZ_TRIG_CONST in case we want
 to make the trig tables const and store them in flash.
 Otherwise use the empty string and keep the table in RAM. */
#ifndef BOOZ_TRIG_CONST
#define BOOZ_TRIG_CONST
#endif

extern BOOZ_TRIG_CONST int16_t booz_trig_int[];

#endif /* BOOZ_TRIG_INT_H */

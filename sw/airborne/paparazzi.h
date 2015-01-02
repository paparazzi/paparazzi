#ifndef PAPARAZZI_H
#define PAPARAZZI_H

#include <inttypes.h>

typedef int16_t pprz_t; /* type of commands */

#define MAX_PPRZ 9600
#define MIN_PPRZ -MAX_PPRZ

#define TRIM_PPRZ(pprz) (pprz <  MIN_PPRZ ? MIN_PPRZ :  \
                         (pprz >  MAX_PPRZ ? MAX_PPRZ : \
                          pprz))
#define TRIM_UPPRZ(pprz) (pprz <  0 ? 0 :  \
                          (pprz >  MAX_PPRZ ? MAX_PPRZ : \
                           pprz))

#if defined FBW && defined AP
#define SINGLE_MCU 1
#endif

#endif /* PAPARAZZI_H */

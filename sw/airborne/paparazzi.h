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

/** PPRZ types of vehicles.
 *  Sent with ALIVE message
 */
#define PPRZ_TYPE_UNDEFINED   0
#define PPRZ_TYPE_GCS         1
#define PPRZ_TYPE_FIXEDWING   2
#define PPRZ_TYPE_ROTORCRAFT  3
#define PPRZ_TYPE_HYBRID      4
#define PPRZ_TYPE_ROVER       5
#define PPRZ_TYPE_SUBMARIN    6
#define PPRZ_TYPE_ROCKET      7
#define PPRZ_TYPE_SPACECRAFT  8 // this one is just in case :)

/** Frame NED or ENU
 */
#define FRAME_NED 0
#define FRAME_ENU 1

/** Single MCU flag.
 *  True if AP and FBW are both defined
 */
#if defined FBW && defined AP
#define SINGLE_MCU 1
#endif

#endif /* PAPARAZZI_H */

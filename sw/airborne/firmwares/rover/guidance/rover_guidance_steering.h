/*  rover_guidance_steering.h */

#ifndef ROVER_GUIDANCE_STEERING_H
#define ROVER_GUIDANCE_STEERING_H

/** Generated airframe.h from airframe.xml
 * - fun: SetActuatorsFromCommands
 * - var: commands
 * - var: 
 **/

#include "generated/airframe.h"

#include "math/pprz_algebra_float.h"
#include "std.h"

extern void rover_guidance_steering_init(void);
extern void rover_guidance_steering_periodic(void);


// helper macro to set AP throttle value
#define SetAPThrottleFromCommands(_cmd_x, _cmd_y) { \
    autopilot.throttle = sqrtf(((_cmd_x * _cmd_x) + (_cmd_y * _cmd_y)) / 2.f); \
  }

#endif // ROVER_GUIDANCE_STEERING_H

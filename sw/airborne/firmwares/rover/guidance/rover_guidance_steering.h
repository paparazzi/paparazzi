/*  rover_guidance_steering.h */

#ifndef ROVER_GUIDANCE_STEERING_H
#define ROVER_GUIDANCE_STEERING_H

/** Generated airframe.h from airframe.xml
 * - fun: SetActuatorsFromCommands
 * - var: commands
 * - var: hardware and construction parameters
 **/

#include "std.h"
#include <math.h>

#include "generated/airframe.h"

// Check critical global definitiones
#ifndef SERVO_MOTOR_THROTTLE
#error "Steering rover firmware requires the servo MOTOR_THROTTLE"
#endif

#ifndef SERVO_MOTOR_STEERING
#error "Steering rover firmware requires the servo MOTOR_STEERING"
#endif

#ifndef COMMAND_THROTTLE
#error "Steering rover firmware requires the command COMMAND_THROTTLE"
#endif

#ifndef COMMAND_STEERING
#error "Steering rover firmware requires the command COMMAND_STEERING"
#endif


/** Global variables definitions **/

// MIN_DELTA, MAX_DELTA: Min and max wheels turning angle (deg)
#ifndef MAX_DELTA 
#define MAX_DELTA 90.0
#endif
#ifndef MIN_DELTA 
#define MIN_DELTA MAX_DELTA
#endif

#ifndef MAX_CMD_SHUT
#define MAX_CMD_SHUT 0
#endif
#ifndef MIN_CMD_SHUT 
#define MIN_CMD_SHUT 0
#endif

// MIN_SPEED, MAX_SPEED: Min and max state speed (m/s)
#ifndef MAX_SPEED 
#define MAX_SPEED 999.0
#endif
#ifndef MIN_SPEED 
#define MIN_SPEED 0.2
#endif

// NAV max throttle
#define MAX_THROTTLE 100000.0

// DRIVE_SHAFT_DISTANCE: Distance between front and rear wheels (m)
#ifndef DRIVE_SHAFT_DISTANCE
#define DRIVE_SHAFT_DISTANCE 0.25
#warning "Construction variable DRIVE_SHAFT_DISTANCE for steer wheels rover not defined"
#endif


/** Steering rover guidance STRUCTURES **/
// High commands
typedef struct {
  float speed;
  float delta;
} RScmd_t;

// Main structure
typedef struct {
  RScmd_t cmd;
  float state_speed;
  float gvf_omega;
  float throttle;
} rover_ctrl;

extern rover_ctrl guidance_control;

/** Steering rover guidance EXT FUNCTIONS **/
extern void rover_guidance_steering_init(void);
extern void rover_guidance_steering_periodic(void);
extern bool rover_guidance_steering_set_delta(float delta);


/** MACROS **/
// Bound delta
#define BoundDelta(delta) (delta < -MIN_DELTA ? -MIN_DELTA : \
                          (delta >  MAX_DELTA ?  MAX_DELTA : \
                           delta));

// Bound speed
#define BoundSpeed(speed) (speed <  MIN_SPEED ? MIN_SPEED : \
                          (speed >  MAX_SPEED ? MAX_SPEED : \
                           speed));

// Bound throttle
#define BoundThrottle(throttle) (throttle < - MAX_THROTTLE ? - MAX_THROTTLE : \
                                (throttle >   MAX_THROTTLE ?   MAX_THROTTLE : \
                                 throttle));

/* Set low commands from high commands */
#define GetCmdFromDelta(delta) (delta >= 0 ? -delta/MAX_DELTA * (MAX_PPRZ - (int)MAX_CMD_SHUT) : \
                                             -delta/MIN_DELTA * (MAX_PPRZ - (int)MIN_CMD_SHUT));

#define GetCmdFromThrottle(throttle) TRIM_PPRZ((int)throttle / MAX_THROTTLE * MAX_PPRZ);
/* .. */

// Set AP throttle value
#define SetAPThrottleFromCommands(void) { \
    autopilot.throttle = commands[COMMAND_THROTTLE]; \
  }

#endif // ROVER_GUIDANCE_STEERING_H

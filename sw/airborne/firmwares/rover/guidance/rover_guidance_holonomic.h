/*
 * Copyright (C) 2018 Fabien Bonneval <fabien.bonneval@gmail.com>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/rover/guidance/rover_guidance_holonomic.h
 *  Basic guidance for rover.
 *  Implement standard PID control loop to track a navigation target.
 *  Guidance "modes" are using the autopilot generation with the "guidance"
 *  state machine.
 */

#ifndef ROVER_GUIDANCE_HOLONOMIC_H
#define ROVER_GUIDANCE_HOLONOMIC_H

#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "std.h"

struct RoverHoloGuidanceSetpoint {
  /** position setpoint in NED.
   */
  struct FloatVect2 pos;    ///< position setpoint
  struct FloatVect2 speed;  ///< speed setpoint
  float heading;            ///< heading setpoint
  uint8_t mask;             ///< bit 5: vx & vy, bit 6: vz, bit 7: vyaw
};

// TODO write a generic PID somewhere else ?
struct RoverHoloGuidancePID {
  float p;
  float i;
  float d;
  float err;
  float d_err;
  float sum_err;
};


struct RoverHoloGuidanceControl {
  float motor_speed_x;
  float motor_speed_y;
  float motor_turn;
};

struct RoverHoloGuidance {
  struct RoverHoloGuidanceControl cmd;    ///< commands
  struct RoverHoloGuidanceSetpoint sp;    ///< setpoints
  struct RoverHoloGuidancePID speed_pid;  ///< motor speed controller
  struct RoverHoloGuidancePID turn_pid;   ///< turn rate controller
};

extern struct RoverHoloGuidance rover_holo_guidance;

extern void rover_holo_guidance_init(void);
extern void rover_holo_guidance_periodic(void); // call state machine
extern void rover_holo_guidance_run(float *heading_sp); // run control loop
extern void rover_holo_guidance_enter(void);

// settings handler
extern void rover_guidance_holonomic_set_speed_igain(float igain);
extern void rover_guidance_holonomic_set_turn_igain(float igain);

// helper macro to set AP throttle value
#define SetAPThrottleFromCommands(_cmd_x, _cmd_y) { \
    autopilot.throttle = sqrtf(((_cmd_x * _cmd_x) + (_cmd_y * _cmd_y)) / 2.f); \
  }

#endif /* ROVER_GUIDANCE_HOLONOMIC_H */

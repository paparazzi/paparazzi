/** \file sim_ir.c
 *  \brief Regroup functions to simulate autopilot/infrared.c
 *
 * Infrared soft simulation. OCaml binding.
 */


#include <inttypes.h>
#include "subsystems/sensors/infrared.h"
#include "generated/airframe.h"

#include <caml/mlvalues.h>

float imu_roll_neutral = RadOfDeg(IMU_ROLL_NEUTRAL_DEFAULT);
float imu_pitch_neutral = RadOfDeg(IMU_PITCH_NEUTRAL_DEFAULT);

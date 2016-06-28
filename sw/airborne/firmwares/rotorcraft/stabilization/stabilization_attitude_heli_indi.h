/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef STABILIZATION_ATTITUDE_HELI_INDI_H
#define STABILIZATION_ATTITUDE_HELI_INDI_H

#include "math/pprz_algebra_int.h"

#include "filters/notch_filter.h"
#include "filters/delayed_first_order_lowpass_filter.h"

#define __k 1
#define INDI_NR_FILTERS 2
#define INDI_DOF 4
#define INDI_ROLL 0
#define INDI_PITCH 1
#define INDI_YAW 2
#define INDI_THRUST 3
#define INDI_YAW_BUFFER_SIZE 9

struct HeliIndiGains {
  int32_t roll_p;
  int32_t pitch_p;
  int32_t yaw_p;
  int32_t yaw_d;
};

/* All these values are in the struct to make it easier for logging */
struct IndiController_int {
  int32_t reference[INDI_DOF];                                      ///< Range -MAX_PPRZ:MAX_PPRZ
  int32_t error[INDI_DOF];                                          ///< virtual control minus measurement
  int32_t invG[INDI_DOF][INDI_DOF];                                 ///< Inverse control effectiveness matrix
  int32_t du[INDI_DOF];                                             ///< Actuator commanded increment
  int32_t u_setpoint[INDI_DOF];                                     ///< Actuator setpoint without compensator
  int32_t actuator_out[INDI_DOF];                                   ///< Actuator position
  int32_t command_out[2][INDI_DOF];                                 ///< Command and command from previous measurement
  int32_t filtered_actuator[INDI_NR_FILTERS][INDI_DOF];             ///< Filtered actuator position
  int32_t measurement[INDI_DOF];                                    ///< Raw measurement
  int32_t filtered_measurement[INDI_NR_FILTERS][INDI_DOF];          ///< Filtered measurement
  int32_t pitch_comp_angle;                                         ///< Angle to rotate pitch/roll commands with INT32_ANGLE_FRAC
  int32_t roll_comp_angle;                                          ///< Angle to rotate pitch/roll commands with INT32_ANGLE_FRAC
  bool enable_notch;                                                ///< Use notch filters
  int16_t motor_rpm;                                                ///< RPM of the main motor
  float sp_offset_roll;                                             ///< Neutral roll angle [deg]
  float sp_offset_pitch;                                            ///< Neutral pitch angle [deg]
  void (*apply_compensator_filters)(int32_t _out[], int32_t _in[]);
  void (*apply_actuator_models)(int32_t _out[], int32_t _in[]);
  void (*apply_actuator_filters[INDI_NR_FILTERS])(int32_t _out[], int32_t _in[]);
  void (*apply_measurement_filters[INDI_NR_FILTERS])(int32_t _out[], int32_t _in[]);
};

//extern struct IndiController_int heli_indi_ctl; // keep private

extern struct delayed_first_order_lowpass_filter_t actuator_model[INDI_DOF];
extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern struct HeliIndiGains heli_indi_gains;

extern void stabilization_attitude_heli_indi_set_steadystate_pitch(float pitch);
extern void stabilization_attitude_heli_indi_set_steadystate_roll(float roll);
extern void stabilization_attitude_heli_indi_set_steadystate_pitchroll(void);

#endif /* STABILIZATION_ATTITUDE_HELI_INDI_H */

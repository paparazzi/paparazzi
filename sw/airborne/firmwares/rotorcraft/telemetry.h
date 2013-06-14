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

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "mcu_periph/uart.h"

#include "subsystems/datalink/downlink.h"
#include "generated/periodic_telemetry.h"

// FIXME no idea where to put this one
//#ifndef AHRS_FLOAT
//#define PERIODIC_SEND_ROTORCRAFT_TUNE_HOVER(DefaultChannel, DefaultDevice) {
//  DOWNLINK_SEND_ROTORCRAFT_TUNE_HOVER(DefaultChannel, DefaultDevice,
//      &radio_control.values[RADIO_ROLL],
//      &radio_control.values[RADIO_PITCH],
//      &radio_control.values[RADIO_YAW],
//      &stabilization_cmd[COMMAND_ROLL],
//      &stabilization_cmd[COMMAND_PITCH],
//      &stabilization_cmd[COMMAND_YAW],
//      &stabilization_cmd[COMMAND_THRUST],
//      &ahrs_impl.ltp_to_imu_euler.phi,
//      &ahrs_impl.ltp_to_imu_euler.theta,
//      &ahrs_impl.ltp_to_imu_euler.psi,
//      &(stateGetNedToBodyEulers_i()->phi),
//      &(stateGetNedToBodyEulers_i()->theta),
//      &(stateGetNedToBodyEulers_i()->psi));
//}
//#endif


#endif /* TELEMETRY_H */

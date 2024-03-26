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

/** @file firmwares/rotorcraft/stabilization/stabilization_attitude.h
 *  General attitude stabilization interface for rotorcrafts.
 *  The actual implementation is automatically included.
 */

#ifndef STABILIZATION_ATTITUDE_H
#define STABILIZATION_ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "firmwares/rotorcraft/stabilization.h"
#include "modules/radio_control/radio_control.h"

/** Stabilization init function
 *
 * needs to be implemented by the selected controller
 */
extern void stabilization_attitude_init(void);

/** Retrun attitude setpoint from RC as euler angles
 *
 * weak function that can be re-implemeted if needed
 *
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 * @return stabilization setpoint
 */
extern struct StabilizationSetpoint stabilization_attitude_read_rc(bool in_flight, bool carefree, bool coordinated_turn, struct RadioControl *rc);

/** Attitude control enter function
 */
extern void stabilization_attitude_enter(void);

/** Attitude control run function
 *
 * @param[in] in_flight         true if in flight
 * @param[in] sp                pointer to the stabilization setpoint structure
 * @param[in] thrust            pointer to the thrust setoint structure
 * @param[out] cmd              pointer to the output command vector
 */
extern void stabilization_attitude_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* STABILIZATION_ATTITUDE_H */

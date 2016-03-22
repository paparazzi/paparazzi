/*
 * Copyright (C) 2015 Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * This file is part of paparazzi:
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
/**
 * @file /paparazzi/paparazzi/sw/airborne/subsystems/ins/ins_module_int.h
 * @author Torbjoern Cunis
 *
 * State filter in a module file.
 *
 * This allows to implement an individual state filter within a paparazzi module, rather than in
 * the inertia sub-system. Thus, different approaches can be realized and tested in a modular way.
 * If the define #INS_USE_MODULE is set in the airframe file, the corresponding init, reset,
 * propagation, and update functions need to be implemented in the module and will be called by the
 * state filter subsystem.
 *
 * One must implement:
 *
 * - void ins_module_int_init (void)
 * - void ins_module_int_reset_local_origin (void)
 * - void ins_module_int_propagate (struct Int32Vect3* accel_meas_ltp, float dt)
 *
 * #if USE_BARO_BOARD
 * - void ins_module_int_update_baro (float pressure)
 * #endif
 *
 * #if USE_GPS
 * - void ins_module_int_update_gps (struct NedCoor_i* gps_pos_ned, struct NedCoor_i* gps_speed_ned, float dt)
 * #endif
 *
 * #if USE_SONAR
 * - void ins_module_int_update_sonar (float distance, float dt)
 * #endif
 *
 * Keep in mind that after the module functions, the ins_ned_to_state function is called; hence,
 * all outputs of the state filter must be written to the ins_ltp struct in order to be written to
 * state.
 *
 * TODO: add update_baro
 */

#ifndef INS_MODULE_INT_H_
#define INS_MODULE_INT_H_


#include "generated/modules.h"


#endif /* INS_MODULE_INT_H_ */

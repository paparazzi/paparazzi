/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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
 *
 */

#include "booz_inter_mcu.h"

#include "booz_imu.h"
#include "booz_ahrs.h"
#include "booz_ins.h"


struct booz_inter_mcu_state inter_mcu_state;

#ifdef BOOZ_FILTER_MCU

//#define LP_GYROS

void inter_mcu_fill_state() {
#if 1
#ifndef LP_GYROS
  inter_mcu_state.r_rates[AXIS_P]  = imu_gyro[AXIS_P] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_Q]  = imu_gyro[AXIS_Q] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_R]  = imu_gyro[AXIS_R] * RATE_PI_S/M_PI;
#else
  inter_mcu_state.r_rates[AXIS_P]  = imu_gyro_lp[AXIS_P] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_Q]  = imu_gyro_lp[AXIS_Q] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_R]  = imu_gyro_lp[AXIS_R] * RATE_PI_S/M_PI;
#endif
  inter_mcu_state.f_rates[AXIS_P]  = booz_ahrs_p * RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_Q]  = booz_ahrs_q * RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_R]  = booz_ahrs_r * RATE_PI_S/M_PI;
  inter_mcu_state.f_eulers[AXIS_X] = booz_ahrs_phi   * ANGLE_PI/M_PI;
  inter_mcu_state.f_eulers[AXIS_Y] = booz_ahrs_theta * ANGLE_PI/M_PI;
  inter_mcu_state.f_eulers[AXIS_Z] = booz_ahrs_psi   * ANGLE_PI/M_PI;
  inter_mcu_state.pos[AXIS_X] = booz_ins_x;
  inter_mcu_state.pos[AXIS_Y] = booz_ins_y;
  inter_mcu_state.pos[AXIS_Z] = booz_ins_z;
  inter_mcu_state.speed[AXIS_X] = booz_ins_xdot;
  inter_mcu_state.speed[AXIS_Y] = booz_ins_ydot;
  inter_mcu_state.speed[AXIS_Z] = booz_ins_zdot;
  inter_mcu_state.status =  booz_ahrs_status;
#else
  inter_mcu_state.r_rates[AXIS_P]  = RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_Q]  = RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_R]  = RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_P]  = RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_Q]  = RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_R]  = RATE_PI_S/M_PI;
  inter_mcu_state.f_eulers[AXIS_X] = ANGLE_PI/M_PI;
  inter_mcu_state.f_eulers[AXIS_Y] = ANGLE_PI/M_PI;
  inter_mcu_state.f_eulers[AXIS_Z] = ANGLE_PI/M_PI;
  inter_mcu_state.status =  1;
#endif /* 0 */
}
#endif

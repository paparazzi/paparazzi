/*
 * Copyright (C) 2013-2020 Chris Efstathiou hendrixgr@gmail.com
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

/**
 * @file modules/uav_recovery/uav_recovery.h
 *
 */

#if !defined(UAV_RECOVERY_H)
#define UAV_RECOVERY_H
#define MY_PARACHUTE_RADIUS DEFAULT_CIRCLE_RADIUS

extern unit_t parachute_compute_approach(uint8_t baseleg, uint8_t release, uint8_t wp_target);
extern float  parachute_start_qdr;
extern bool   deploy_parachute_var;
extern float  airborne_wind_dir;
extern float  airborne_wind_speed;
extern bool   wind_measurements_valid;
extern bool   wind_info_valid;

extern void uav_recovery_init(void);
extern void uav_recovery_periodic(void);
extern uint8_t DeployParachute(void);
extern uint8_t LockParachute(void);
extern uint8_t calculate_wind_no_airspeed(uint8_t wp, float radius, float height);


#define ParachuteComputeApproach(_baseleg, _release, _target) parachute_compute_approach(_baseleg, _release, _target)

#endif

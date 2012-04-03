/*
 * Copyright (C) 2010 ENAC
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
 *  @file firmwares/fixedwing/guidance/guidance_v_n.h
 *  "New" vertical control for fixed wing vehicles.
 *
 */

#ifndef FW_V_CTL_N_H
#define FW_V_CTL_N_H

#define V_CTL_SPEED_THROTTLE    0
#define V_CTL_SPEED_AIRSPEED    1
#define V_CTL_SPEED_GROUNDSPEED 2

extern float v_ctl_auto_pitch_dgain;
extern float v_ctl_auto_airspeed_throttle_pgain;
extern float v_ctl_auto_airspeed_throttle_dgain;
extern float v_ctl_auto_airspeed_throttle_igain;
extern float v_ctl_auto_airspeed_throttle_sum_err;
extern float v_ctl_auto_airspeed_pitch_pgain;
extern float v_ctl_auto_airspeed_pitch_dgain;
extern float v_ctl_auto_airspeed_pitch_igain;
extern float v_ctl_auto_airspeed_pitch_sum_err;

extern uint8_t v_ctl_speed_mode;

extern float v_ctl_pitch_loiter_trim;
extern float v_ctl_pitch_dash_trim;

#endif /* FW_V_CTL_N_H */

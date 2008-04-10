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

#ifndef BOOZ_CONTROL_H
#define BOOZ_CONTROL_H

#include "std.h"
#include "paparazzi.h"

extern void booz_control_init(void);

extern void booz_control_rate_read_setpoints_from_rc(void);
extern void booz_control_rate_run(void);

extern void booz_control_attitude_read_setpoints_from_rc(void);
extern void booz_control_attitude_run(void);

extern float booz_control_p_sp;
extern float booz_control_q_sp;
extern float booz_control_r_sp;
extern float booz_control_power_sp;

extern float booz_control_rate_pq_pgain;
extern float booz_control_rate_pq_dgain;
extern float booz_control_rate_r_pgain;
extern float booz_control_rate_r_dgain;

extern float booz_control_attitude_phi_sp;
extern float booz_control_attitude_theta_sp;
extern float booz_control_attitude_psi_sp;

extern float booz_control_attitude_phi_theta_pgain;
extern float booz_control_attitude_phi_theta_dgain;
extern float booz_control_attitude_psi_pgain;
extern float booz_control_attitude_psi_dgain;


#define BoozControlAttitudeSetSetPoints(_phi_sp, _theta_sp, _psi_sp, _power_sp) { \
    booz_control_attitude_phi_sp = _phi_sp;				          \
    booz_control_attitude_theta_sp = _theta_sp;				          \
    booz_control_attitude_psi_sp = _psi_sp;				          \
    booz_control_power_sp = _power_sp;					          \
  }

#include "airframe.h"
extern pprz_t booz_control_commands[COMMANDS_NB];

#endif /* BOOZ_CONTROL_H */

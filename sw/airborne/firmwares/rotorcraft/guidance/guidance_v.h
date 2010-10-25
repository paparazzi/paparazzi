/*
 * $Id$
 *
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

#ifndef GUIDANCE_V
#define GUIDANCE_V

#include "std.h"

#include "firmwares/rotorcraft/guidance/guidance_v_ref.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adpt.h"

#define GUIDANCE_V_MODE_KILL      0
#define GUIDANCE_V_MODE_RC_DIRECT 1
#define GUIDANCE_V_MODE_RC_CLIMB  2
#define GUIDANCE_V_MODE_CLIMB     3
#define GUIDANCE_V_MODE_HOVER     4
#define GUIDANCE_V_MODE_NAV       5

extern uint8_t guidance_v_mode;

extern int32_t guidance_v_z_sp;
extern int32_t guidance_v_zd_sp;
extern int32_t guidance_v_z_ref;
extern int32_t guidance_v_zd_ref;
extern int32_t guidance_v_zdd_ref;
extern int32_t guidance_v_z_sum_err;
extern int32_t guidance_v_ff_cmd;
extern int32_t guidance_v_fb_cmd;
extern int32_t guidance_v_delta_t;

extern int32_t guidance_v_kp;
extern int32_t guidance_v_kd;
extern int32_t guidance_v_ki;

extern void guidance_v_init(void);
extern void guidance_v_read_rc(void);
extern void guidance_v_mode_changed(uint8_t new_mode);
extern void guidance_v_notify_in_flight(bool_t in_flight);
extern void guidance_v_run(bool_t in_flight);

#define guidance_v_SetKi(_val) {			\
    guidance_v_ki = _val;				\
    guidance_v_z_sum_err = 0;			\
  }


#endif /* GUIDANCE_V */

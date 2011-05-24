/*
 * Copyright (C) 2008-2011 Joby Energy Inc
 */
#ifndef QUAT_SETPOINT_H
#define QUAT_SETPOINT_H

#include "stabilization.h"

#include "subsystems/radio_control.h"
#include "math/pprz_algebra.h"

#include "stabilization/stabilization_attitude_ref_int.h"

void stabilization_attitude_sp_enter(void);
void stabilization_attitude_read_rc_incremental(bool_t enable_alpha_vane, bool_t enable_beta_vane);
void stabilization_attitude_read_rc_absolute(struct Int32Eulers sp, bool_t in_flight);
void quat_setpoint_enter_absolute(void);
void booz_stab_att_vane_on(void);
void booz_stab_att_vane_off(void);

#endif

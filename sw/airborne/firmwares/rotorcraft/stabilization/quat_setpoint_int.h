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

void stabilization_attitude_read_rc_absolute(bool_t in_flight);

#endif
